/**
 * @file csi_capture.c
 * @brief CSI data capture and ADR-018 binary frame serialization.
 *
 * Ported from the original esp32-csi-node/csi_collector.c. The only
 * change from the original: frames are dispatched via mesh_transport_send()
 * instead of stream_sender_send() (UDP). The ADR-018 binary frame format
 * is identical — the aggregator cannot tell the difference.
 */

#include "csi_capture.h"
#include "mesh_transport.h"

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "sdkconfig.h"

static const char *TAG = "csi_cap";

static uint8_t  s_node_id    = 1;
static uint32_t s_sequence   = 0;
static uint32_t s_cb_count   = 0;
static uint32_t s_frame_count = 0;

/**
 * Serialize CSI data into ADR-018 binary frame format.
 *
 * Layout (identical to original firmware):
 *   [0..3]   Magic: 0xC5110001 (LE)
 *   [4]      Node ID
 *   [5]      Number of antennas
 *   [6..7]   Number of subcarriers (LE u16)
 *   [8..11]  Frequency MHz (LE u32)
 *   [12..15] Sequence number (LE u32)
 *   [16]     RSSI (i8)
 *   [17]     Noise floor (i8)
 *   [18..19] Reserved
 *   [20..]   I/Q data (raw bytes from ESP-IDF callback)
 */
static size_t serialize_frame(const wifi_csi_info_t *info,
                              uint8_t *buf, size_t buf_len)
{
    if (info == NULL || buf == NULL || info->buf == NULL) {
        return 0;
    }

    uint8_t n_antennas = 1;
    uint16_t iq_len = (uint16_t)info->len;
    uint16_t n_subcarriers = iq_len / (2 * n_antennas);

    size_t frame_size = CSI_HEADER_SIZE + iq_len;
    if (frame_size > buf_len) {
        ESP_LOGW(TAG, "Buffer too small: need %u, have %u",
                 (unsigned)frame_size, (unsigned)buf_len);
        return 0;
    }

    /* Derive frequency from channel */
    uint8_t channel = info->rx_ctrl.channel;
    uint32_t freq_mhz;
    if (channel >= 1 && channel <= 13) {
        freq_mhz = 2412 + (channel - 1) * 5;
    } else if (channel == 14) {
        freq_mhz = 2484;
    } else if (channel >= 36 && channel <= 177) {
        freq_mhz = 5000 + channel * 5;
    } else {
        freq_mhz = 0;
    }

    /* Magic (LE) */
    uint32_t magic = CSI_MAGIC;
    memcpy(&buf[0], &magic, 4);

    /* Node ID — from NVS config, not Kconfig */
    buf[4] = s_node_id;

    /* Number of antennas */
    buf[5] = n_antennas;

    /* Number of subcarriers (LE u16) */
    memcpy(&buf[6], &n_subcarriers, 2);

    /* Frequency MHz (LE u32) */
    memcpy(&buf[8], &freq_mhz, 4);

    /* Sequence number (LE u32) */
    uint32_t seq = s_sequence++;
    memcpy(&buf[12], &seq, 4);

    /* RSSI (i8) */
    buf[16] = (uint8_t)(int8_t)info->rx_ctrl.rssi;

    /* Noise floor (i8) */
    buf[17] = (uint8_t)(int8_t)info->rx_ctrl.noise_floor;

    /* Reserved */
    buf[18] = 0;
    buf[19] = 0;

    /* I/Q data */
    memcpy(&buf[CSI_HEADER_SIZE], info->buf, iq_len);

    return frame_size;
}

/**
 * WiFi CSI callback — invoked by ESP-IDF when CSI data is available.
 * Serializes to ADR-018 format and sends via mesh transport.
 */
static void wifi_csi_callback(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;
    s_cb_count++;

    if (s_cb_count <= 3 || (s_cb_count % 200) == 0) {
        ESP_LOGI(TAG, "CSI cb #%lu: len=%d rssi=%d ch=%d mac=%02X:%02X:%02X:%02X:%02X:%02X",
                 (unsigned long)s_cb_count, info->len,
                 info->rx_ctrl.rssi, info->rx_ctrl.channel,
                 info->mac[0], info->mac[1], info->mac[2],
                 info->mac[3], info->mac[4], info->mac[5]);
    }

    uint8_t frame_buf[CSI_MAX_FRAME_SIZE];
    size_t frame_len = serialize_frame(info, frame_buf, sizeof(frame_buf));

    if (frame_len > 0) {
        int ret = mesh_transport_send(frame_buf, frame_len);
        if (ret >= 0) {
            s_frame_count++;
        }
    }
}

/**
 * Promiscuous mode callback — required for CSI to fire on all frames.
 * The mesh traffic (beacons, data relay) generates the radio activity
 * that creates CSI capture opportunities.
 */
static void wifi_promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    (void)buf;
    (void)type;
}

void csi_capture_init(uint8_t node_id)
{
    s_node_id = node_id;

    /* Enable promiscuous mode for CSI on all frames.
     *
     * Note on mesh coexistence: ESP-MESH uses the WiFi radio for mesh
     * management and data relay. Promiscuous mode captures CSI from
     * ALL frames on the channel, including mesh traffic. The constant
     * mesh management frames (beacons, routing updates, data relay)
     * serve as the sensing signal — replacing the need for explicit
     * ping traffic used in the original firmware. */
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_cb));

    wifi_promiscuous_filter_t filt = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT |
                       WIFI_PROMIS_FILTER_MASK_DATA,
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filt));

    ESP_LOGI(TAG, "Promiscuous mode enabled for CSI capture");

    /* Configure CSI collection */
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = false,
        .manu_scale        = false,
        .shift             = false,
    };

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_callback, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    ESP_LOGI(TAG, "CSI capture initialized (node_id=%u, channel=%u)",
             node_id, (unsigned)CONFIG_MESH_CHANNEL);
}

uint32_t csi_capture_get_cb_count(void)
{
    return s_cb_count;
}

uint32_t csi_capture_get_frame_count(void)
{
    return s_frame_count;
}
