/**
 * @file mesh_root.c
 * @brief Mesh root receive loop and UDP forwarding.
 *
 * Receives ADR-018 binary CSI frames from mesh child nodes and forwards
 * them to the aggregator over UDP. Frames are forwarded byte-for-byte —
 * the aggregator receives identical data as if nodes sent directly.
 */

#include "mesh_root.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "mesh_root";

/** Maximum frame size: ADR-018 header (20) + 4 antennas * 256 subcarriers * 2 */
#define MAX_FRAME_SIZE (20 + 4 * 256 * 2)

/** Receive buffer — slightly larger to catch oversized frames */
#define RX_BUF_SIZE (MAX_FRAME_SIZE + 64)

static int s_udp_sock = -1;
static struct sockaddr_in s_dest_addr;
static uint32_t s_frame_count = 0;
static uint32_t s_fwd_errors  = 0;

int mesh_root_udp_init(const char *ip, uint16_t port)
{
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_udp_sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket: errno %d", errno);
        return -1;
    }

    memset(&s_dest_addr, 0, sizeof(s_dest_addr));
    s_dest_addr.sin_family = AF_INET;
    s_dest_addr.sin_port   = htons(port);

    if (inet_pton(AF_INET, ip, &s_dest_addr.sin_addr) <= 0) {
        ESP_LOGE(TAG, "Invalid aggregator IP: %s", ip);
        close(s_udp_sock);
        s_udp_sock = -1;
        return -1;
    }

    ESP_LOGI(TAG, "UDP forwarder initialized: %s:%u", ip, port);
    return 0;
}

/**
 * FreeRTOS task: receive mesh data and forward via UDP.
 *
 * This task blocks on esp_mesh_recv() waiting for frames from child
 * nodes. Each received frame (which is an ADR-018 binary CSI frame)
 * is forwarded as-is to the aggregator over UDP.
 */
static void mesh_recv_task(void *arg)
{
    (void)arg;

    uint8_t rx_buf[RX_BUF_SIZE];
    mesh_addr_t from;
    mesh_data_t data;
    int flag = 0;

    ESP_LOGI(TAG, "Mesh receive task started — forwarding to aggregator");

    while (1) {
        data.data = rx_buf;
        data.size = sizeof(rx_buf);

        esp_err_t err = esp_mesh_recv(&from, &data, portMAX_DELAY,
                                      &flag, NULL, 0);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_mesh_recv error: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (data.size < 20) {
            /* Too small to be a valid ADR-018 frame — skip */
            continue;
        }

        /* Validate ADR-018 magic number: 0xC5110001 (LE) */
        uint32_t magic;
        memcpy(&magic, rx_buf, 4);
        if (magic != 0xC5110001) {
            /* Not a CSI frame — could be mesh control data, skip */
            continue;
        }

        /* Forward the frame byte-for-byte via UDP */
        if (s_udp_sock >= 0) {
            int sent = sendto(s_udp_sock, rx_buf, data.size, 0,
                              (struct sockaddr *)&s_dest_addr,
                              sizeof(s_dest_addr));
            if (sent > 0) {
                s_frame_count++;
                if (s_frame_count <= 3 || (s_frame_count % 500) == 0) {
                    uint8_t node_id = rx_buf[4];
                    ESP_LOGI(TAG, "Forwarded frame #%lu from node %u (%d bytes)",
                             (unsigned long)s_frame_count, node_id,
                             (int)data.size);
                }
            } else {
                s_fwd_errors++;
                if (s_fwd_errors <= 5 || (s_fwd_errors % 100) == 0) {
                    ESP_LOGW(TAG, "UDP sendto failed: errno %d (errors: %lu)",
                             errno, (unsigned long)s_fwd_errors);
                }
            }
        }
    }
}

void mesh_root_start(void)
{
    xTaskCreatePinnedToCore(mesh_recv_task, "mesh_recv", 4096, NULL,
                            5, NULL, tskNO_AFFINITY);
}

uint32_t mesh_root_get_frame_count(void)
{
    return s_frame_count;
}
