/**
 * @file mesh_transport.c
 * @brief Mesh transport layer — send CSI frames via ESP-MESH to root.
 *
 * Replaces the UDP sender from the original firmware. CSI frames are sent
 * via esp_mesh_send() toward the root node, which then bridges them to
 * the aggregator over UDP. Includes a ring buffer for buffering frames
 * when the mesh connection is temporarily lost.
 */

#include "mesh_transport.h"

#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_mesh.h"

static const char *TAG = "mesh_tx";

/* ---- Ring buffer for offline buffering ---- */

typedef struct {
    uint8_t  data[MESH_TX_MAX_FRAME];
    uint16_t len;
} ring_entry_t;

static ring_entry_t s_ring[MESH_TX_RING_SIZE];
static uint32_t s_ring_head = 0;  /* Next write position */
static uint32_t s_ring_tail = 0;  /* Next read position */
static uint32_t s_ring_count = 0; /* Number of entries in buffer */
static SemaphoreHandle_t s_ring_mutex = NULL;

/* ---- State ---- */

static bool     s_connected  = false;
static uint32_t s_send_count = 0;
static uint32_t s_send_errors = 0;
static uint32_t s_buffer_drops = 0;

void mesh_transport_init(void)
{
    s_ring_head  = 0;
    s_ring_tail  = 0;
    s_ring_count = 0;
    s_connected  = false;

    s_ring_mutex = xSemaphoreCreateMutex();
    if (s_ring_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create ring buffer mutex");
    }

    ESP_LOGI(TAG, "Mesh transport initialized (ring buffer: %d frames)",
             MESH_TX_RING_SIZE);
}

/**
 * Internal: try to send a frame via esp_mesh_send() to root.
 * Returns true on success, false on failure.
 */
static bool send_to_root(const uint8_t *data, size_t len)
{
    mesh_data_t mesh_data;
    mesh_data.data  = (uint8_t *)data; /* esp_mesh_send takes non-const */
    mesh_data.size  = (int)len;
    mesh_data.proto = MESH_PROTO_BIN;
    mesh_data.tos   = MESH_TOS_P2P;

    /* NULL destination = send to root */
    esp_err_t err = esp_mesh_send(NULL, &mesh_data, MESH_DATA_P2P, NULL, 0);
    if (err == ESP_OK) {
        return true;
    }

    if (err != ESP_ERR_MESH_DISCONNECTED) {
        s_send_errors++;
        if (s_send_errors <= 5 || (s_send_errors % 100) == 0) {
            ESP_LOGW(TAG, "esp_mesh_send failed: %s (errors: %lu)",
                     esp_err_to_name(err), (unsigned long)s_send_errors);
        }
    }
    return false;
}

/**
 * Internal: add a frame to the ring buffer.
 * Overwrites oldest entry if full (ring buffer semantics).
 */
static void ring_push(const uint8_t *data, size_t len)
{
    if (s_ring_mutex == NULL) return;
    if (len > MESH_TX_MAX_FRAME) return;

    xSemaphoreTake(s_ring_mutex, portMAX_DELAY);

    memcpy(s_ring[s_ring_head].data, data, len);
    s_ring[s_ring_head].len = (uint16_t)len;
    s_ring_head = (s_ring_head + 1) % MESH_TX_RING_SIZE;

    if (s_ring_count < MESH_TX_RING_SIZE) {
        s_ring_count++;
    } else {
        /* Overwrite oldest — advance tail */
        s_ring_tail = (s_ring_tail + 1) % MESH_TX_RING_SIZE;
        s_buffer_drops++;
        if (s_buffer_drops <= 3 || (s_buffer_drops % 100) == 0) {
            ESP_LOGW(TAG, "Ring buffer full — dropped oldest frame (drops: %lu)",
                     (unsigned long)s_buffer_drops);
        }
    }

    xSemaphoreGive(s_ring_mutex);
}

int mesh_transport_send(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return -1;
    }

    if (s_connected && esp_mesh_is_root() == false) {
        /* Try direct send */
        if (send_to_root(data, len)) {
            s_send_count++;
            if (s_send_count <= 3 || (s_send_count % 500) == 0) {
                ESP_LOGI(TAG, "Sent frame #%lu via mesh (%d bytes)",
                         (unsigned long)s_send_count, (int)len);
            }
            return (int)len;
        }
    }

    /* Mesh not ready or send failed — buffer locally */
    ring_push(data, len);
    return 0;
}

void mesh_transport_flush(void)
{
    if (s_ring_mutex == NULL || s_ring_count == 0) return;

    ESP_LOGI(TAG, "Flushing %lu buffered frames to mesh",
             (unsigned long)s_ring_count);

    uint32_t flushed = 0;

    xSemaphoreTake(s_ring_mutex, portMAX_DELAY);

    while (s_ring_count > 0) {
        ring_entry_t *entry = &s_ring[s_ring_tail];

        /* Release mutex briefly for the send */
        xSemaphoreGive(s_ring_mutex);

        if (send_to_root(entry->data, entry->len)) {
            s_send_count++;
            flushed++;
        } else {
            /* Send failed — stop flushing, try again later */
            ESP_LOGW(TAG, "Flush stopped — send failed after %lu frames",
                     (unsigned long)flushed);
            return;
        }

        xSemaphoreTake(s_ring_mutex, portMAX_DELAY);
        s_ring_tail = (s_ring_tail + 1) % MESH_TX_RING_SIZE;
        s_ring_count--;
    }

    xSemaphoreGive(s_ring_mutex);

    ESP_LOGI(TAG, "Flush complete: %lu frames sent", (unsigned long)flushed);
}

void mesh_transport_set_connected(bool connected)
{
    bool was_disconnected = !s_connected;
    s_connected = connected;

    if (connected && was_disconnected && s_ring_count > 0) {
        /* Just reconnected and we have buffered frames — flush them */
        mesh_transport_flush();
    }
}

bool mesh_transport_is_connected(void)
{
    return s_connected;
}

uint32_t mesh_transport_get_send_count(void)
{
    return s_send_count;
}

uint32_t mesh_transport_get_buffer_count(void)
{
    return s_ring_count;
}
