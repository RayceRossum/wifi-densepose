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
                uint8_t node_id = rx_buf[4];
                topo_record_node(node_id);
                if (s_frame_count <= 3 || (s_frame_count % 500) == 0) {
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

/* ---- Topology report (magic 0xC5110002) ---- */

/**
 * Topology report wire format:
 *
 *   [0..4]   Magic: 0xC5110002 (LE)
 *   [4]      Report type: 0x01 (topology)
 *   [5]      Node count (N)
 *   [6]      Max layer depth
 *   [7]      Reserved
 *   -- per node (8 bytes each, N entries): --
 *   [+0]     Node ID (u8) — extracted from CSI frames' node_id field
 *   [+1]     Parent ID (u8, 0xFF = root / no parent)
 *   [+2]     Layer (u8)
 *   [+3]     Child count (u8)
 *   [+4]     Role (u8: 0=root, 1=relay, 2=leaf)
 *   [+5..+7] Reserved
 *
 * Total: 8 + N*8 bytes. For 12 nodes: 104 bytes.
 *
 * ESP-MESH only exposes the routing table (list of MAC addresses).
 * We don't have per-node layer/parent info at the root, so we report
 * the routing table size and the direct children we can see. The
 * aggregator merges this with CSI frame arrival data.
 *
 * Note: ESP-MESH's esp_mesh_get_routing_table() gives us the MAC
 * addresses of all mesh nodes, but NOT their node_id (which is an
 * application-layer concept stored in NVS). The gateway can only
 * learn node_ids from the CSI frames it forwards. So the topology
 * report focuses on what the gateway knows: its own identity as root,
 * the total routing table size, and per-node entries built from the
 * CSI frames it has seen.
 */

#define TOPO_MAGIC     0xC5110002
#define TOPO_HEADER_SIZE 8
#define TOPO_ENTRY_SIZE  8
#define TOPO_MAX_NODES   32

/** Per-node entry seen via CSI frame forwarding. */
typedef struct {
    uint8_t  node_id;
    uint32_t last_seen_tick;  /* xTaskGetTickCount() at last frame */
    uint32_t frame_count;
} topo_node_entry_t;

static topo_node_entry_t s_topo_nodes[TOPO_MAX_NODES];
static uint8_t s_topo_node_count = 0;

/**
 * Called from mesh_recv_task when forwarding a CSI frame.
 * Tracks which node_ids we've seen for topology reports.
 */
static void topo_record_node(uint8_t node_id)
{
    /* Search existing entries */
    for (uint8_t i = 0; i < s_topo_node_count; i++) {
        if (s_topo_nodes[i].node_id == node_id) {
            s_topo_nodes[i].last_seen_tick = xTaskGetTickCount();
            s_topo_nodes[i].frame_count++;
            return;
        }
    }

    /* New node — add if space */
    if (s_topo_node_count < TOPO_MAX_NODES) {
        s_topo_nodes[s_topo_node_count].node_id = node_id;
        s_topo_nodes[s_topo_node_count].last_seen_tick = xTaskGetTickCount();
        s_topo_nodes[s_topo_node_count].frame_count = 1;
        s_topo_node_count++;
        ESP_LOGI(TAG, "Topology: new node %u discovered (total: %u)",
                 node_id, s_topo_node_count);
    }
}

/**
 * Build and send a topology report over UDP.
 */
static void topo_send_report(void)
{
    if (s_udp_sock < 0) return;

    int rt_size = esp_mesh_get_routing_table_size();
    uint8_t max_layer = 0; /* Root doesn't know layers directly */

    /* Count alive nodes (seen within last 30 seconds) */
    TickType_t now = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(30000);
    uint8_t alive_count = 0;

    for (uint8_t i = 0; i < s_topo_node_count; i++) {
        if ((now - s_topo_nodes[i].last_seen_tick) < timeout) {
            alive_count++;
        }
    }

    /* Build report: header + root entry + sensor node entries */
    uint8_t node_count = 1 + alive_count; /* root + alive sensors */
    size_t report_size = TOPO_HEADER_SIZE + node_count * TOPO_ENTRY_SIZE;
    uint8_t report[TOPO_HEADER_SIZE + (1 + TOPO_MAX_NODES) * TOPO_ENTRY_SIZE];

    /* Header */
    uint32_t magic = TOPO_MAGIC;
    memcpy(&report[0], &magic, 4);
    report[4] = 0x01; /* report type: topology */
    report[5] = node_count;
    report[6] = max_layer;
    report[7] = 0; /* reserved */

    /* Root entry (gateway itself) */
    uint8_t *entry = &report[TOPO_HEADER_SIZE];
    entry[0] = 0;    /* node_id: 0 for gateway */
    entry[1] = 0xFF; /* parent_id: none (root) */
    entry[2] = 0;    /* layer: 0 */
    entry[3] = (uint8_t)(rt_size > 0 ? rt_size - 1 : 0); /* children ≈ rt - self */
    entry[4] = 0;    /* role: root */
    entry[5] = 0; entry[6] = 0; entry[7] = 0; /* reserved */

    /* Sensor node entries */
    uint8_t written = 0;
    for (uint8_t i = 0; i < s_topo_node_count && written < alive_count; i++) {
        if ((now - s_topo_nodes[i].last_seen_tick) >= timeout) {
            continue; /* Skip dead nodes */
        }

        entry = &report[TOPO_HEADER_SIZE + (1 + written) * TOPO_ENTRY_SIZE];
        entry[0] = s_topo_nodes[i].node_id;
        entry[1] = 0xFF; /* parent unknown at root — aggregator infers */
        entry[2] = 0xFF; /* layer unknown at root */
        entry[3] = 0;    /* child count unknown at root */
        entry[4] = 2;    /* role: leaf (default, aggregator can refine) */
        entry[5] = 0; entry[6] = 0; entry[7] = 0;
        written++;
    }

    int sent = sendto(s_udp_sock, report, report_size, 0,
                      (struct sockaddr *)&s_dest_addr,
                      sizeof(s_dest_addr));
    if (sent > 0) {
        ESP_LOGI(TAG, "Topology report sent: %u nodes, %d routing entries, %d bytes",
                 node_count, rt_size, sent);
    } else {
        ESP_LOGW(TAG, "Topology report send failed: errno %d", errno);
    }
}

/**
 * FreeRTOS task: send topology reports every 10 seconds.
 */
static void topo_report_task(void *arg)
{
    (void)arg;

    /* Wait for mesh to settle before first report */
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "Topology report task started (interval: 10s)");

    while (1) {
        topo_send_report();
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void mesh_root_start_topology_reports(void)
{
    xTaskCreatePinnedToCore(topo_report_task, "topo_rpt", 3072, NULL,
                            2, NULL, tskNO_AFFINITY);
}
