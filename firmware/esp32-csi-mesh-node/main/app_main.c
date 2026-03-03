/**
 * @file app_main.c
 * @brief ESP32-S3 CSI Mesh Node — sensor + mesh relay.
 *
 * This firmware runs on every sensor node in the field. It:
 * 1. Joins an ESP-MESH network (auto-selects parent toward root)
 * 2. Captures CSI data from all WiFi frames on the mesh channel
 * 3. Serializes CSI into ADR-018 binary format
 * 4. Sends frames via esp_mesh_send() toward the root (gateway)
 * 5. Acts as a relay for frames from deeper nodes (ESP-MESH automatic)
 *
 * The mesh traffic itself (beacons, routing, data relay) generates the
 * radio activity that creates CSI capture opportunities, replacing the
 * need for explicit ping traffic.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "nvs_config.h"
#include "mesh_transport.h"
#include "csi_capture.h"

static const char *TAG = "node_main";

static mesh_node_config_t s_cfg;
static bool s_csi_started = false;

/**
 * Mesh event handler — tracks parent connection and triggers CSI start.
 */
static void mesh_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    switch (event_id) {
    case MESH_EVENT_STARTED:
        ESP_LOGI(TAG, "Mesh started — searching for parent...");
        break;

    case MESH_EVENT_STOPPED:
        ESP_LOGW(TAG, "Mesh stopped");
        mesh_transport_set_connected(false);
        break;

    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *conn =
            (mesh_event_connected_t *)event_data;
        ESP_LOGI(TAG, "Connected to parent: " MACSTR " (layer=%d, type=%d)",
                 MAC2STR(conn->connected.bssid),
                 (int)esp_mesh_get_layer(),
                 (int)conn->self_type);

        mesh_transport_set_connected(true);

        /* Start CSI capture on first connection */
        if (!s_csi_started) {
            csi_capture_init(s_cfg.node_id);
            s_csi_started = true;
            ESP_LOGI(TAG, "CSI capture started after mesh connection");
        }
        break;
    }

    case MESH_EVENT_PARENT_DISCONNECTED:
        ESP_LOGW(TAG, "Disconnected from parent — frames will be buffered");
        mesh_transport_set_connected(false);
        break;

    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child =
            (mesh_event_child_connected_t *)event_data;
        ESP_LOGI(TAG, "Child connected: " MACSTR " (aid=%d)",
                 MAC2STR(child->mac), child->aid);
        break;
    }

    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child =
            (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Child disconnected: " MACSTR " (aid=%d)",
                 MAC2STR(child->mac), child->aid);
        break;
    }

    case MESH_EVENT_ROUTING_TABLE_ADD:
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_t *rt =
            (mesh_event_routing_table_t *)event_data;
        ESP_LOGI(TAG, "Routing table %s: %d changed, total=%d",
                 event_id == MESH_EVENT_ROUTING_TABLE_ADD ? "ADD" : "REMOVE",
                 (int)rt->rt_size_change, (int)rt->rt_size_new);
        break;
    }

    case MESH_EVENT_NO_PARENT_FOUND:
        ESP_LOGW(TAG, "No mesh parent found — will keep scanning");
        break;

    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *lc =
            (mesh_event_layer_change_t *)event_data;
        ESP_LOGI(TAG, "Layer changed to %d", (int)lc->new_layer);
        break;
    }

    default:
        ESP_LOGD(TAG, "Mesh event: %ld", (long)event_id);
        break;
    }
}

/**
 * IP event handler — not typically used by non-root mesh nodes, but
 * included for completeness in case this node becomes root.
 */
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
}

/**
 * Initialize and start ESP-MESH as a non-root node.
 */
static void mesh_init_node(void)
{
    /* Initialize network stack */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Create STA netif (needed by mesh internals) */
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    /* Register event handlers */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL, NULL));

    /* Start WiFi */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Initialize mesh */
    ESP_ERROR_CHECK(esp_mesh_init());

    /* Configure mesh */
    mesh_cfg_t mesh_cfg = MESH_INIT_CONFIG_DEFAULT();

    /* Mesh ID — must match gateway */
    memcpy(mesh_cfg.mesh_id.addr, s_cfg.mesh_id, 6);

    /* Channel — must match gateway */
    mesh_cfg.channel = s_cfg.mesh_channel;

    /* Router config — even non-root nodes need this for mesh init */
    mesh_cfg.router.ssid_len = strlen(s_cfg.router_ssid);
    memcpy(mesh_cfg.router.ssid, s_cfg.router_ssid,
           mesh_cfg.router.ssid_len);
    if (strlen(s_cfg.router_password) > 0) {
        memcpy(mesh_cfg.router.password, s_cfg.router_password,
               strlen(s_cfg.router_password));
    }

    /* Mesh AP config — allows this node to be a relay parent */
    mesh_cfg.mesh_ap.max_connection = s_cfg.mesh_max_connections;
    memcpy(mesh_cfg.mesh_ap.password, s_cfg.mesh_password,
           strlen(s_cfg.mesh_password));

    ESP_ERROR_CHECK(esp_mesh_set_config(&mesh_cfg));

    /* This is NOT the root — let it auto-select parent */
    ESP_ERROR_CHECK(esp_mesh_fix_root(true));
    /* Note: fix_root(true) means root is externally designated. Non-root
     * nodes still participate normally — they just won't try to become root
     * through election, since the gateway is the fixed root. */

    /* Set max mesh layers */
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(s_cfg.mesh_max_layer));

    /* Self-organized networking */
    ESP_ERROR_CHECK(esp_mesh_set_self_organized(true, false));

    /* Start mesh */
    ESP_ERROR_CHECK(esp_mesh_start());

    ESP_LOGI(TAG, "ESP-MESH node started on channel %u",
             s_cfg.mesh_channel);
    ESP_LOGI(TAG, "Mesh ID: %02X:%02X:%02X:%02X:%02X:%02X",
             s_cfg.mesh_id[0], s_cfg.mesh_id[1], s_cfg.mesh_id[2],
             s_cfg.mesh_id[3], s_cfg.mesh_id[4], s_cfg.mesh_id[5]);
}

/**
 * Status reporting task — periodically logs node state.
 */
static void status_task(void *arg)
{
    (void)arg;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); /* Every 10 seconds */

        int layer = esp_mesh_get_layer();
        bool connected = mesh_transport_is_connected();

        ESP_LOGI(TAG, "=== Node %u Status ===", s_cfg.node_id);
        ESP_LOGI(TAG, "  Mesh: %s | Layer: %d",
                 connected ? "CONNECTED" : "DISCONNECTED", layer);
        ESP_LOGI(TAG, "  CSI callbacks: %lu | Frames sent: %lu",
                 (unsigned long)csi_capture_get_cb_count(),
                 (unsigned long)mesh_transport_get_send_count());
        ESP_LOGI(TAG, "  Buffered frames: %lu",
                 (unsigned long)mesh_transport_get_buffer_count());
        ESP_LOGI(TAG, "  Free heap: %lu bytes",
                 (unsigned long)esp_get_free_heap_size());
    }
}

void app_main(void)
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Load configuration */
    mesh_node_config_load(&s_cfg);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP32-S3 CSI Mesh Node (Sensor+Relay)");
    ESP_LOGI(TAG, " Node ID: %u", s_cfg.node_id);
    ESP_LOGI(TAG, "========================================");

    /* Initialize mesh transport (ring buffer) */
    mesh_transport_init();

    /* Initialize and start ESP-MESH */
    mesh_init_node();

    /* CSI capture starts after mesh connection (see mesh_event_handler).
     * This ensures the mesh radio is operational before we enable
     * promiscuous mode for CSI. */

    /* Start status reporting task */
    xTaskCreatePinnedToCore(status_task, "node_status", 3072, NULL,
                            2, NULL, tskNO_AFFINITY);

    ESP_LOGI(TAG, "Node operational — searching for mesh parent...");
}
