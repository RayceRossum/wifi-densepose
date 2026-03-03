/**
 * @file app_main.c
 * @brief ESP32-S3 Mesh Gateway — ESP-MESH root node + UDP bridge.
 *
 * This firmware runs on a single ESP32-S3 placed near the travel router.
 * It acts as the ESP-MESH root node, receives ADR-018 CSI frames from
 * sensor nodes via the mesh, and forwards them over UDP to the aggregator.
 *
 * The aggregator sees identical frames as if nodes sent directly — it
 * requires zero changes.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "nvs_config.h"
#include "mesh_root.h"

static const char *TAG = "gw_main";

static gateway_config_t s_cfg;

/* Mesh event tracking */
static bool s_mesh_connected = false;
static int  s_mesh_child_count = 0;

/**
 * IP event handler — fires when the root node obtains an IP from the router.
 */
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP from router: " IPSTR, IP2STR(&event->ip_info.ip));
}

/**
 * Mesh event handler — tracks topology changes and connection state.
 */
static void mesh_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    switch (event_id) {
    case MESH_EVENT_STARTED:
        ESP_LOGI(TAG, "Mesh started");
        /* As root, we don't need to find a parent — we ARE the root.
         * The mesh stack will handle router connection automatically. */
        break;

    case MESH_EVENT_STOPPED:
        ESP_LOGW(TAG, "Mesh stopped");
        s_mesh_connected = false;
        break;

    case MESH_EVENT_PARENT_CONNECTED:
        /* For root node, this means we connected to the upstream router */
        ESP_LOGI(TAG, "Root connected to upstream router");
        s_mesh_connected = true;
        break;

    case MESH_EVENT_PARENT_DISCONNECTED:
        ESP_LOGW(TAG, "Root disconnected from upstream router");
        s_mesh_connected = false;
        break;

    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child =
            (mesh_event_child_connected_t *)event_data;
        s_mesh_child_count++;
        ESP_LOGI(TAG, "Child connected: " MACSTR " (aid=%d) — total children: %d",
                 MAC2STR(child->mac), child->aid, s_mesh_child_count);
        break;
    }

    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child =
            (mesh_event_child_disconnected_t *)event_data;
        if (s_mesh_child_count > 0) {
            s_mesh_child_count--;
        }
        ESP_LOGW(TAG, "Child disconnected: " MACSTR " (aid=%d) — total children: %d",
                 MAC2STR(child->mac), child->aid, s_mesh_child_count);
        break;
    }

    case MESH_EVENT_ROUTING_TABLE_ADD:
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_t *rt =
            (mesh_event_routing_table_t *)event_data;
        ESP_LOGI(TAG, "Routing table %s: %d entries changed, total=%d",
                 event_id == MESH_EVENT_ROUTING_TABLE_ADD ? "ADD" : "REMOVE",
                 (int)rt->rt_size_change, (int)rt->rt_size_new);
        break;
    }

    case MESH_EVENT_ROOT_GOT_IP: {
        mesh_event_root_got_ip_t *got_ip =
            (mesh_event_root_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Root got IP: " IPSTR, IP2STR(&got_ip->ip_info.ip));
        break;
    }

    case MESH_EVENT_ROOT_LOST_IP:
        ESP_LOGW(TAG, "Root lost IP");
        break;

    default:
        ESP_LOGD(TAG, "Mesh event: %ld", (long)event_id);
        break;
    }
}

/**
 * Initialize and start ESP-MESH as the root node.
 */
static void mesh_init_root(void)
{
    /* Initialize WiFi (required before mesh) */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Create netif for STA (root needs STA to connect to router) */
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
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); /* Disable power save */
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Initialize mesh */
    ESP_ERROR_CHECK(esp_mesh_init());

    /* Configure mesh */
    mesh_cfg_t mesh_cfg = MESH_INIT_CONFIG_DEFAULT();

    /* Mesh ID */
    memcpy(mesh_cfg.mesh_id.addr, s_cfg.mesh_id, 6);

    /* Channel */
    mesh_cfg.channel = s_cfg.mesh_channel;

    /* Router configuration (upstream AP) */
    mesh_cfg.router.ssid_len = strlen(s_cfg.router_ssid);
    memcpy(mesh_cfg.router.ssid, s_cfg.router_ssid,
           mesh_cfg.router.ssid_len);
    if (strlen(s_cfg.router_password) > 0) {
        memcpy(mesh_cfg.router.password, s_cfg.router_password,
               strlen(s_cfg.router_password));
    }

    /* Mesh AP configuration (how children connect to us) */
    mesh_cfg.mesh_ap.max_connection = s_cfg.mesh_max_connections;
    memcpy(mesh_cfg.mesh_ap.password, s_cfg.mesh_password,
           strlen(s_cfg.mesh_password));

    ESP_ERROR_CHECK(esp_mesh_set_config(&mesh_cfg));

    /* Set this node as fixed root */
    ESP_ERROR_CHECK(esp_mesh_fix_root(true));
    ESP_ERROR_CHECK(esp_mesh_set_type(MESH_ROOT));

    /* Set max mesh layers */
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(s_cfg.mesh_max_layer));

    /* Allow root networking (DHCP from router) */
    ESP_ERROR_CHECK(esp_mesh_set_self_organized(true, false));

    /* Start mesh */
    ESP_ERROR_CHECK(esp_mesh_start());

    ESP_LOGI(TAG, "ESP-MESH root node started on channel %u",
             s_cfg.mesh_channel);
    ESP_LOGI(TAG, "Mesh ID: %02X:%02X:%02X:%02X:%02X:%02X",
             s_cfg.mesh_id[0], s_cfg.mesh_id[1], s_cfg.mesh_id[2],
             s_cfg.mesh_id[3], s_cfg.mesh_id[4], s_cfg.mesh_id[5]);
    ESP_LOGI(TAG, "Router SSID: %s", s_cfg.router_ssid);
}

/**
 * Status reporting task — periodically logs mesh topology info.
 */
static void status_task(void *arg)
{
    (void)arg;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); /* Every 10 seconds */

        int rt_size = esp_mesh_get_routing_table_size();
        bool is_root = esp_mesh_is_root();

        ESP_LOGI(TAG, "=== Gateway Status ===");
        ESP_LOGI(TAG, "  Root: %s | Connected: %s",
                 is_root ? "YES" : "NO",
                 s_mesh_connected ? "YES" : "NO");
        ESP_LOGI(TAG, "  Direct children: %d | Routing table: %d nodes",
                 s_mesh_child_count, rt_size);
        ESP_LOGI(TAG, "  Frames forwarded: %lu",
                 (unsigned long)mesh_root_get_frame_count());
        ESP_LOGI(TAG, "  Aggregator: %s:%u",
                 s_cfg.aggregator_ip, s_cfg.aggregator_port);
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
    gateway_config_load(&s_cfg);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP32-S3 Mesh Gateway (Root Node)");
    ESP_LOGI(TAG, " Aggregator: %s:%u",
             s_cfg.aggregator_ip, s_cfg.aggregator_port);
    ESP_LOGI(TAG, "========================================");

    /* Initialize mesh as root */
    mesh_init_root();

    /* Initialize UDP forwarder */
    if (mesh_root_udp_init(s_cfg.aggregator_ip, s_cfg.aggregator_port) != 0) {
        ESP_LOGE(TAG, "Failed to initialize UDP forwarder — halting");
        return;
    }

    /* Start mesh receive + forward task */
    mesh_root_start();

    /* Start topology report task (sends mesh tree to aggregator) */
    mesh_root_start_topology_reports();

    /* Start status reporting task */
    xTaskCreatePinnedToCore(status_task, "gw_status", 3072, NULL,
                            2, NULL, tskNO_AFFINITY);

    ESP_LOGI(TAG, "Gateway operational — waiting for mesh nodes...");
}
