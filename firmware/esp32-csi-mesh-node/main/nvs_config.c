/**
 * @file nvs_config.c
 * @brief NVS configuration for the CSI mesh sensor node.
 */

#include "nvs_config.h"

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "node_nvs";

void mesh_node_config_load(mesh_node_config_t *cfg)
{
    if (cfg == NULL) {
        ESP_LOGE(TAG, "mesh_node_config_load: cfg is NULL");
        return;
    }

    /* Defaults from Kconfig */
    cfg->node_id = (uint8_t)CONFIG_MESH_NODE_ID;

    memcpy(cfg->mesh_id, "RUVIEW", NVS_CFG_MESH_ID_LEN);

    strncpy(cfg->mesh_password, CONFIG_MESH_AP_PASSWORD,
            NVS_CFG_PASS_MAX - 1);
    cfg->mesh_password[NVS_CFG_PASS_MAX - 1] = '\0';

    cfg->mesh_channel         = (uint8_t)CONFIG_MESH_CHANNEL;
    cfg->mesh_max_connections = (uint8_t)CONFIG_MESH_AP_MAX_CONNECTIONS;
    cfg->mesh_max_layer       = (uint8_t)CONFIG_MESH_MAX_LAYER;

    strncpy(cfg->router_ssid, CONFIG_MESH_ROUTER_SSID,
            NVS_CFG_SSID_MAX - 1);
    cfg->router_ssid[NVS_CFG_SSID_MAX - 1] = '\0';

    strncpy(cfg->router_password, CONFIG_MESH_ROUTER_PASSWORD,
            NVS_CFG_PASS_MAX - 1);
    cfg->router_password[NVS_CFG_PASS_MAX - 1] = '\0';

    /* Try to override from NVS namespace "mesh_node" */
    nvs_handle_t handle;
    esp_err_t err = nvs_open("mesh_node", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No NVS config found, using compiled defaults");
        return;
    }

    size_t len;
    char buf[NVS_CFG_PASS_MAX];

    /* Node ID */
    uint8_t nid;
    if (nvs_get_u8(handle, "node_id", &nid) == ESP_OK) {
        cfg->node_id = nid;
        ESP_LOGI(TAG, "NVS: node_id=%u", nid);
    }

    /* Mesh ID (6-byte blob) */
    uint8_t mid[NVS_CFG_MESH_ID_LEN];
    len = NVS_CFG_MESH_ID_LEN;
    if (nvs_get_blob(handle, "mesh_id", mid, &len) == ESP_OK &&
        len == NVS_CFG_MESH_ID_LEN) {
        memcpy(cfg->mesh_id, mid, NVS_CFG_MESH_ID_LEN);
        ESP_LOGI(TAG, "NVS: mesh_id=%.*s", NVS_CFG_MESH_ID_LEN,
                 (char *)cfg->mesh_id);
    }

    /* Mesh password */
    len = sizeof(buf);
    if (nvs_get_str(handle, "mesh_pass", buf, &len) == ESP_OK && len > 1) {
        strncpy(cfg->mesh_password, buf, NVS_CFG_PASS_MAX - 1);
        cfg->mesh_password[NVS_CFG_PASS_MAX - 1] = '\0';
        ESP_LOGI(TAG, "NVS: mesh_pass=***");
    }

    /* Mesh channel */
    uint8_t ch;
    if (nvs_get_u8(handle, "mesh_chan", &ch) == ESP_OK && ch >= 1 && ch <= 13) {
        cfg->mesh_channel = ch;
        ESP_LOGI(TAG, "NVS: mesh_channel=%u", ch);
    }

    /* Router SSID */
    len = sizeof(buf);
    if (nvs_get_str(handle, "router_ssid", buf, &len) == ESP_OK && len > 1) {
        strncpy(cfg->router_ssid, buf, NVS_CFG_SSID_MAX - 1);
        cfg->router_ssid[NVS_CFG_SSID_MAX - 1] = '\0';
        ESP_LOGI(TAG, "NVS: router_ssid=%s", cfg->router_ssid);
    }

    /* Router password */
    len = sizeof(buf);
    if (nvs_get_str(handle, "router_pass", buf, &len) == ESP_OK) {
        strncpy(cfg->router_password, buf, NVS_CFG_PASS_MAX - 1);
        cfg->router_password[NVS_CFG_PASS_MAX - 1] = '\0';
        ESP_LOGI(TAG, "NVS: router_pass=***");
    }

    nvs_close(handle);
}
