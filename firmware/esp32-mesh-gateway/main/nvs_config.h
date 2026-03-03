/**
 * @file nvs_config.h
 * @brief NVS configuration for the ESP-MESH gateway.
 *
 * Reads mesh network credentials, router credentials, and aggregator
 * target from NVS. Falls back to Kconfig defaults when absent.
 */

#ifndef NVS_CONFIG_H
#define NVS_CONFIG_H

#include <stdint.h>

#define NVS_CFG_SSID_MAX  33
#define NVS_CFG_PASS_MAX  65
#define NVS_CFG_IP_MAX    16
#define NVS_CFG_MESH_ID_LEN 6

typedef struct {
    /* Mesh network */
    uint8_t  mesh_id[NVS_CFG_MESH_ID_LEN]; /**< 6-byte mesh network ID. */
    char     mesh_password[NVS_CFG_PASS_MAX];
    uint8_t  mesh_channel;
    uint8_t  mesh_max_connections;
    uint8_t  mesh_max_layer;

    /* Upstream router (STA connection) */
    char     router_ssid[NVS_CFG_SSID_MAX];
    char     router_password[NVS_CFG_PASS_MAX];

    /* Aggregator target */
    char     aggregator_ip[NVS_CFG_IP_MAX];
    uint16_t aggregator_port;
} gateway_config_t;

/**
 * Load gateway configuration from NVS, falling back to Kconfig defaults.
 * Must be called after nvs_flash_init().
 */
void gateway_config_load(gateway_config_t *cfg);

#endif /* NVS_CONFIG_H */
