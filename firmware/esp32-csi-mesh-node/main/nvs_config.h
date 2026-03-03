/**
 * @file nvs_config.h
 * @brief NVS configuration for the CSI mesh sensor node.
 *
 * Reads mesh network credentials and node identity from NVS.
 * Falls back to Kconfig defaults when absent.
 */

#ifndef NVS_CONFIG_H
#define NVS_CONFIG_H

#include <stdint.h>

#define NVS_CFG_SSID_MAX    33
#define NVS_CFG_PASS_MAX    65
#define NVS_CFG_MESH_ID_LEN 6

typedef struct {
    /* Node identity */
    uint8_t  node_id;

    /* Mesh network */
    uint8_t  mesh_id[NVS_CFG_MESH_ID_LEN]; /**< 6-byte mesh network ID. */
    char     mesh_password[NVS_CFG_PASS_MAX];
    uint8_t  mesh_channel;
    uint8_t  mesh_max_connections;
    uint8_t  mesh_max_layer;

    /* Router SSID/pass — needed by mesh config even for non-root nodes */
    char     router_ssid[NVS_CFG_SSID_MAX];
    char     router_password[NVS_CFG_PASS_MAX];
} mesh_node_config_t;

/**
 * Load sensor node configuration from NVS, falling back to Kconfig defaults.
 * Must be called after nvs_flash_init().
 */
void mesh_node_config_load(mesh_node_config_t *cfg);

#endif /* NVS_CONFIG_H */
