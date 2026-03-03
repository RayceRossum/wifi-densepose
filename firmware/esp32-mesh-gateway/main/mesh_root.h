/**
 * @file mesh_root.h
 * @brief Mesh root receive loop and UDP forwarding.
 *
 * Runs a FreeRTOS task that receives CSI frames from mesh child nodes
 * and forwards them via UDP to the aggregator. The frames are ADR-018
 * binary format — forwarded byte-for-byte with no modification.
 */

#ifndef MESH_ROOT_H
#define MESH_ROOT_H

#include <stdint.h>

/**
 * Initialize the UDP forwarder socket.
 *
 * @param ip   Aggregator IP address string.
 * @param port Aggregator UDP port.
 * @return 0 on success, -1 on error.
 */
int mesh_root_udp_init(const char *ip, uint16_t port);

/**
 * Start the mesh receive + UDP forward task.
 * Spawns a FreeRTOS task that blocks on esp_mesh_recv() and forwards
 * each received frame to the aggregator via UDP.
 */
void mesh_root_start(void);

/**
 * Get the count of frames forwarded since boot.
 */
uint32_t mesh_root_get_frame_count(void);

/**
 * Start the periodic topology report task.
 *
 * Sends a compact topology report (magic 0xC5110002) to the aggregator
 * every 10 seconds over the same UDP socket. Contains per-node entries
 * with node_id, parent_id, layer, child_count, and role.
 *
 * The aggregator distinguishes these from CSI frames by magic number.
 */
void mesh_root_start_topology_reports(void);

#endif /* MESH_ROOT_H */
