/**
 * @file mesh_transport.h
 * @brief Mesh transport layer — send CSI frames via ESP-MESH to root.
 *
 * Replaces direct UDP sending with esp_mesh_send() toward the root node.
 * Includes a ring buffer for local frame buffering when mesh is disconnected.
 */

#ifndef MESH_TRANSPORT_H
#define MESH_TRANSPORT_H

#include <stdint.h>
#include <stddef.h>

/** Ring buffer capacity (number of frames to buffer during disconnection). */
#define MESH_TX_RING_SIZE 100

/** Maximum size of a single buffered frame. */
#define MESH_TX_MAX_FRAME (20 + 4 * 256 * 2)

/**
 * Initialize the mesh transport layer.
 * Allocates the ring buffer for offline buffering.
 */
void mesh_transport_init(void);

/**
 * Send a CSI frame via mesh toward the root node.
 *
 * If the mesh is connected, sends immediately via esp_mesh_send().
 * If disconnected, buffers the frame in the ring buffer and returns 0.
 *
 * @param data  Frame data (ADR-018 binary format).
 * @param len   Frame length in bytes.
 * @return Number of bytes sent (or buffered), or -1 on error.
 */
int mesh_transport_send(const uint8_t *data, size_t len);

/**
 * Flush any buffered frames to the mesh.
 * Called when mesh connectivity is restored.
 */
void mesh_transport_flush(void);

/**
 * Set the mesh connection state.
 * Called from the mesh event handler when parent connected/disconnected.
 *
 * @param connected  true if mesh parent is connected.
 */
void mesh_transport_set_connected(bool connected);

/**
 * Check if mesh transport is currently connected.
 */
bool mesh_transport_is_connected(void);

/**
 * Get the count of frames successfully sent via mesh.
 */
uint32_t mesh_transport_get_send_count(void);

/**
 * Get the count of frames currently buffered (waiting for reconnect).
 */
uint32_t mesh_transport_get_buffer_count(void);

#endif /* MESH_TRANSPORT_H */
