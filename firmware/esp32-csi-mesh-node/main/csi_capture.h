/**
 * @file csi_capture.h
 * @brief CSI data capture and ADR-018 binary frame serialization.
 *
 * Ported from the original esp32-csi-node firmware. The key difference
 * is that serialized frames are sent via mesh_transport instead of
 * direct UDP.
 */

#ifndef CSI_CAPTURE_H
#define CSI_CAPTURE_H

#include <stdint.h>
#include <stddef.h>

/** ADR-018 magic number. */
#define CSI_MAGIC 0xC5110001

/** ADR-018 header size in bytes. */
#define CSI_HEADER_SIZE 20

/** Maximum frame buffer size (header + 4 antennas * 256 subcarriers * 2 bytes). */
#define CSI_MAX_FRAME_SIZE (CSI_HEADER_SIZE + 4 * 256 * 2)

/**
 * Initialize CSI capture.
 *
 * Enables promiscuous mode and registers the CSI callback. Captured frames
 * are serialized in ADR-018 format and sent via mesh_transport_send().
 *
 * @param node_id  Node ID to embed in the ADR-018 frame header.
 */
void csi_capture_init(uint8_t node_id);

/**
 * Get the total number of CSI callbacks received.
 */
uint32_t csi_capture_get_cb_count(void);

/**
 * Get the number of frames successfully serialized and dispatched.
 */
uint32_t csi_capture_get_frame_count(void);

#endif /* CSI_CAPTURE_H */
