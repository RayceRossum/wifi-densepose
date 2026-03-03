//! ESP-MESH topology types and health tracking.
//!
//! Provides data structures for representing the self-organizing mesh
//! network topology used by ESP-MESH backhaul nodes. These types are
//! used by the aggregator to track mesh health, estimate multi-hop
//! latency, and detect node connectivity issues.
//!
//! The mesh topology is a tree rooted at the gateway node:
//! ```text
//!                    Router
//!                      │
//!                  [Gateway]   ← root (layer 0)
//!                   /     \
//!              [Node 1] [Node 2]  ← layer 1
//!               /          \
//!          [Node 3]     [Node 4]  ← layer 2
//! ```
//!
//! # Usage
//!
//! ```rust
//! use wifi_densepose_hardware::esp32::mesh::{MeshTopology, MeshNodeRole};
//!
//! let mut topo = MeshTopology::new(0); // gateway is node 0
//! topo.register_node(1, MeshNodeRole::Relay, 1, Some(0));
//! topo.register_node(3, MeshNodeRole::Leaf, 2, Some(1));
//!
//! assert_eq!(topo.hop_count(3), Some(2));
//! assert_eq!(topo.path_to_root(3), vec![3, 1, 0]);
//! ```

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Role of a node in the ESP-MESH tree.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MeshNodeRole {
    /// Root node (gateway) — bridges mesh to upstream router + UDP.
    Root,
    /// Intermediate node — captures CSI and relays child traffic.
    Relay,
    /// Leaf node — captures CSI only, no children.
    Leaf,
}

impl std::fmt::Display for MeshNodeRole {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MeshNodeRole::Root => write!(f, "Root"),
            MeshNodeRole::Relay => write!(f, "Relay"),
            MeshNodeRole::Leaf => write!(f, "Leaf"),
        }
    }
}

/// Information about a single node in the mesh.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MeshNodeInfo {
    /// Node ID (matches ADR-018 frame header node_id field).
    pub node_id: u8,
    /// Role in the mesh tree.
    pub role: MeshNodeRole,
    /// Mesh layer depth (0 = root, 1 = direct child of root, etc.).
    pub layer: u8,
    /// Parent node ID (None for root).
    pub parent_id: Option<u8>,
    /// Number of direct children.
    pub child_count: u8,
    /// Number of hops to reach root (same as layer for tree topology).
    pub hop_count: u8,
}

/// Mesh topology — represents the full ESP-MESH tree structure.
///
/// Tracks which nodes are in the mesh, their parent-child relationships,
/// and provides path/hop queries. Updated from gateway status messages
/// or inferred from frame arrival patterns.
#[derive(Debug, Clone)]
pub struct MeshTopology {
    /// Root (gateway) node ID.
    root_id: u8,
    /// Per-node info, keyed by node_id.
    nodes: HashMap<u8, MeshNodeInfo>,
}

impl MeshTopology {
    /// Create a new topology with the given root node ID.
    pub fn new(root_id: u8) -> Self {
        let mut nodes = HashMap::new();
        nodes.insert(root_id, MeshNodeInfo {
            node_id: root_id,
            role: MeshNodeRole::Root,
            layer: 0,
            parent_id: None,
            child_count: 0,
            hop_count: 0,
        });
        Self { root_id, nodes }
    }

    /// Register a node in the topology.
    ///
    /// If `parent_id` is provided and exists, the parent's child_count
    /// is incremented. Hop count is derived from the parent's layer + 1.
    pub fn register_node(
        &mut self,
        node_id: u8,
        role: MeshNodeRole,
        layer: u8,
        parent_id: Option<u8>,
    ) {
        // Increment parent's child count
        if let Some(pid) = parent_id {
            if let Some(parent) = self.nodes.get_mut(&pid) {
                parent.child_count += 1;
                // Upgrade parent from Leaf to Relay if it gains children
                if parent.role == MeshNodeRole::Leaf {
                    parent.role = MeshNodeRole::Relay;
                }
            }
        }

        self.nodes.insert(node_id, MeshNodeInfo {
            node_id,
            role,
            layer,
            parent_id,
            child_count: 0,
            hop_count: layer,
        });
    }

    /// Remove a node from the topology.
    ///
    /// Decrements the parent's child_count. Does NOT cascade-remove children.
    pub fn remove_node(&mut self, node_id: u8) {
        if node_id == self.root_id {
            return; // Cannot remove root
        }
        if let Some(info) = self.nodes.remove(&node_id) {
            if let Some(pid) = info.parent_id {
                if let Some(parent) = self.nodes.get_mut(&pid) {
                    parent.child_count = parent.child_count.saturating_sub(1);
                }
            }
        }
    }

    /// Get info for a specific node.
    pub fn node_info(&self, node_id: u8) -> Option<&MeshNodeInfo> {
        self.nodes.get(&node_id)
    }

    /// Get the root node ID.
    pub fn root_id(&self) -> u8 {
        self.root_id
    }

    /// Get the number of nodes in the mesh (including root).
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Get the number of sensor nodes (excluding root gateway).
    pub fn sensor_count(&self) -> usize {
        self.nodes.values().filter(|n| n.role != MeshNodeRole::Root).count()
    }

    /// Get the hop count from a node to the root.
    /// Returns None if node not found.
    pub fn hop_count(&self, node_id: u8) -> Option<u8> {
        self.nodes.get(&node_id).map(|n| n.hop_count)
    }

    /// Get the path from a node to root (list of node IDs).
    /// Returns empty vec if node not found.
    pub fn path_to_root(&self, node_id: u8) -> Vec<u8> {
        let mut path = Vec::new();
        let mut current = node_id;

        // Walk up the tree following parent pointers (max 25 layers)
        for _ in 0..25 {
            if let Some(info) = self.nodes.get(&current) {
                path.push(current);
                if current == self.root_id {
                    break;
                }
                match info.parent_id {
                    Some(pid) => current = pid,
                    None => break,
                }
            } else {
                break;
            }
        }

        path
    }

    /// Get the maximum layer depth in the mesh.
    pub fn max_layer(&self) -> u8 {
        self.nodes.values().map(|n| n.layer).max().unwrap_or(0)
    }

    /// Get all node IDs at a specific layer.
    pub fn nodes_at_layer(&self, layer: u8) -> Vec<u8> {
        self.nodes.values()
            .filter(|n| n.layer == layer)
            .map(|n| n.node_id)
            .collect()
    }

    /// Get all node infos.
    pub fn all_nodes(&self) -> impl Iterator<Item = &MeshNodeInfo> {
        self.nodes.values()
    }

    /// Estimate one-way mesh relay latency for a node based on hop count.
    ///
    /// ESP-MESH typically adds 2-5ms per hop for small payloads (<512 bytes).
    /// CSI frames are 150-300 bytes, so ~3ms/hop is a reasonable estimate.
    pub fn estimated_relay_latency(&self, node_id: u8) -> Option<Duration> {
        self.hop_count(node_id).map(|hops| {
            Duration::from_millis(hops as u64 * 3)
        })
    }
}

/// Per-node health metrics tracked from frame arrival patterns.
#[derive(Debug, Clone)]
pub struct MeshNodeHealth {
    /// Node ID.
    pub node_id: u8,
    /// Total frames received from this node.
    pub frames_received: u64,
    /// Frames dropped (sequence gaps).
    pub frames_dropped: u64,
    /// Last time a frame was received from this node.
    pub last_seen: Instant,
    /// Estimated frame rate (frames per second), smoothed.
    pub frame_rate_hz: f64,
    /// Whether this node is considered alive.
    pub alive: bool,

    // Internal tracking
    rate_window_start: Instant,
    rate_window_count: u64,
}

impl MeshNodeHealth {
    /// Create a new health tracker for a node.
    pub fn new(node_id: u8) -> Self {
        let now = Instant::now();
        Self {
            node_id,
            frames_received: 0,
            frames_dropped: 0,
            last_seen: now,
            frame_rate_hz: 0.0,
            alive: true,
            rate_window_start: now,
            rate_window_count: 0,
        }
    }

    /// Record a frame arrival. Updates rate estimate and liveness.
    pub fn record_frame(&mut self, dropped: u64) {
        let now = Instant::now();
        self.frames_received += 1;
        self.frames_dropped += dropped;
        self.last_seen = now;
        self.alive = true;

        // Update frame rate using a 1-second sliding window
        self.rate_window_count += 1;
        let elapsed = now.duration_since(self.rate_window_start);
        if elapsed >= Duration::from_secs(1) {
            self.frame_rate_hz = self.rate_window_count as f64
                / elapsed.as_secs_f64();
            self.rate_window_start = now;
            self.rate_window_count = 0;
        }
    }

    /// Check liveness. Mark dead if no frame in `timeout`.
    pub fn check_liveness(&mut self, timeout: Duration) {
        if self.last_seen.elapsed() > timeout {
            self.alive = false;
            self.frame_rate_hz = 0.0;
        }
    }

    /// Frame delivery ratio (0.0 - 1.0).
    pub fn delivery_ratio(&self) -> f64 {
        let total = self.frames_received + self.frames_dropped;
        if total == 0 {
            return 1.0;
        }
        self.frames_received as f64 / total as f64
    }

    /// Time since last frame was seen.
    pub fn time_since_last_frame(&self) -> Duration {
        self.last_seen.elapsed()
    }
}

/// Aggregate health metrics for the entire mesh network.
#[derive(Debug, Clone)]
pub struct MeshHealthSummary {
    /// Total nodes tracked.
    pub total_nodes: usize,
    /// Nodes currently alive (receiving frames).
    pub alive_nodes: usize,
    /// Nodes that have gone silent.
    pub dead_nodes: usize,
    /// Aggregate frame rate across all nodes (frames/sec).
    pub total_frame_rate_hz: f64,
    /// Worst-case delivery ratio across all nodes.
    pub min_delivery_ratio: f64,
    /// Maximum mesh layer depth observed.
    pub max_layer: u8,
}

/// Mesh-aware health tracker for all nodes.
///
/// Sits alongside the `Esp32Aggregator` to provide mesh-level health
/// monitoring. Call `record_frame()` for each received frame and
/// `check_health()` periodically.
#[derive(Debug)]
pub struct MeshHealthTracker {
    nodes: HashMap<u8, MeshNodeHealth>,
    /// How long without a frame before a node is considered dead.
    liveness_timeout: Duration,
}

impl MeshHealthTracker {
    /// Create a new tracker with the given liveness timeout.
    pub fn new(liveness_timeout: Duration) -> Self {
        Self {
            nodes: HashMap::new(),
            liveness_timeout,
        }
    }

    /// Create with default 5-second timeout.
    pub fn default_timeout() -> Self {
        Self::new(Duration::from_secs(5))
    }

    /// Record a frame from a node.
    pub fn record_frame(&mut self, node_id: u8, sequence_gap: u64) {
        self.nodes
            .entry(node_id)
            .or_insert_with(|| MeshNodeHealth::new(node_id))
            .record_frame(sequence_gap);
    }

    /// Check liveness for all nodes. Call periodically (e.g., every second).
    pub fn check_health(&mut self) {
        for health in self.nodes.values_mut() {
            health.check_liveness(self.liveness_timeout);
        }
    }

    /// Get health info for a specific node.
    pub fn node_health(&self, node_id: u8) -> Option<&MeshNodeHealth> {
        self.nodes.get(&node_id)
    }

    /// Get aggregate mesh health summary.
    pub fn summary(&self) -> MeshHealthSummary {
        let total_nodes = self.nodes.len();
        let alive_nodes = self.nodes.values().filter(|n| n.alive).count();
        let total_frame_rate_hz: f64 = self.nodes.values()
            .map(|n| n.frame_rate_hz).sum();
        let min_delivery_ratio = self.nodes.values()
            .map(|n| n.delivery_ratio())
            .fold(1.0_f64, f64::min);

        MeshHealthSummary {
            total_nodes,
            alive_nodes,
            dead_nodes: total_nodes - alive_nodes,
            total_frame_rate_hz,
            min_delivery_ratio,
            max_layer: 0, // Updated when paired with MeshTopology
        }
    }

    /// Get all tracked node IDs.
    pub fn tracked_nodes(&self) -> Vec<u8> {
        self.nodes.keys().copied().collect()
    }

    /// Number of tracked nodes.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_topology_new() {
        let topo = MeshTopology::new(0);
        assert_eq!(topo.root_id(), 0);
        assert_eq!(topo.node_count(), 1);
        assert_eq!(topo.sensor_count(), 0);
        assert_eq!(topo.hop_count(0), Some(0));
    }

    #[test]
    fn test_topology_register_nodes() {
        let mut topo = MeshTopology::new(0);
        topo.register_node(1, MeshNodeRole::Relay, 1, Some(0));
        topo.register_node(2, MeshNodeRole::Relay, 1, Some(0));
        topo.register_node(3, MeshNodeRole::Leaf, 2, Some(1));
        topo.register_node(4, MeshNodeRole::Leaf, 2, Some(2));

        assert_eq!(topo.node_count(), 5);
        assert_eq!(topo.sensor_count(), 4);
        assert_eq!(topo.max_layer(), 2);

        // Root should have 2 children
        let root = topo.node_info(0).unwrap();
        assert_eq!(root.child_count, 2);

        // Node 1 should have 1 child
        let node1 = topo.node_info(1).unwrap();
        assert_eq!(node1.child_count, 1);
        assert_eq!(node1.role, MeshNodeRole::Relay);
    }

    #[test]
    fn test_topology_hop_count() {
        let mut topo = MeshTopology::new(0);
        topo.register_node(1, MeshNodeRole::Relay, 1, Some(0));
        topo.register_node(3, MeshNodeRole::Leaf, 2, Some(1));

        assert_eq!(topo.hop_count(0), Some(0));
        assert_eq!(topo.hop_count(1), Some(1));
        assert_eq!(topo.hop_count(3), Some(2));
        assert_eq!(topo.hop_count(99), None); // Not in topology
    }

    #[test]
    fn test_topology_path_to_root() {
        let mut topo = MeshTopology::new(0);
        topo.register_node(1, MeshNodeRole::Relay, 1, Some(0));
        topo.register_node(3, MeshNodeRole::Leaf, 2, Some(1));

        assert_eq!(topo.path_to_root(3), vec![3, 1, 0]);
        assert_eq!(topo.path_to_root(1), vec![1, 0]);
        assert_eq!(topo.path_to_root(0), vec![0]);
        assert_eq!(topo.path_to_root(99), vec![]); // Not found
    }

    #[test]
    fn test_topology_remove_node() {
        let mut topo = MeshTopology::new(0);
        topo.register_node(1, MeshNodeRole::Relay, 1, Some(0));
        topo.register_node(2, MeshNodeRole::Leaf, 1, Some(0));

        assert_eq!(topo.node_count(), 3);
        assert_eq!(topo.node_info(0).unwrap().child_count, 2);

        topo.remove_node(2);
        assert_eq!(topo.node_count(), 2);
        assert_eq!(topo.node_info(0).unwrap().child_count, 1);
    }

    #[test]
    fn test_topology_cannot_remove_root() {
        let mut topo = MeshTopology::new(0);
        topo.remove_node(0);
        assert_eq!(topo.node_count(), 1); // Root still present
    }

    #[test]
    fn test_topology_nodes_at_layer() {
        let mut topo = MeshTopology::new(0);
        topo.register_node(1, MeshNodeRole::Relay, 1, Some(0));
        topo.register_node(2, MeshNodeRole::Relay, 1, Some(0));
        topo.register_node(3, MeshNodeRole::Leaf, 2, Some(1));

        let layer1 = topo.nodes_at_layer(1);
        assert_eq!(layer1.len(), 2);
        assert!(layer1.contains(&1));
        assert!(layer1.contains(&2));
    }

    #[test]
    fn test_topology_estimated_latency() {
        let mut topo = MeshTopology::new(0);
        topo.register_node(1, MeshNodeRole::Relay, 1, Some(0));
        topo.register_node(3, MeshNodeRole::Leaf, 2, Some(1));

        // Root: 0 hops → 0ms
        assert_eq!(topo.estimated_relay_latency(0), Some(Duration::from_millis(0)));
        // Layer 1: 1 hop → 3ms
        assert_eq!(topo.estimated_relay_latency(1), Some(Duration::from_millis(3)));
        // Layer 2: 2 hops → 6ms
        assert_eq!(topo.estimated_relay_latency(3), Some(Duration::from_millis(6)));
    }

    #[test]
    fn test_topology_leaf_promoted_to_relay() {
        let mut topo = MeshTopology::new(0);
        topo.register_node(1, MeshNodeRole::Leaf, 1, Some(0));
        assert_eq!(topo.node_info(1).unwrap().role, MeshNodeRole::Leaf);

        // Register a child of node 1 — should promote to Relay
        topo.register_node(3, MeshNodeRole::Leaf, 2, Some(1));
        assert_eq!(topo.node_info(1).unwrap().role, MeshNodeRole::Relay);
    }

    #[test]
    fn test_node_health_tracking() {
        let mut health = MeshNodeHealth::new(1);
        assert_eq!(health.frames_received, 0);
        assert!(health.alive);

        health.record_frame(0); // No gap
        assert_eq!(health.frames_received, 1);
        assert_eq!(health.frames_dropped, 0);

        health.record_frame(4); // Gap of 4
        assert_eq!(health.frames_received, 2);
        assert_eq!(health.frames_dropped, 4);
    }

    #[test]
    fn test_node_health_delivery_ratio() {
        let mut health = MeshNodeHealth::new(1);
        health.record_frame(0);
        health.record_frame(0);
        health.record_frame(0);
        // 3 received, 0 dropped → 1.0
        assert!((health.delivery_ratio() - 1.0).abs() < 0.001);

        health.record_frame(2); // 2 dropped
        // 4 received, 2 dropped → 4/6 = 0.667
        assert!((health.delivery_ratio() - 4.0 / 6.0).abs() < 0.01);
    }

    #[test]
    fn test_health_tracker() {
        let mut tracker = MeshHealthTracker::default_timeout();

        tracker.record_frame(1, 0);
        tracker.record_frame(2, 0);
        tracker.record_frame(1, 0);

        assert_eq!(tracker.node_count(), 2);

        let h1 = tracker.node_health(1).unwrap();
        assert_eq!(h1.frames_received, 2);
        assert!(h1.alive);

        let h2 = tracker.node_health(2).unwrap();
        assert_eq!(h2.frames_received, 1);
    }

    #[test]
    fn test_health_summary() {
        let mut tracker = MeshHealthTracker::default_timeout();
        tracker.record_frame(1, 0);
        tracker.record_frame(2, 0);

        let summary = tracker.summary();
        assert_eq!(summary.total_nodes, 2);
        assert_eq!(summary.alive_nodes, 2);
        assert_eq!(summary.dead_nodes, 0);
    }

    #[test]
    fn test_mesh_node_role_display() {
        assert_eq!(format!("{}", MeshNodeRole::Root), "Root");
        assert_eq!(format!("{}", MeshNodeRole::Relay), "Relay");
        assert_eq!(format!("{}", MeshNodeRole::Leaf), "Leaf");
    }
}
