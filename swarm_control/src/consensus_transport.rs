//! Zenoh transport loop for Raft consensus messages.
//!
//! Bridges [`MissionConsensus`] with the Zenoh pub/sub network so that
//! Raft messages are exchanged between drones over `/swarm/drone_N/consensus/raft_tx`.
//!
//! # Topic layout
//!
//! Each drone publishes its outgoing Raft messages to:
//!   `/swarm/drone_{self.id}/consensus/raft_tx`
//!
//! and subscribes to all peers:
//!   `/swarm/drone_*/consensus/raft_tx`
//!
//! Messages are Protobuf-encoded (`raft::eraftpb::Message`).
//!
//! A dedup cache (bounded ring buffer of recent message hashes) prevents
//! processing the same Raft message twice under Zenoh's at-least-once delivery.

use std::collections::VecDeque;
use std::hash::{Hash, Hasher};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use raft::eraftpb::Message as RaftMessage;
use raft::Storage;

use crate::consensus::{
    deserialize_raft_message, serialize_raft_message, ConsensusError, MissionConsensus,
};

// ── Dedup cache ──────────────────────────────────────────────────────────────

/// Fingerprint a Raft message for dedup purposes.
/// Uses (from, to, term, index, log_term, msg_type) as the key.
fn message_fingerprint(msg: &RaftMessage) -> u64 {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    msg.get_from().hash(&mut hasher);
    msg.get_to().hash(&mut hasher);
    msg.get_term().hash(&mut hasher);
    msg.get_index().hash(&mut hasher);
    msg.get_log_term().hash(&mut hasher);
    (msg.get_msg_type() as i32).hash(&mut hasher);
    msg.get_commit().hash(&mut hasher);
    hasher.finish()
}

/// Bounded ring-buffer dedup cache.
pub struct DedupCache {
    seen: VecDeque<u64>,
    capacity: usize,
}

impl DedupCache {
    pub fn new(capacity: usize) -> Self {
        Self {
            seen: VecDeque::with_capacity(capacity),
            capacity,
        }
    }

    /// Returns `true` if the fingerprint was already seen (duplicate).
    /// Otherwise inserts it and returns `false`.
    pub fn check_and_insert(&mut self, fingerprint: u64) -> bool {
        if self.seen.contains(&fingerprint) {
            return true;
        }
        if self.seen.len() >= self.capacity {
            self.seen.pop_front();
        }
        self.seen.push_back(fingerprint);
        false
    }

    pub fn clear(&mut self) {
        self.seen.clear();
    }

    pub fn len(&self) -> usize {
        self.seen.len()
    }

    pub fn is_empty(&self) -> bool {
        self.seen.is_empty()
    }
}

// ── Transport error ──────────────────────────────────────────────────────────

#[derive(Debug)]
pub enum TransportLoopError {
    Consensus(ConsensusError),
    Serialization(String),
    ZenohPublish(String),
}

impl std::fmt::Display for TransportLoopError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Consensus(e) => write!(f, "consensus: {e}"),
            Self::Serialization(e) => write!(f, "serialization: {e}"),
            Self::ZenohPublish(e) => write!(f, "zenoh publish: {e}"),
        }
    }
}

impl From<ConsensusError> for TransportLoopError {
    fn from(e: ConsensusError) -> Self {
        Self::Consensus(e)
    }
}

// ── Topic helpers ────────────────────────────────────────────────────────────

/// Publish topic for a given drone's outgoing Raft messages.
pub fn raft_tx_topic(drone_id: u64) -> String {
    format!("swarm/drone_{drone_id}/consensus/raft_tx")
}

/// Wildcard subscription pattern for all drones' Raft messages.
pub fn raft_tx_subscribe_pattern() -> String {
    "swarm/drone_$*/consensus/raft_tx".to_string()
}

/// Extract the source drone ID from a key expression like
/// `swarm/drone_3/consensus/raft_tx`.  Returns `None` if unparseable.
pub fn drone_id_from_key(key: &str) -> Option<u64> {
    let parts: Vec<&str> = key.split('/').collect();
    // Expected: ["swarm", "drone_N", "consensus", "raft_tx"]
    if parts.len() >= 2 {
        parts[1]
            .strip_prefix("drone_")
            .and_then(|s| s.parse::<u64>().ok())
    } else {
        None
    }
}

// ── ConsensusTransportLoop ───────────────────────────────────────────────────

/// Configuration for the consensus transport loop.
#[derive(Debug, Clone)]
pub struct TransportLoopConfig {
    /// Raft tick interval (how often `tick()` is called).
    pub tick_interval: Duration,
    /// Maximum number of message fingerprints to cache for dedup.
    pub dedup_cache_size: usize,
}

impl Default for TransportLoopConfig {
    fn default() -> Self {
        Self {
            tick_interval: Duration::from_millis(100),
            dedup_cache_size: 4096,
        }
    }
}

/// The consensus transport loop bridges a `MissionConsensus` node with the
/// Zenoh network.
///
/// # Architecture
///
/// ```text
///  ┌─────────────────┐     protobuf     ┌────────────────┐
///  │ MissionConsensus │ ──drain_outgoing──▶ Zenoh publish  │
///  │   (raft-rs)      │                   │ /swarm/drone_N │
///  │                  │ ◀──step()──────── │ /consensus/    │
///  └─────────────────┘     deserialize   │  raft_tx       │
///                                         └────────────────┘
/// ```
///
/// The loop runs on a tokio task and:
/// 1. Ticks the Raft node at `tick_interval`
/// 2. Drains outgoing messages, serializes with Protobuf, publishes to
///    per-drone topics (targeted, not broadcast)
/// 3. Receives incoming messages from Zenoh subscriber, dedup-checks,
///    deserializes, and steps into the Raft node
pub struct ConsensusTransportLoop<S: Storage> {
    /// Shared consensus node (behind Arc<Mutex<>> for subscriber callback).
    consensus: Arc<Mutex<MissionConsensus<S>>>,
    /// This drone's node ID.
    node_id: u64,
    /// Dedup cache for incoming messages.
    dedup: DedupCache,
    /// Configuration.
    config: TransportLoopConfig,
    /// Outgoing message buffer (serialized, ready to publish).
    /// Each entry is (target_drone_id, serialized_bytes).
    outbox: Vec<(u64, Vec<u8>)>,
}

impl<S: Storage> ConsensusTransportLoop<S> {
    pub fn new(
        consensus: Arc<Mutex<MissionConsensus<S>>>,
        node_id: u64,
        config: TransportLoopConfig,
    ) -> Self {
        let dedup = DedupCache::new(config.dedup_cache_size);
        Self {
            consensus,
            node_id,
            dedup,
            config,
            outbox: Vec::new(),
        }
    }

    /// Get the tick interval for external scheduling.
    pub fn tick_interval(&self) -> Duration {
        self.config.tick_interval
    }

    /// Run one consensus tick: advance the Raft clock, drain outgoing messages,
    /// serialize them, and buffer for publishing.
    ///
    /// Returns the number of messages queued for publishing.
    pub fn run_tick(&mut self) -> Result<usize, TransportLoopError> {
        let mut consensus = self.consensus.lock().unwrap();
        consensus.tick();

        let messages = consensus.drain_outgoing();
        self.outbox.clear();

        for msg in &messages {
            let target_id = msg.get_to();
            let bytes = serialize_raft_message(msg).map_err(|e| {
                TransportLoopError::Serialization(format!("protobuf encode: {e}"))
            })?;
            self.outbox.push((target_id, bytes));
        }

        Ok(messages.len())
    }

    /// Handle an incoming Raft message from the network.
    ///
    /// Checks the dedup cache, deserializes from Protobuf, and steps into
    /// the consensus node.  Returns `Ok(true)` if the message was processed,
    /// `Ok(false)` if it was a duplicate.
    pub fn handle_incoming(&mut self, raw: &[u8]) -> Result<bool, TransportLoopError> {
        let msg = deserialize_raft_message(raw).map_err(|e| {
            TransportLoopError::Serialization(format!("protobuf decode: {e}"))
        })?;

        // Skip our own messages
        if msg.get_from() == self.node_id {
            return Ok(false);
        }

        // Skip if not addressed to us
        if msg.get_to() != self.node_id {
            return Ok(false);
        }

        // Dedup check
        let fp = message_fingerprint(&msg);
        if self.dedup.check_and_insert(fp) {
            return Ok(false);
        }

        let mut consensus = self.consensus.lock().unwrap();
        consensus.step(msg)?;
        Ok(true)
    }

    /// Drain the outbox of serialized messages ready to publish.
    /// Each item is `(target_drone_id, serialized_bytes)`.
    pub fn drain_outbox(&mut self) -> Vec<(u64, Vec<u8>)> {
        std::mem::take(&mut self.outbox)
    }

    /// Access the shared consensus node.
    pub fn consensus(&self) -> &Arc<Mutex<MissionConsensus<S>>> {
        &self.consensus
    }

    /// Access the dedup cache (for diagnostics).
    pub fn dedup_cache_len(&self) -> usize {
        self.dedup.len()
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::consensus::{
        ConsensusConfig, MissionCommand, MissionConsensus, MissionState,
        elect_leader, route_messages, serialize_raft_message,
    };
    use raft::eraftpb::Message as RaftMessage;
    use raft::storage::MemStorage;

    fn make_transport_cluster(
        peers: &[u64],
    ) -> Vec<ConsensusTransportLoop<MemStorage>> {
        peers
            .iter()
            .map(|&id| {
                let storage = MemStorage::new_with_conf_state((peers.to_vec(), vec![]));
                let consensus =
                    MissionConsensus::new(id, peers, storage).unwrap();
                let shared = Arc::new(Mutex::new(consensus));
                ConsensusTransportLoop::new(
                    shared,
                    id,
                    TransportLoopConfig::default(),
                )
            })
            .collect()
    }

    // ── DedupCache tests ─────────────────────────────────────────────────

    #[test]
    fn dedup_cache_rejects_duplicates() {
        let mut cache = DedupCache::new(16);
        assert!(!cache.check_and_insert(42));
        assert!(cache.check_and_insert(42)); // duplicate
        assert!(!cache.check_and_insert(43)); // new
    }

    #[test]
    fn dedup_cache_evicts_oldest() {
        let mut cache = DedupCache::new(3);
        cache.check_and_insert(1);
        cache.check_and_insert(2);
        cache.check_and_insert(3);
        // Cache full: [1, 2, 3]
        cache.check_and_insert(4);
        // Should have evicted 1: [2, 3, 4]
        assert!(!cache.check_and_insert(1)); // 1 is gone, treated as new
        assert!(cache.check_and_insert(4)); // 4 still there
    }

    #[test]
    fn dedup_cache_clear() {
        let mut cache = DedupCache::new(16);
        cache.check_and_insert(1);
        cache.check_and_insert(2);
        assert_eq!(cache.len(), 2);
        cache.clear();
        assert!(cache.is_empty());
        assert!(!cache.check_and_insert(1)); // 1 is new again
    }

    // ── Topic helpers ────────────────────────────────────────────────────

    #[test]
    fn raft_tx_topic_format() {
        assert_eq!(raft_tx_topic(3), "swarm/drone_3/consensus/raft_tx");
    }

    #[test]
    fn drone_id_from_key_parses() {
        assert_eq!(
            drone_id_from_key("swarm/drone_5/consensus/raft_tx"),
            Some(5)
        );
        assert_eq!(drone_id_from_key("swarm/drone_12/consensus/raft_tx"), Some(12));
        assert_eq!(drone_id_from_key("garbage"), None);
        assert_eq!(drone_id_from_key("swarm/not_drone/consensus"), None);
    }

    // ── Transport loop tests ─────────────────────────────────────────────

    #[test]
    fn run_tick_produces_messages_after_election_timeout() {
        let peers = [1, 2, 3, 4, 5, 6];
        let mut loops = make_transport_cluster(&peers);

        // Tick enough times to trigger election timeout (election_tick=15)
        let mut total_msgs = 0;
        for _ in 0..20 {
            for tl in loops.iter_mut() {
                total_msgs += tl.run_tick().unwrap();
            }
        }
        // At least one node should have generated vote request messages
        assert!(total_msgs > 0, "should produce Raft messages after ticking");
    }

    #[test]
    fn handle_incoming_rejects_own_messages() {
        let peers = [1, 2, 3, 4, 5, 6];
        let mut loops = make_transport_cluster(&peers);

        let mut msg = RaftMessage::new();
        msg.set_from(1);
        msg.set_to(1);
        msg.set_term(1);
        let bytes = serialize_raft_message(&msg).unwrap();

        // Node 1 should reject its own message
        let processed = loops[0].handle_incoming(&bytes).unwrap();
        assert!(!processed);
    }

    #[test]
    fn handle_incoming_rejects_wrong_target() {
        let peers = [1, 2, 3, 4, 5, 6];
        let mut loops = make_transport_cluster(&peers);

        let mut msg = RaftMessage::new();
        msg.set_from(2);
        msg.set_to(3); // addressed to node 3
        msg.set_term(1);
        let bytes = serialize_raft_message(&msg).unwrap();

        // Node 1 should reject message not addressed to it
        let processed = loops[0].handle_incoming(&bytes).unwrap();
        assert!(!processed);
    }

    #[test]
    fn handle_incoming_dedup_rejects_repeat() {
        let peers = [1, 2, 3, 4, 5, 6];
        let mut loops = make_transport_cluster(&peers);

        let mut msg = RaftMessage::new();
        msg.set_from(2);
        msg.set_to(1);
        msg.set_term(1);
        let bytes = serialize_raft_message(&msg).unwrap();

        // First delivery
        let _ = loops[0].handle_incoming(&bytes);
        // Second delivery (duplicate) — should be rejected by dedup
        let processed = loops[0].handle_incoming(&bytes).unwrap();
        assert!(!processed);
    }

    #[test]
    fn outbox_contains_targeted_messages() {
        let peers = [1, 2, 3, 4, 5, 6];
        let mut loops = make_transport_cluster(&peers);

        // Tick enough to generate election messages
        for _ in 0..20 {
            for tl in loops.iter_mut() {
                tl.run_tick().unwrap();
            }
        }

        // Check that at least one loop has outbox entries with valid target IDs
        let has_outbox = loops.iter_mut().any(|tl| {
            let outbox = tl.drain_outbox();
            outbox.iter().all(|(target_id, bytes)| {
                peers.contains(target_id) && !bytes.is_empty()
            }) && !outbox.is_empty()
        });
        assert!(has_outbox, "should have targeted outbox messages");
    }

    #[test]
    fn full_transport_round_trip() {
        // Simulate a full transport round: tick → drain outbox → deliver to targets
        let peers = [1, 2, 3, 4, 5, 6];
        let mut loops = make_transport_cluster(&peers);

        // Run enough rounds to elect a leader
        for _ in 0..50 {
            // Tick all nodes
            for tl in loops.iter_mut() {
                tl.run_tick().unwrap();
            }

            // Collect all outbox messages
            let mut all_messages: Vec<(u64, Vec<u8>)> = Vec::new();
            for tl in loops.iter_mut() {
                all_messages.extend(tl.drain_outbox());
            }

            // Deliver messages to targets
            for (target_id, bytes) in &all_messages {
                if let Some(tl) = loops.iter_mut().find(|t| t.node_id == *target_id) {
                    let _ = tl.handle_incoming(bytes);
                }
            }
        }

        // Check that a leader was elected
        let leader_count = loops
            .iter()
            .filter(|tl| {
                let c = tl.consensus.lock().unwrap();
                c.is_leader()
            })
            .count();
        assert_eq!(leader_count, 1, "exactly one leader should be elected");

        // All nodes should agree on the leader
        let leader_ids: Vec<Option<u64>> = loops
            .iter()
            .map(|tl| {
                let c = tl.consensus.lock().unwrap();
                c.leader_id()
            })
            .collect();
        let first = leader_ids[0];
        assert!(first.is_some(), "leader should be known");
        for lid in &leader_ids {
            assert_eq!(*lid, first, "all nodes should agree on leader");
        }
    }

    #[test]
    fn transport_propose_and_commit() {
        let peers = [1, 2, 3, 4, 5, 6];
        let mut loops = make_transport_cluster(&peers);

        // Elect a leader via transport round-trips
        for _ in 0..50 {
            for tl in loops.iter_mut() {
                tl.run_tick().unwrap();
            }
            let mut msgs: Vec<(u64, Vec<u8>)> = Vec::new();
            for tl in loops.iter_mut() {
                msgs.extend(tl.drain_outbox());
            }
            for (tid, bytes) in &msgs {
                if let Some(tl) = loops.iter_mut().find(|t| t.node_id == *tid) {
                    let _ = tl.handle_incoming(bytes);
                }
            }
        }

        // Find leader and propose a state change
        let leader_id = loops
            .iter()
            .find(|tl| tl.consensus.lock().unwrap().is_leader())
            .map(|tl| tl.node_id)
            .expect("leader should exist");

        {
            let leader_tl = loops.iter().find(|tl| tl.node_id == leader_id).unwrap();
            let mut c = leader_tl.consensus.lock().unwrap();
            c.propose_state(MissionCommand {
                epoch: 1,
                state: MissionState::Lift,
            })
            .unwrap();
        }

        // Run more rounds to replicate
        for _ in 0..30 {
            for tl in loops.iter_mut() {
                tl.run_tick().unwrap();
            }
            let mut msgs: Vec<(u64, Vec<u8>)> = Vec::new();
            for tl in loops.iter_mut() {
                msgs.extend(tl.drain_outbox());
            }
            for (tid, bytes) in &msgs {
                if let Some(tl) = loops.iter_mut().find(|t| t.node_id == *tid) {
                    let _ = tl.handle_incoming(bytes);
                }
            }
        }

        // All nodes should have committed Lift
        for tl in &loops {
            let c = tl.consensus.lock().unwrap();
            assert_eq!(
                c.current_state(),
                MissionState::Lift,
                "node {} should be Lift",
                tl.node_id
            );
        }
    }

    #[test]
    fn message_fingerprint_differs_for_different_messages() {
        let mut m1 = RaftMessage::new();
        m1.set_from(1);
        m1.set_to(2);
        m1.set_term(1);

        let mut m2 = RaftMessage::new();
        m2.set_from(1);
        m2.set_to(3);
        m2.set_term(1);

        assert_ne!(message_fingerprint(&m1), message_fingerprint(&m2));
    }

    #[test]
    fn message_fingerprint_same_for_identical() {
        let mut m1 = RaftMessage::new();
        m1.set_from(1);
        m1.set_to(2);
        m1.set_term(5);
        m1.set_index(10);

        let mut m2 = RaftMessage::new();
        m2.set_from(1);
        m2.set_to(2);
        m2.set_term(5);
        m2.set_index(10);

        assert_eq!(message_fingerprint(&m1), message_fingerprint(&m2));
    }
}
