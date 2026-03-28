//! Raft-based mission state consensus for 6-drone swarm.
//!
//! All drones must agree on the current `MissionState` (Transit / Lift / Hold)
//! even under network instability.  Uses `raft-rs` with configurable election
//! and heartbeat ticks, pre-vote, and check_quorum for partition resilience.

use std::collections::VecDeque;

use protobuf::Message as ProtobufMessage;
use raft::eraftpb::{ConfChange, ConfChangeType, Entry, EntryType, Message};
use raft::{Config, RawNode, StateRole, Storage};
use serde::{Deserialize, Serialize};

// ── Mission types ───────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MissionState {
    Transit,
    Lift,
    Hold,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MissionCommand {
    pub epoch: u64,
    pub state: MissionState,
}

impl MissionCommand {
    pub fn encode(&self) -> Result<Vec<u8>, bincode::Error> {
        bincode::serialize(self)
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, bincode::Error> {
        bincode::deserialize(bytes)
    }
}

// ── Protobuf wire format ────────────────────────────────────────────────────

pub fn serialize_raft_message(message: &Message) -> Result<Vec<u8>, protobuf::Error> {
    message.write_to_bytes()
}

pub fn deserialize_raft_message(bytes: &[u8]) -> Result<Message, protobuf::Error> {
    Message::parse_from_bytes(bytes)
}

// ── Configuration ───────────────────────────────────────────────────────────

/// Tunable Raft protocol parameters.
///
/// Default values are tuned for a 100 ms tick interval on a lossy wireless
/// network: `election_tick=15` → 1.5 s election timeout, `heartbeat_tick=3`
/// → 300 ms heartbeat.  Increasing `election_tick` prevents spurious
/// elections under packet loss; lowering it accelerates leader recovery.
#[derive(Debug, Clone)]
pub struct ConsensusConfig {
    /// Number of ticks before a follower starts an election.
    pub election_tick: usize,
    /// Number of ticks between leader heartbeats.
    pub heartbeat_tick: usize,
    /// Maximum bytes per Raft message.
    pub max_size_per_msg: u64,
    /// Maximum in-flight Raft messages per peer.
    pub max_inflight_msgs: usize,
    /// Leader steps down if it cannot reach a quorum within election_tick.
    pub check_quorum: bool,
    /// Use pre-vote to avoid term inflation from partitioned nodes.
    pub pre_vote: bool,
}

impl Default for ConsensusConfig {
    fn default() -> Self {
        Self {
            election_tick: 15,
            heartbeat_tick: 3,
            max_size_per_msg: 1024 * 1024,
            max_inflight_msgs: 64,
            check_quorum: true,
            pre_vote: true,
        }
    }
}

// ── Error types ─────────────────────────────────────────────────────────────

#[derive(Debug)]
pub enum ConsensusError {
    InvalidConfig(String),
    Raft(raft::Error),
    MessageSerialization(String),
    DuplicateMessage,
}

impl From<raft::Error> for ConsensusError {
    fn from(value: raft::Error) -> Self {
        Self::Raft(value)
    }
}

impl std::fmt::Display for ConsensusError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidConfig(s) => write!(f, "invalid config: {s}"),
            Self::Raft(e) => write!(f, "raft: {e}"),
            Self::MessageSerialization(s) => write!(f, "serialization: {s}"),
            Self::DuplicateMessage => write!(f, "duplicate message"),
        }
    }
}

// ── Core consensus node ─────────────────────────────────────────────────────

pub struct MissionConsensus<S: Storage> {
    node_id: u64,
    raft: RawNode<S>,
    outgoing_messages: VecDeque<Message>,
    pending_state: Option<MissionCommand>,
    applied_state: MissionState,
    applied_epoch: u64,
}

impl<S: Storage> MissionConsensus<S> {
    /// Create with default configuration (election_tick=15, heartbeat_tick=3).
    pub fn new(node_id: u64, peers: &[u64], storage: S) -> Result<Self, ConsensusError> {
        Self::with_config(node_id, peers, storage, ConsensusConfig::default())
    }

    /// Create with custom configuration.
    pub fn with_config(
        node_id: u64,
        peers: &[u64],
        storage: S,
        config: ConsensusConfig,
    ) -> Result<Self, ConsensusError> {
        if peers.len() < 6 {
            return Err(ConsensusError::InvalidConfig(
                "mission consensus requires at least 6 drones".to_string(),
            ));
        }

        if !peers.contains(&node_id) {
            return Err(ConsensusError::InvalidConfig(format!(
                "node_id {node_id} is not part of configured peers"
            )));
        }

        let cfg = Config {
            id: node_id,
            election_tick: config.election_tick,
            heartbeat_tick: config.heartbeat_tick,
            max_size_per_msg: config.max_size_per_msg,
            max_inflight_msgs: config.max_inflight_msgs,
            check_quorum: config.check_quorum,
            pre_vote: config.pre_vote,
            ..Default::default()
        };
        cfg.validate()
            .map_err(|e| ConsensusError::InvalidConfig(e.to_string()))?;

        let mut raft = RawNode::new(&cfg, storage, vec![])?;

        if raft.raft.raft_log.last_index() == 0 {
            for id in peers {
                let mut conf_change = ConfChange::new();
                conf_change.set_change_type(ConfChangeType::AddNode);
                conf_change.set_node_id(*id);
                raft.apply_conf_change(&conf_change)?;
            }
        }

        Ok(Self {
            node_id,
            raft,
            outgoing_messages: VecDeque::new(),
            pending_state: None,
            applied_state: MissionState::Transit,
            applied_epoch: 0,
        })
    }

    // ── Accessors ───────────────────────────────────────────────────────

    pub fn node_id(&self) -> u64 {
        self.node_id
    }

    pub fn is_leader(&self) -> bool {
        self.raft.raft.state == StateRole::Leader
    }

    /// Current Raft role (Follower, Candidate, Leader, PreCandidate).
    pub fn role(&self) -> StateRole {
        self.raft.raft.state
    }

    /// Current Raft term.
    pub fn term(&self) -> u64 {
        self.raft.raft.term
    }

    /// ID of the current leader, or `None` during election.
    pub fn leader_id(&self) -> Option<u64> {
        let id = self.raft.raft.leader_id;
        if id == 0 {
            None
        } else {
            Some(id)
        }
    }

    /// Last committed mission state.
    pub fn current_state(&self) -> MissionState {
        self.applied_state
    }

    /// Last committed mission epoch.
    pub fn applied_epoch(&self) -> u64 {
        self.applied_epoch
    }

    // ── Raft operations ─────────────────────────────────────────────────

    pub fn tick(&mut self) {
        self.raft.tick();
        self.collect_ready();
    }

    pub fn step(&mut self, message: Message) -> Result<(), ConsensusError> {
        self.raft.step(message)?;
        self.collect_ready();
        Ok(())
    }

    pub fn propose_state(&mut self, command: MissionCommand) -> Result<(), ConsensusError> {
        let payload = command
            .encode()
            .map_err(|e| ConsensusError::MessageSerialization(e.to_string()))?;
        self.pending_state = Some(command);
        self.raft
            .propose(vec![], payload)
            .map_err(ConsensusError::from)?;
        self.collect_ready();
        Ok(())
    }

    pub fn drain_outgoing(&mut self) -> Vec<Message> {
        self.outgoing_messages.drain(..).collect()
    }

    fn collect_ready(&mut self) {
        if !self.raft.has_ready() {
            return;
        }

        let mut ready = self.raft.ready();
        self.outgoing_messages.extend(ready.take_messages());

        for entry in ready.committed_entries() {
            self.apply_entry(entry);
        }

        self.raft.advance(ready);
    }

    fn apply_entry(&mut self, entry: &Entry) {
        if entry.get_entry_type() != EntryType::EntryNormal {
            return;
        }

        if entry.data.is_empty() {
            return;
        }

        if let Ok(command) = MissionCommand::decode(entry.get_data()) {
            if command.epoch >= self.applied_epoch {
                self.applied_epoch = command.epoch;
                self.applied_state = command.state;
                self.pending_state = None;
            }
        }
    }
}

// ── In-memory multi-node harness (for tests and simulation) ─────────────────

/// Route messages between N in-memory Raft nodes.
/// Returns the number of messages delivered.
pub fn route_messages<S: Storage>(nodes: &mut [MissionConsensus<S>]) -> usize {
    let mut pending: Vec<Message> = Vec::new();
    for node in nodes.iter_mut() {
        pending.extend(node.drain_outgoing());
    }
    let count = pending.len();
    for msg in pending {
        let to = msg.get_to();
        if let Some(target) = nodes.iter_mut().find(|n| n.node_id() == to) {
            let _ = target.step(msg);
        }
    }
    count
}

/// Tick all nodes and route messages until a leader is elected or max_rounds.
/// Returns `Some(leader_id)` if a leader was elected.
pub fn elect_leader<S: Storage>(
    nodes: &mut [MissionConsensus<S>],
    max_rounds: usize,
) -> Option<u64> {
    for _ in 0..max_rounds {
        for node in nodes.iter_mut() {
            node.tick();
        }
        route_messages(nodes);
        if let Some(leader) = nodes.iter().find(|n| n.is_leader()) {
            return Some(leader.node_id());
        }
    }
    None
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use raft::storage::MemStorage;

    fn make_cluster(peers: &[u64]) -> Vec<MissionConsensus<MemStorage>> {
        peers
            .iter()
            .map(|&id| {
                let storage = MemStorage::new_with_conf_state((peers.to_vec(), vec![]));
                MissionConsensus::new(id, peers, storage).unwrap()
            })
            .collect()
    }

    fn make_cluster_with_config(
        peers: &[u64],
        config: ConsensusConfig,
    ) -> Vec<MissionConsensus<MemStorage>> {
        peers
            .iter()
            .map(|&id| {
                let storage = MemStorage::new_with_conf_state((peers.to_vec(), vec![]));
                MissionConsensus::with_config(id, peers, storage, config.clone()).unwrap()
            })
            .collect()
    }

    // -- preserved tests --

    #[test]
    fn mission_command_round_trip() {
        let cmd = MissionCommand {
            epoch: 9,
            state: MissionState::Lift,
        };
        let encoded = cmd.encode().unwrap();
        let decoded = MissionCommand::decode(&encoded).unwrap();
        assert_eq!(decoded, cmd);
    }

    #[test]
    fn raft_message_round_trip() {
        let mut msg = Message::new();
        msg.set_from(1);
        msg.set_to(2);
        msg.set_term(4);
        let encoded = serialize_raft_message(&msg).unwrap();
        let decoded = deserialize_raft_message(&encoded).unwrap();
        assert_eq!(decoded.get_from(), 1);
        assert_eq!(decoded.get_to(), 2);
        assert_eq!(decoded.get_term(), 4);
    }

    #[test]
    fn rejects_cluster_smaller_than_six() {
        let storage = MemStorage::new_with_conf_state((vec![1, 2, 3], vec![]));
        let result = MissionConsensus::new(1, &[1, 2, 3], storage);
        assert!(matches!(result, Err(ConsensusError::InvalidConfig(_))));
    }

    #[test]
    fn accepts_six_drone_cluster() {
        let peers = [1, 2, 3, 4, 5, 6];
        let storage = MemStorage::new_with_conf_state((peers.to_vec(), vec![]));
        let consensus = MissionConsensus::new(1, &peers, storage).unwrap();
        assert_eq!(consensus.current_state(), MissionState::Transit);
        assert_eq!(consensus.node_id(), 1);
    }

    // -- new tests --

    #[test]
    fn configurable_election_tick() {
        let peers = [1, 2, 3, 4, 5, 6];
        let config = ConsensusConfig {
            election_tick: 20,
            heartbeat_tick: 5,
            ..Default::default()
        };
        let storage = MemStorage::new_with_conf_state((peers.to_vec(), vec![]));
        let c = MissionConsensus::with_config(1, &peers, storage, config).unwrap();
        assert_eq!(c.current_state(), MissionState::Transit);
    }

    #[test]
    fn role_starts_as_follower() {
        let peers = [1, 2, 3, 4, 5, 6];
        let storage = MemStorage::new_with_conf_state((peers.to_vec(), vec![]));
        let c = MissionConsensus::new(1, &peers, storage).unwrap();
        assert_eq!(c.role(), StateRole::Follower);
        assert_eq!(c.leader_id(), None);
    }

    #[test]
    fn term_starts_at_zero() {
        let peers = [1, 2, 3, 4, 5, 6];
        let storage = MemStorage::new_with_conf_state((peers.to_vec(), vec![]));
        let c = MissionConsensus::new(1, &peers, storage).unwrap();
        // Initial term is implementation-defined but should be small
        assert!(c.term() <= 1);
    }

    #[test]
    fn six_node_leader_election() {
        let peers = [1, 2, 3, 4, 5, 6];
        let mut nodes = make_cluster(&peers);
        let leader = elect_leader(&mut nodes, 50);
        assert!(leader.is_some(), "should elect a leader within 50 rounds");
        // All nodes should agree on the leader
        let lid = leader.unwrap();
        for node in &nodes {
            assert_eq!(node.leader_id(), Some(lid));
        }
    }

    #[test]
    fn propose_and_commit() {
        let peers = [1, 2, 3, 4, 5, 6];
        let mut nodes = make_cluster(&peers);

        // Elect leader first
        let lid = elect_leader(&mut nodes, 50).expect("leader election");

        // Propose a state change from the leader
        let leader = nodes.iter_mut().find(|n| n.node_id() == lid).unwrap();
        leader
            .propose_state(MissionCommand {
                epoch: 1,
                state: MissionState::Lift,
            })
            .unwrap();

        // Tick and route until all nodes commit
        for _ in 0..20 {
            for node in nodes.iter_mut() {
                node.tick();
            }
            route_messages(&mut nodes);
        }

        // All nodes should have the committed state
        for node in &nodes {
            assert_eq!(
                node.current_state(),
                MissionState::Lift,
                "node {} should be Lift",
                node.node_id()
            );
            assert_eq!(node.applied_epoch(), 1);
        }
    }

    #[test]
    fn network_partition_minority_cannot_commit() {
        let peers = [1, 2, 3, 4, 5, 6];
        let mut nodes = make_cluster(&peers);

        // Elect leader
        let lid = elect_leader(&mut nodes, 50).expect("leader election");

        // Partition: isolate 2 nodes (the minority)
        // Stable alternative to drain_filter (nightly-only)
        let mut minority = Vec::new();
        let mut majority = Vec::new();
        for node in nodes.into_iter() {
            if node.node_id() > 4 {
                minority.push(node);
            } else {
                majority.push(node);
            }
        }
        let mut nodes = majority;

        // Majority can still elect and commit
        for _ in 0..30 {
            for node in nodes.iter_mut() {
                node.tick();
            }
            route_messages(&mut nodes);
        }

        // Majority should have a leader
        assert!(nodes.iter().any(|n| n.is_leader()));

        // Minority should not have a leader (check_quorum + pre_vote)
        for _ in 0..30 {
            for node in minority.iter_mut() {
                node.tick();
            }
            route_messages(&mut minority);
        }
        // With only 2 nodes out of 6, quorum is 4 — minority can't elect
        assert!(!minority.iter().any(|n| n.is_leader()));
    }

    #[test]
    fn duplicate_raft_message_is_harmless() {
        let peers = [1, 2, 3, 4, 5, 6];
        let mut nodes = make_cluster(&peers);

        // Tick once to generate messages
        for node in nodes.iter_mut() {
            node.tick();
        }
        let msgs: Vec<Message> = nodes[0].drain_outgoing();

        // Deliver each message twice — should not panic
        for msg in &msgs {
            let to = msg.get_to();
            if let Some(target) = nodes.iter_mut().find(|n| n.node_id() == to) {
                let _ = target.step(msg.clone());
                let _ = target.step(msg.clone()); // duplicate
            }
        }
    }

    #[test]
    fn stale_term_message_ignored() {
        let peers = [1, 2, 3, 4, 5, 6];
        let mut nodes = make_cluster(&peers);

        // Elect leader to advance term
        elect_leader(&mut nodes, 50).expect("leader election");
        let current_term = nodes[0].term();

        // Craft a message with term 0 (stale)
        let mut stale_msg = Message::new();
        stale_msg.set_from(2);
        stale_msg.set_to(1);
        stale_msg.set_term(0);

        // Should not panic or change state
        let state_before = nodes[0].current_state();
        let _ = nodes[0].step(stale_msg);
        assert_eq!(nodes[0].current_state(), state_before);
        assert!(nodes[0].term() >= current_term);
    }
}
