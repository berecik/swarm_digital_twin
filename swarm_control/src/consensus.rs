use std::collections::VecDeque;

use protobuf::Message as ProtobufMessage;
use raft::eraftpb::{ConfChange, ConfChangeType, Entry, EntryType, Message};
use raft::{Config, RawNode, StateRole, Storage};
use serde::{Deserialize, Serialize};

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

pub fn serialize_raft_message(message: &Message) -> Result<Vec<u8>, protobuf::Error> {
    message.write_to_bytes()
}

pub fn deserialize_raft_message(bytes: &[u8]) -> Result<Message, protobuf::Error> {
    Message::parse_from_bytes(bytes)
}

#[derive(Debug)]
pub enum ConsensusError {
    InvalidConfig(String),
    Raft(raft::Error),
}

impl From<raft::Error> for ConsensusError {
    fn from(value: raft::Error) -> Self {
        Self::Raft(value)
    }
}

pub struct MissionConsensus<S: Storage> {
    node_id: u64,
    raft: RawNode<S>,
    outgoing_messages: VecDeque<Message>,
    pending_state: Option<MissionCommand>,
    applied_state: MissionState,
    applied_epoch: u64,
}

impl<S: Storage> MissionConsensus<S> {
    pub fn new(node_id: u64, peers: &[u64], storage: S) -> Result<Self, ConsensusError> {
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
            election_tick: 10,
            heartbeat_tick: 2,
            max_size_per_msg: 1024 * 1024,
            max_inflight_msgs: 64,
            check_quorum: true,
            pre_vote: true,
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

    pub fn node_id(&self) -> u64 {
        self.node_id
    }

    pub fn is_leader(&self) -> bool {
        self.raft.raft.state == StateRole::Leader
    }

    pub fn current_state(&self) -> MissionState {
        self.applied_state
    }

    pub fn tick(&mut self) {
        self.raft.tick();
        self.collect_ready();
    }

    pub fn step(&mut self, message: Message) -> Result<(), raft::Error> {
        self.raft.step(message)?;
        self.collect_ready();
        Ok(())
    }

    pub fn propose_state(&mut self, command: MissionCommand) -> Result<(), raft::Error> {
        let payload = command
            .encode()
            .map_err(|e| raft::Error::CodecError(e.to_string()))?;
        self.pending_state = Some(command);
        self.raft.propose(vec![], payload)?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use raft::storage::MemStorage;

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
}
