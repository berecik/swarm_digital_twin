use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use zenoh::handlers::FifoChannelHandler;
use zenoh::pubsub::Subscriber;
use zenoh::sample::Sample;
use zenoh::Session;
use crate::boids::Boid;
use crate::consensus_transport::{raft_tx_topic, raft_tx_subscribe_pattern};
use bincode;

pub struct ZenohManager {
    session: Session,
    pub neighbors: Arc<Mutex<HashMap<String, Boid>>>,
    _drone_id: String,
    drone_id_numeric: u64,
    namespace: String,
}

impl ZenohManager {
    pub async fn new(drone_id: String) -> Self {
        let session = zenoh::open(zenoh::Config::default()).await.unwrap();
        let neighbors = Arc::new(Mutex::new(HashMap::new()));
        let namespace = format!("swarm/{}/", drone_id);

        let neighbors_clone = Arc::clone(&neighbors);
        let my_id = drone_id.clone();

        let subscriber = session
            .declare_subscriber("swarm/*/state")
            .await
            .unwrap();

        tokio::spawn(async move {
            while let Ok(sample) = subscriber.recv_async().await {
                Self::process_sample(&sample, &my_id, &neighbors_clone);
            }
        });

        let drone_id_numeric = drone_id
            .strip_prefix("drone_")
            .and_then(|s| s.parse::<u64>().ok())
            .unwrap_or(0);

        ZenohManager {
            session,
            neighbors,
            _drone_id: drone_id,
            drone_id_numeric,
            namespace,
        }
    }

    pub fn process_sample(sample: &Sample, my_id: &str, neighbors: &Arc<Mutex<HashMap<String, Boid>>>) {
        let key = sample.key_expr().as_str();
        let payload = sample.payload().to_bytes();
        Self::process_raw_data(key, &payload, my_id, neighbors);
    }

    pub fn process_raw_data(key: &str, payload: &[u8], my_id: &str, neighbors: &Arc<Mutex<HashMap<String, Boid>>>) {
        let parts: Vec<&str> = key.split('/').collect();
        if parts.len() >= 2 {
            let sender_id = parts[1].to_string();
            if sender_id != my_id {
                if let Ok(boid) = bincode::deserialize::<Boid>(payload) {
                    let mut n = neighbors.lock().unwrap();
                    n.insert(sender_id, boid);
                }
            }
        }
    }

    pub async fn publish_state(&self, boid: &Boid) {
        let key = format!("{}state", self.namespace);
        if let Ok(payload) = bincode::serialize(boid) {
            self.session.put(&key, payload).await.unwrap();
        }
    }

    // ── Consensus (Raft) pub/sub ────────────────────────────────────────

    /// Publish a serialized Raft message to a specific target drone's
    /// consensus topic: `/swarm/drone_{target_id}/consensus/raft_tx`.
    pub async fn publish_raft_message(
        &self,
        target_id: u64,
        payload: Vec<u8>,
    ) -> Result<(), String> {
        let topic = raft_tx_topic(target_id);
        self.session
            .put(&topic, payload)
            .await
            .map_err(|e| format!("zenoh put to {topic}: {e}"))
    }

    /// Publish a batch of outgoing Raft messages (from `ConsensusTransportLoop::drain_outbox`).
    pub async fn publish_raft_batch(
        &self,
        messages: Vec<(u64, Vec<u8>)>,
    ) -> Result<usize, String> {
        let mut sent = 0;
        for (target_id, payload) in messages {
            self.publish_raft_message(target_id, payload).await?;
            sent += 1;
        }
        Ok(sent)
    }

    /// Subscribe to all drones' Raft consensus traffic.
    pub async fn subscribe_raft(&self) -> Subscriber<FifoChannelHandler<Sample>> {
        let pattern = raft_tx_subscribe_pattern();
        self.session
            .declare_subscriber(&pattern)
            .await
            .unwrap()
    }

    /// Get this manager's numeric drone ID.
    pub fn drone_id_numeric(&self) -> u64 {
        self.drone_id_numeric
    }

    /// Access the underlying Zenoh session.
    pub fn session(&self) -> &Session {
        &self.session
    }
}
