use std::fmt;
use std::io;
use std::net::UdpSocket;
use std::time::Duration;

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::driver_core::{DriverAction, DriverStatus};

// ── Error type ───────────────────────────────────────────────────────────

#[derive(Debug)]
pub enum TransportError {
    Io(io::Error),
    Serialization(String),
    Timeout,
}

impl fmt::Display for TransportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TransportError::Io(e) => write!(f, "transport I/O error: {e}"),
            TransportError::Serialization(e) => write!(f, "transport serialization error: {e}"),
            TransportError::Timeout => write!(f, "transport timeout"),
        }
    }
}

impl std::error::Error for TransportError {}

impl From<io::Error> for TransportError {
    fn from(e: io::Error) -> Self {
        if e.kind() == io::ErrorKind::WouldBlock || e.kind() == io::ErrorKind::TimedOut {
            TransportError::Timeout
        } else {
            TransportError::Io(e)
        }
    }
}

// ── Wire messages (JSON over UDP) ────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ActionMessage {
    RequestOffboard,
    RequestArm,
    PublishSetpoint { x: f32, y: f32, z: f32 },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatusMessage {
    pub nav_state: u8,
    pub arming_state: u8,
    pub position: [f32; 3],
}

impl ActionMessage {
    pub fn from_driver_action(action: &DriverAction) -> Self {
        match action {
            DriverAction::RequestOffboard => ActionMessage::RequestOffboard,
            DriverAction::RequestArm => ActionMessage::RequestArm,
            DriverAction::PublishSetpoint(v) => ActionMessage::PublishSetpoint {
                x: v.x,
                y: v.y,
                z: v.z,
            },
        }
    }

    pub fn to_driver_action(&self) -> DriverAction {
        match self {
            ActionMessage::RequestOffboard => DriverAction::RequestOffboard,
            ActionMessage::RequestArm => DriverAction::RequestArm,
            ActionMessage::PublishSetpoint { x, y, z } => {
                DriverAction::PublishSetpoint(Vector3::new(*x, *y, *z))
            }
        }
    }
}

impl StatusMessage {
    pub fn to_driver_status(&self) -> DriverStatus {
        DriverStatus {
            nav_state: self.nav_state,
            arming_state: self.arming_state,
        }
    }
}

// ── Transport trait ──────────────────────────────────────────────────────

pub trait Transport {
    /// Send a batch of actions produced by DriverCore::tick().
    fn send_actions(&mut self, actions: &[DriverAction]) -> Result<(), TransportError>;

    /// Receive the latest vehicle status.
    fn recv_status(&mut self) -> Result<DriverStatus, TransportError>;
}

// ── InProcessTransport (replaces the old SimVehicle inline) ──────────────

pub struct InProcessTransport {
    pub position: [f32; 3],
    nav_state: u8,
    arming_state: u8,
    alpha: f32,
}

impl InProcessTransport {
    pub fn new(alpha: f32) -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            nav_state: 0,
            arming_state: 0,
            alpha,
        }
    }
}

impl Transport for InProcessTransport {
    fn send_actions(&mut self, actions: &[DriverAction]) -> Result<(), TransportError> {
        for action in actions {
            match action {
                DriverAction::RequestOffboard => {
                    self.nav_state = 14;
                }
                DriverAction::RequestArm => {
                    self.arming_state = 2;
                }
                DriverAction::PublishSetpoint(target) => {
                    self.position[0] += (target.x - self.position[0]) * self.alpha;
                    self.position[1] += (target.y - self.position[1]) * self.alpha;
                    self.position[2] += (target.z - self.position[2]) * self.alpha;
                }
            }
        }
        Ok(())
    }

    fn recv_status(&mut self) -> Result<DriverStatus, TransportError> {
        Ok(DriverStatus {
            nav_state: self.nav_state,
            arming_state: self.arming_state,
        })
    }
}

// ── SimTransport (UDP bridge to Python physics) ──────────────────────────

pub struct SimTransport {
    socket: UdpSocket,
    buf: Vec<u8>,
}

impl SimTransport {
    /// Create a SimTransport that sends actions to `remote_addr` and listens
    /// for status responses on the same socket.
    ///
    /// `remote_addr` is typically `"127.0.0.1:9100"` where the Python
    /// `sim_bridge.py` server is listening.
    pub fn new(remote_addr: &str, timeout_ms: u64) -> Result<Self, TransportError> {
        let socket = UdpSocket::bind("127.0.0.1:0")?;
        socket.connect(remote_addr)?;
        socket.set_read_timeout(Some(Duration::from_millis(timeout_ms)))?;
        socket.set_write_timeout(Some(Duration::from_millis(timeout_ms)))?;
        Ok(Self {
            socket,
            buf: vec![0u8; 4096],
        })
    }
}

impl Transport for SimTransport {
    fn send_actions(&mut self, actions: &[DriverAction]) -> Result<(), TransportError> {
        let messages: Vec<ActionMessage> = actions.iter().map(ActionMessage::from_driver_action).collect();
        let json = serde_json::to_string(&messages)
            .map_err(|e| TransportError::Serialization(e.to_string()))?;
        self.socket.send(json.as_bytes())?;
        Ok(())
    }

    fn recv_status(&mut self) -> Result<DriverStatus, TransportError> {
        let n = self.socket.recv(&mut self.buf)?;
        let msg: StatusMessage = serde_json::from_slice(&self.buf[..n])
            .map_err(|e| TransportError::Serialization(e.to_string()))?;
        Ok(msg.to_driver_status())
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::driver_core::DriverCore;

    #[test]
    fn in_process_transport_full_lifecycle() {
        let mut core = DriverCore::new(5.0);
        let mut transport = InProcessTransport::new(0.2);

        // Run through the FSM until loiter
        for _ in 0..50 {
            let status = transport.recv_status().unwrap();
            core.update_status(status);
            // Feed transport position back into driver for takeoff detection
            let pos = transport.position;
            core.update_position(nalgebra::Vector3::new(pos[0], pos[1], pos[2]));
            let actions = core.tick_simple();
            transport.send_actions(&actions).unwrap();
        }

        assert_eq!(core.flight_state, crate::driver_core::FlightState::Loiter);
        assert!(transport.position[2] > 0.0, "should have moved up");
    }

    #[test]
    fn action_message_roundtrip() {
        let actions = vec![
            DriverAction::RequestOffboard,
            DriverAction::RequestArm,
            DriverAction::PublishSetpoint(Vector3::new(1.0, 2.0, 3.0)),
        ];

        for action in &actions {
            let msg = ActionMessage::from_driver_action(action);
            let json = serde_json::to_string(&msg).unwrap();
            let decoded: ActionMessage = serde_json::from_str(&json).unwrap();
            assert_eq!(decoded.to_driver_action(), *action);
        }
    }

    #[test]
    fn status_message_roundtrip() {
        let msg = StatusMessage {
            nav_state: 14,
            arming_state: 2,
            position: [1.0, 2.0, 3.0],
        };
        let json = serde_json::to_string(&msg).unwrap();
        let decoded: StatusMessage = serde_json::from_str(&json).unwrap();
        let status = decoded.to_driver_status();
        assert_eq!(status.nav_state, 14);
        assert_eq!(status.arming_state, 2);
    }

    #[test]
    fn in_process_transport_converges_to_target() {
        let mut transport = InProcessTransport::new(0.5);
        let target = DriverAction::PublishSetpoint(Vector3::new(0.0, 0.0, 5.0));

        for _ in 0..50 {
            transport.send_actions(&[target.clone()]).unwrap();
        }

        assert!((transport.position[2] - 5.0).abs() < 0.01);
    }
}
