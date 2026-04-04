use swarm_control_core::driver_core::{DriverCore, FlightState};
use swarm_control_core::timing::{run_timed_loop, TimingMetrics};
use swarm_control_core::transport::{InProcessTransport, SimTransport, Transport};

const TARGET_HZ: f64 = 10.0;
const MAX_STEPS: usize = 150;
const TAKEOFF_HEIGHT: f32 = 5.0;

fn run_with_transport(transport: &mut dyn Transport) -> TimingMetrics {
    let mut core = DriverCore::new(TAKEOFF_HEIGHT);

    println!("[sim_realtime_driver] starting {TARGET_HZ}Hz loop using shared DriverCore");

    let metrics = run_timed_loop(TARGET_HZ, MAX_STEPS, |step| {
        let status = transport.recv_status().unwrap_or_default();
        core.update_status(status);
        let actions = core.tick_simple();

        println!(
            "step={step:03} state={:?} actions={actions:?}",
            core.flight_state
        );

        let _ = transport.send_actions(&actions);

        // Stop when loiter reached and near target altitude
        if core.flight_state == FlightState::Loiter {
            if let Ok(s) = transport.recv_status() {
                // For InProcessTransport, position is tracked internally.
                // For SimTransport, we rely on the status alone (check is in Python side).
                let _ = s; // status-only check: stop after enough loiter ticks
            }
            // Simple heuristic: stop after 20 ticks in loiter
            return step >= 20;
        }
        false
    });

    println!("[sim_realtime_driver] done");
    println!("{}", metrics.report());
    metrics
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    if args.len() > 1 && args[1] == "--udp" {
        let addr = args.get(2).map(|s| s.as_str()).unwrap_or("127.0.0.1:9100");
        println!("[sim_realtime_driver] connecting to Python sim bridge at {addr}");
        match SimTransport::new(addr, 500) {
            Ok(mut transport) => {
                run_with_transport(&mut transport);
            }
            Err(e) => {
                eprintln!("Failed to connect: {e}");
                std::process::exit(1);
            }
        }
    } else {
        println!("[sim_realtime_driver] using in-process transport (alpha=0.2)");
        let mut transport = InProcessTransport::new(0.2);
        run_with_transport(&mut transport);
    }
}
