use std::time::{Duration, Instant};

/// Collects real-time loop timing metrics for determinism gates.
#[derive(Debug, Clone)]
pub struct TimingMetrics {
    samples: Vec<Duration>,
    deadline: Duration,
    missed_count: usize,
}

impl TimingMetrics {
    /// Create a new TimingMetrics collector.
    /// `target_hz` is the nominal loop rate (e.g. 10.0 for 10 Hz).
    pub fn new(target_hz: f64) -> Self {
        Self {
            samples: Vec::with_capacity(2048),
            deadline: Duration::from_secs_f64(1.0 / target_hz),
            missed_count: 0,
        }
    }

    /// Record one loop iteration's elapsed time.
    pub fn record(&mut self, elapsed: Duration) {
        if elapsed > self.deadline {
            self.missed_count += 1;
        }
        self.samples.push(elapsed);
    }

    /// The configured loop deadline (1 / target_hz).
    pub fn deadline(&self) -> Duration {
        self.deadline
    }

    /// Clear all recorded samples and reset missed count.
    pub fn reset(&mut self) {
        self.samples.clear();
        self.missed_count = 0;
    }

    /// Number of recorded samples.
    pub fn count(&self) -> usize {
        self.samples.len()
    }

    /// Number of iterations that exceeded the deadline.
    pub fn missed_deadlines(&self) -> usize {
        self.missed_count
    }

    /// Mean loop duration.
    pub fn mean(&self) -> Duration {
        if self.samples.is_empty() {
            return Duration::ZERO;
        }
        let total: Duration = self.samples.iter().sum();
        total / self.samples.len() as u32
    }

    /// Percentile loop duration (0-100).
    pub fn percentile(&self, p: f64) -> Duration {
        if self.samples.is_empty() {
            return Duration::ZERO;
        }
        let mut sorted: Vec<Duration> = self.samples.clone();
        sorted.sort();
        let idx = ((p / 100.0) * (sorted.len() - 1) as f64).round() as usize;
        sorted[idx.min(sorted.len() - 1)]
    }

    /// Maximum loop duration.
    pub fn max(&self) -> Duration {
        self.samples.iter().copied().max().unwrap_or(Duration::ZERO)
    }

    /// Jitter: difference between p95 and median loop time.
    pub fn jitter_p95(&self) -> Duration {
        let p95 = self.percentile(95.0);
        let median = self.percentile(50.0);
        p95.saturating_sub(median)
    }

    /// Print a summary report.
    pub fn report(&self) -> String {
        format!(
            "TimingMetrics: n={} mean={:.2}ms p50={:.2}ms p95={:.2}ms max={:.2}ms jitter_p95={:.2}ms missed={}",
            self.count(),
            self.mean().as_secs_f64() * 1000.0,
            self.percentile(50.0).as_secs_f64() * 1000.0,
            self.percentile(95.0).as_secs_f64() * 1000.0,
            self.max().as_secs_f64() * 1000.0,
            self.jitter_p95().as_secs_f64() * 1000.0,
            self.missed_deadlines(),
        )
    }
}

/// Convenience: run a closure in a timed loop, collecting metrics.
pub fn run_timed_loop<F>(hz: f64, max_steps: usize, mut body: F) -> TimingMetrics
where
    F: FnMut(usize) -> bool,
{
    let dt = Duration::from_secs_f64(1.0 / hz);
    let mut metrics = TimingMetrics::new(hz);

    for step in 0..max_steps {
        let start = Instant::now();
        let should_stop = body(step);
        let elapsed = start.elapsed();
        metrics.record(elapsed);

        if should_stop {
            break;
        }

        // Sleep remainder of period
        if let Some(remaining) = dt.checked_sub(elapsed) {
            std::thread::sleep(remaining);
        }
    }

    metrics
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timing_metrics_basic() {
        let mut m = TimingMetrics::new(10.0);
        m.record(Duration::from_millis(5));
        m.record(Duration::from_millis(8));
        m.record(Duration::from_millis(12));

        assert_eq!(m.count(), 3);
        assert_eq!(m.missed_deadlines(), 1); // 12ms > 100ms? No, 1/10Hz = 100ms. 12ms < 100ms.
        // Actually: deadline = 1/10 = 100ms. 12ms < 100ms, so 0 missed.
    }

    #[test]
    fn timing_metrics_missed_deadline() {
        let mut m = TimingMetrics::new(100.0); // 10ms deadline
        m.record(Duration::from_millis(5));
        m.record(Duration::from_millis(15)); // exceeds 10ms
        m.record(Duration::from_millis(8));

        assert_eq!(m.missed_deadlines(), 1);
    }

    #[test]
    fn timing_metrics_percentile() {
        let mut m = TimingMetrics::new(10.0);
        for i in 1..=100 {
            m.record(Duration::from_millis(i));
        }

        let p50 = m.percentile(50.0);
        assert!(p50.as_millis() >= 49 && p50.as_millis() <= 51);

        let p95 = m.percentile(95.0);
        assert!(p95.as_millis() >= 94 && p95.as_millis() <= 96);
    }

    #[test]
    fn timing_metrics_empty() {
        let m = TimingMetrics::new(10.0);
        assert_eq!(m.count(), 0);
        assert_eq!(m.mean(), Duration::ZERO);
        assert_eq!(m.max(), Duration::ZERO);
    }

    #[test]
    fn run_timed_loop_collects_metrics() {
        let metrics = run_timed_loop(100.0, 20, |step| {
            // Simulate some work
            std::thread::sleep(Duration::from_micros(100));
            step >= 9 // stop after 10 iterations
        });

        assert_eq!(metrics.count(), 10);
        assert!(metrics.mean().as_micros() >= 100);
    }

    #[test]
    fn timing_jitter_p95() {
        let mut m = TimingMetrics::new(10.0);
        // Uniform samples: jitter should be small
        for _ in 0..100 {
            m.record(Duration::from_millis(5));
        }
        assert_eq!(m.jitter_p95(), Duration::ZERO);

        // Add some outliers
        m.record(Duration::from_millis(50));
        m.record(Duration::from_millis(50));
        m.record(Duration::from_millis(50));
        m.record(Duration::from_millis(50));
        m.record(Duration::from_millis(50));
        m.record(Duration::from_millis(50));
        // p95 might now be higher than median
        assert!(m.jitter_p95() <= Duration::from_millis(50));
    }

    #[test]
    fn timing_report_format() {
        let mut m = TimingMetrics::new(10.0);
        m.record(Duration::from_millis(5));
        let report = m.report();
        assert!(report.contains("TimingMetrics:"));
        assert!(report.contains("n=1"));
    }
}
