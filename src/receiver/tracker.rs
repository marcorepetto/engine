// tracker.rs — Kalman filter tracker for robots and ball
// Port of receivers/tracker.cpp + kalman/ekf.cpp

use nalgebra::{SMatrix, SVector};
use serde::Serialize;
use std::collections::HashMap;
use std::sync::{LazyLock, Mutex};

#[derive(Serialize, Clone, Debug)]
pub struct EkfTelemetry {
    pub timestamp_ms: u64,
    pub team: i32,
    pub id: i32,
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub vx: f64,
    pub vy: f64,
    pub omega: f64,
    pub innovation_x: f64,
    pub innovation_y: f64,
    pub innovation_theta: f64,
    pub p_trace: f64,
    pub q_trace: f64,
    pub r_trace: f64,
}

pub struct TelemetryManager {
    pub enabled: bool,
    pub filepath: String,
    pub records: Vec<EkfTelemetry>,
}

impl TelemetryManager {
    pub const fn new() -> Self {
        Self {
            enabled: false,
            filepath: String::new(),
            records: Vec::new(),
        }
    }
}

pub static TELEMETRY: LazyLock<Mutex<TelemetryManager>> = LazyLock::new(|| {
    Mutex::new(TelemetryManager::new())
});

pub fn start_telemetry(filepath: String) {
    if let Ok(mut tm) = TELEMETRY.lock() {
        tm.enabled = true;
        tm.filepath = filepath;
        tm.records.clear();
        println!("[EKF Telemetry] Recording started. Target file: {}", tm.filepath);
    }
}

pub fn stop_telemetry() {
    let mut filepath = String::new();
    let mut records = Vec::new();
    let mut was_enabled = false;

    if let Ok(mut tm) = TELEMETRY.lock() {
        if tm.enabled {
            filepath = tm.filepath.clone();
            
            // Normalize path extension to .csv
            if filepath.ends_with(".jsonl") {
                filepath = filepath.replace(".jsonl", ".csv");
            } else if filepath.ends_with(".json") {
                filepath = filepath.replace(".json", ".csv");
            } else if !filepath.ends_with(".csv") {
                filepath.push_str(".csv");
            }

            records = std::mem::take(&mut tm.records);
            tm.enabled = false;
            was_enabled = true;
        }
    }

    if was_enabled && !filepath.is_empty() && !records.is_empty() {
        println!("[EKF Telemetry] Recording stopped. Writing {} records...", records.len());
        std::thread::spawn(move || {
            let path = std::path::Path::new(&filepath);
            if let Some(parent) = path.parent() {
                let _ = std::fs::create_dir_all(parent);
            }

            match csv::Writer::from_path(path) {
                Ok(mut writer) => {
                    let mut count = 0;
                    for record in records {
                        if writer.serialize(record).is_ok() {
                            count += 1;
                        }
                    }
                    let _ = writer.flush();
                    println!("[EKF Telemetry] Successfully wrote {} CSV records to {:?}", count, path);
                }
                Err(e) => {
                    eprintln!("[EKF Telemetry] Failed to create CSV writer for {:?}: {:?}", path, e);
                }
            }
        });
    } else {
        println!("[EKF Telemetry] Recording stop requested, but no active telemetry session was running.");
    }
}

/// 7-state Extended Kalman Filter: [x, y, sin(θ), cos(θ), vx, vy, ω]
pub struct ExtendedKalmanFilter {
    x: SVector<f64, 7>,          // State
    p: SMatrix<f64, 7, 7>,       // Covariance
    q: SMatrix<f64, 7, 7>,       // Process noise
    r: SMatrix<f64, 3, 3>,       // Measurement noise
}

impl ExtendedKalmanFilter {
    pub fn new(
        initial_state: SVector<f64, 7>,
        initial_cov: SMatrix<f64, 7, 7>,
        process_noise: SMatrix<f64, 7, 7>,
        measurement_noise: SMatrix<f64, 3, 3>,
    ) -> Self {
        Self {
            x: initial_state,
            p: initial_cov,
            q: process_noise,
            r: measurement_noise,
        }
    }

    pub fn predict(&mut self, dt: f64) {
        let f_jac = self.jacobian_f(dt);
        self.x = self.f(dt);
        self.p = f_jac * self.p * f_jac.transpose() + self.q;
    }

    pub fn update(&mut self, z: &SVector<f64, 3>) -> SVector<f64, 3> {
        let h_jac = self.jacobian_h();
        let mut y = z - self.h();
        y[2] = normalize_angle(y[2]);
        let innovation = y;

        let s = h_jac * self.p * h_jac.transpose() + self.r;
        let s_inv = s.try_inverse().unwrap_or_else(|| SMatrix::<f64, 3, 3>::identity());
        let k = self.p * h_jac.transpose() * s_inv;

        self.x = self.x + k * innovation;
        self.p = (SMatrix::<f64, 7, 7>::identity() - k * h_jac) * self.p;

        innovation
    }

    /// Combined predict + update, returns (x, y, θ, vx, vy, ω, innovation, p_trace, q_trace, r_trace)
    pub fn filter_pose(
        &mut self,
        x_meas: f64,
        y_meas: f64,
        theta_meas: f64,
        dt: f64,
    ) -> (f64, f64, f64, f64, f64, f64, SVector<f64, 3>, f64, f64, f64) {
        self.predict(dt);

        let z = SVector::<f64, 3>::new(x_meas, y_meas, theta_meas);
        let innovation = self.update(&z);

        let x_f = self.x[0];
        let y_f = self.x[1];
        let theta_f = self.x[2].atan2(self.x[3]);
        let vx = self.x[4];
        let vy = self.x[5];
        let omega = self.x[6];

        let p_trace = self.p.trace();
        let q_trace = self.q.trace();
        let r_trace = self.r.trace();

        (x_f, y_f, theta_f, vx, vy, omega, innovation, p_trace, q_trace, r_trace)
    }

    // State transition: [x, y, sin(θ), cos(θ), vx, vy, ω]
    fn f(&self, dt: f64) -> SVector<f64, 7> {
        let sin_theta = self.x[2];
        let cos_theta = self.x[3];
        let vx = self.x[4];
        let vy = self.x[5];
        let omega = self.x[6];

        let theta = sin_theta.atan2(cos_theta);
        let theta_new = theta + omega * dt;

        let mut new_x = self.x;
        new_x[0] += vx * dt;
        new_x[1] += vy * dt;
        new_x[2] = theta_new.sin();
        new_x[3] = theta_new.cos();
        new_x
    }

    fn h(&self) -> SVector<f64, 3> {
        let theta = self.x[2].atan2(self.x[3]);
        SVector::<f64, 3>::new(self.x[0], self.x[1], theta)
    }

    fn jacobian_f(&self, dt: f64) -> SMatrix<f64, 7, 7> {
        let mut f = SMatrix::<f64, 7, 7>::identity();
        f[(0, 4)] = dt;
        f[(1, 5)] = dt;

        let sin_theta = self.x[2];
        let cos_theta = self.x[3];
        let omega = self.x[6];
        let theta = sin_theta.atan2(cos_theta);
        let theta_new = theta + omega * dt;

        f[(2, 6)] = dt * theta_new.cos();
        f[(3, 6)] = -dt * theta_new.sin();

        f
    }

    fn jacobian_h(&self) -> SMatrix<f64, 3, 7> {
        let mut h = SMatrix::<f64, 3, 7>::zeros();
        h[(0, 0)] = 1.0;
        h[(1, 1)] = 1.0;

        let sin_theta = self.x[2];
        let cos_theta = self.x[3];
        let denom = sin_theta * sin_theta + cos_theta * cos_theta;

        if denom.abs() > 1e-12 {
            h[(2, 2)] = cos_theta / denom;
            h[(2, 3)] = -sin_theta / denom;
        }

        h
    }

    pub fn set_noise_parameters(&mut self, p_noise_p: f64, p_noise_v: f64, m_noise: f64) {
        // Position noise
        self.q[(0, 0)] = p_noise_p;
        self.q[(1, 1)] = p_noise_p;
        // Velocity noise
        self.q[(4, 4)] = p_noise_v;
        self.q[(5, 5)] = p_noise_v;

        // Measurement noise
        self.r[(0, 0)] = m_noise;
        self.r[(1, 1)] = m_noise;
        self.r[(2, 2)] = m_noise;
    }
}

fn normalize_angle(mut angle: f64) -> f64 {
    while angle > std::f64::consts::PI {
        angle -= 2.0 * std::f64::consts::PI;
    }
    while angle < -std::f64::consts::PI {
        angle += 2.0 * std::f64::consts::PI;
    }
    angle
}

/// Tracker: maintains per-robot EKF instances keyed by (team, id)
pub struct Tracker {
    filters: HashMap<(i32, i32), ExtendedKalmanFilter>,
    last_states: HashMap<(i32, i32), (f64, f64, f64)>, // (x, y, theta)
    enabled: bool,
    process_noise_p: f64,
    process_noise_v: f64,
    measurement_noise: f64,
}

impl Tracker {
    pub fn new() -> Self {
        Self {
            filters: HashMap::new(),
            last_states: HashMap::new(),
            enabled: true,
            process_noise_p: 1e-7,
            process_noise_v: 1e-4,
            measurement_noise: 1e-6,
        }
    }

    /// Track a robot/ball, returns (x, y, θ, vx, vy, ω).
    /// Position is returned unfiltered, velocity is filtered.
    pub fn track(
        &mut self,
        team: i32,
        id: i32,
        x: f64,
        y: f64,
        theta: f64,
        dt: f64,
    ) -> (f64, f64, f64, f64, f64, f64) {
        if !self.enabled {
            // Calculate simple derivative velocity
            let (last_x, last_y, last_theta) = self.last_states.get(&(team, id)).copied().unwrap_or((x, y, theta));
            let vx = if dt > 0.0 { (x - last_x) / dt } else { 0.0 };
            let vy = if dt > 0.0 { (y - last_y) / dt } else { 0.0 };

            let mut diff_theta = theta - last_theta;
            diff_theta = normalize_angle(diff_theta);
            let omega = if dt > 0.0 { diff_theta / dt } else { 0.0 };

            self.last_states.insert((team, id), (x, y, theta));

            return (x, y, theta, vx, vy, omega);
        }

        let process_noise_p = self.process_noise_p;
        let process_noise_v = self.process_noise_v;
        let measurement_noise = self.measurement_noise;

        let filter = self
            .filters
            .entry((team, id))
            .or_insert_with(|| Self::create_initial_filter(process_noise_p, process_noise_v, measurement_noise));

        filter.set_noise_parameters(process_noise_p, process_noise_v, measurement_noise);

        let (x_f, y_f, theta_f, vx, vy, omega, innovation, p_trace, q_trace, r_trace) = filter.filter_pose(x, y, theta, dt);

        if let Ok(mut tm) = TELEMETRY.lock() {
            if tm.enabled {
                let timestamp_ms = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_millis() as u64;

                tm.records.push(EkfTelemetry {
                    timestamp_ms,
                    team,
                    id,
                    x: x_f,
                    y: y_f,
                    theta: theta_f,
                    vx,
                    vy,
                    omega,
                    innovation_x: innovation[0],
                    innovation_y: innovation[1],
                    innovation_theta: innovation[2],
                    p_trace,
                    q_trace,
                    r_trace,
                });
            }
        }

        (x, y, theta, vx, vy, omega)
    }

    fn create_initial_filter(p_noise_p: f64, p_noise_v: f64, m_noise: f64) -> ExtendedKalmanFilter {
        let mut initial_state = SVector::<f64, 7>::zeros();
        initial_state[2] = 0.0_f64.sin();
        initial_state[3] = 0.0_f64.cos();

        let mut p = SMatrix::<f64, 7, 7>::zeros();
        p[(0, 0)] = 1e-7;
        p[(1, 1)] = 1e-7;
        p[(2, 2)] = 1e-7;
        p[(3, 3)] = 1e-7;
        p[(4, 4)] = 1.0;
        p[(5, 5)] = 1.0;
        p[(6, 6)] = 1.0;

        let mut q = SMatrix::<f64, 7, 7>::zeros();
        // Position noise
        q[(0, 0)] = p_noise_p;
        q[(1, 1)] = p_noise_p;
        // Angle noise (fixed for now or scaled?)
        q[(2, 2)] = 1e-4;
        q[(3, 3)] = 1e-4;
        // Velocity noise
        q[(4, 4)] = p_noise_v;
        q[(5, 5)] = p_noise_v;
        q[(6, 6)] = 1e-2;

        let mut r = SMatrix::<f64, 3, 3>::zeros();
        r[(0, 0)] = m_noise;
        r[(1, 1)] = m_noise;
        r[(2, 2)] = m_noise;

        ExtendedKalmanFilter::new(initial_state, p, q, r)
    }

    pub fn update_config(&mut self, enabled: bool, process_noise_p: f64, process_noise_v: f64, measurement_noise: f64) {
        self.enabled = enabled;
        self.process_noise_p = process_noise_p;
        self.process_noise_v = process_noise_v;
        self.measurement_noise = measurement_noise;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalize_angle_wraps() {
        let a = normalize_angle(4.0 * std::f64::consts::PI);
        assert!(a.abs() < 1e-9);
    }

    #[test]
    fn ekf_predict_update_runs() {
        let mut ekf = ExtendedKalmanFilter::new(
            SVector::<f64, 7>::zeros(),
            SMatrix::<f64, 7, 7>::identity(),
            SMatrix::<f64, 7, 7>::identity() * 1e-3,
            SMatrix::<f64, 3, 3>::identity() * 1e-3,
        );

        ekf.predict(0.016);
        let _ = ekf.update(&SVector::<f64, 3>::new(0.0, 0.0, 0.0));

        let (_x, _y, _theta, _vx, _vy, _omega, _, _, _, _) = ekf.filter_pose(0.1, -0.2, 0.3, 0.016);
    }

    #[test]
    fn tracker_returns_values() {
        let mut tracker = Tracker::new();
        let (_x, _y, _theta, vx, vy, omega) = tracker.track(0, 1, 1.0, 2.0, 0.5, 0.016);
        // Finite checks only
        assert!(vx.is_finite() && vy.is_finite() && omega.is_finite());
    }
}

