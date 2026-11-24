//! Host-side evidence for control transactions, without extra diagnostic traffic.
use super::*;

#[derive(Default)]
pub(super) struct DiagnosticCache {
    control_reads: u64,
    imu_missing_reads: u64,
    servo_missing_reads: u64,
    host_checksum_errors: u64,
    host_invalid_lengths: u64,
    last_control_missing: Vec<u8>,
    servo_alarms: [u8; NUM_JOINTS],
    voltage_minima: Option<[f64; NUM_JOINTS]>,
    voltage_maximum: Option<f64>,
    voltage_low_reads: [u32; NUM_JOINTS],
    voltage_valid_reads: [u32; NUM_JOINTS],
    voltage_last_report: Option<Instant>,
    voltage_min_reads: [u32; NUM_JOINTS],
}

impl DiagnosticCache {
    pub(super) fn servo_alarm(&self, joint: usize) -> u8 {
        self.servo_alarms[joint]
    }

    pub(super) fn observe_servo_status(&mut self, id: u8, status: u8) {
        let Some(joint) = crate::model::JOINT_IDS.iter().position(|&expected| expected == id) else {
            return;
        };
        if self.servo_alarms[joint] != status {
            self.servo_alarms[joint] = status;
            if status == 0 {
                tracing::info!(id, "servo alarm cleared");
            } else {
                tracing::warn!(id, status = format_args!("0x{status:02X}"),
                    alarm = %crate::read_diagnostic::servo_alarm_names(status),
                    "servo alarm reported; valid feedback retained");
            }
        }
    }

    pub(super) fn observe_voltages(&mut self, volts: [f64; NUM_JOINTS]) {
        let minima = self.voltage_minima.get_or_insert([0.0; NUM_JOINTS]);
        for (joint, (minimum, value)) in minima.iter_mut().zip(volts).enumerate() {
            if value.is_finite() && value > 0.0 {
                self.voltage_maximum = Some(self.voltage_maximum.map_or(value, |maximum| maximum.max(value)));
                if *minimum == 0.0 || value < *minimum {
                    *minimum = value;
                    self.voltage_min_reads[joint] = 1;
                } else if value == *minimum {
                    self.voltage_min_reads[joint] += 1;
                }
                self.voltage_valid_reads[joint] += 1;
                self.voltage_low_reads[joint] += u32::from(value < 7.0);
            }
        }
        let now = Instant::now();
        let elapsed = now.duration_since(*self.voltage_last_report.get_or_insert(now));
        if elapsed < Duration::from_secs(1) {
            return;
        }
        let readings: Vec<(u8, f64, u32, u32, u32)> = crate::model::JOINT_IDS.into_iter()
            .zip(*minima).enumerate()
            .filter(|(joint, _)| self.voltage_valid_reads[*joint] > 0)
            .map(|(joint, (id, value))| (id, value, self.voltage_min_reads[joint],
                self.voltage_low_reads[joint], self.voltage_valid_reads[joint])).collect();
        let min_volts = readings.iter().map(|entry| entry.1).reduce(f64::min);
        let max_volts = self.voltage_maximum;
        let servos = readings.iter().filter(|entry| Some(entry.1) == min_volts)
            .map(|(id, _, min_reads, _, _)| format!("id{id}：{min_reads}"))
            .collect::<Vec<_>>().join("，");
        let voltage_summary = match min_volts.zip(max_volts) {
            Some((minimum, maximum)) => format!("max:{maximum:.1}V，min:{minimum:.1}V[{servos}]"),
            None => "电压无有效数据".to_owned(),
        };
        let missing_ids: Vec<u8> = crate::model::JOINT_IDS.into_iter().enumerate()
            .filter(|(joint, _)| self.voltage_valid_reads[*joint] == 0)
            .map(|(_, id)| id).collect();
        static TEST_VOLTAGE_DEBUG: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        if *TEST_VOLTAGE_DEBUG.get_or_init(|| {
            std::env::var("ROBOT_VOLTAGE_LOG_DEBUG").as_deref() == Ok("1")
        }) {
            tracing::debug!(threshold_v = 7.0, window_ms = elapsed.as_millis() as u64,
                min_volts = ?min_volts, max_volts = ?max_volts, %voltage_summary, ?missing_ids,
                ids_min_volts_min_reads_low_reads_valid_reads = ?readings,
                "servo voltage window summary");
        } else if min_volts.is_some_and(|value| value < 7.0) {
            tracing::warn!(threshold_v = 7.0, window_ms = elapsed.as_millis() as u64,
                min_volts = ?min_volts, max_volts = ?max_volts, %voltage_summary, ?missing_ids,
                ids_min_volts_min_reads_low_reads_valid_reads = ?readings,
                "servo voltage window summary");
        } else if !missing_ids.is_empty() {
            tracing::info!(threshold_v = 7.0, window_ms = elapsed.as_millis() as u64,
                min_volts = ?min_volts, max_volts = ?max_volts, %voltage_summary, ?missing_ids,
                ids_min_volts_min_reads_low_reads_valid_reads = ?readings,
                "servo voltage window summary");
        }
        self.voltage_minima = None;
        self.voltage_maximum = None;
        self.voltage_min_reads = [0; NUM_JOINTS];
        self.voltage_low_reads = [0; NUM_JOINTS];
        self.voltage_valid_reads = [0; NUM_JOINTS];
        self.voltage_last_report = Some(now);
    }

    pub(super) fn fill_failure(&self, failure: &mut crate::read_diagnostic::ReadFailure) {
        failure.host_reads = self.control_reads;
        failure.host_imu_miss = self.imu_missing_reads;
        failure.host_servo_miss = self.servo_missing_reads;
        failure.host_checksum = self.host_checksum_errors;
        failure.host_bad_length = self.host_invalid_lengths;
    }

    pub(super) fn record_control_read(
        &mut self,
        tx: &[u8],
        found: &[StatusFrame],
        parser: &Frames,
    ) {
        if tx.len() < 9
            || !matches!(tx[4], SYNC_READ | UNOQ_FEEDBACK_READ)
            || !matches!(
                (tx[5], tx[6]),
                (PRESENT_POSITION, PRESENT_POSITION_LEN)
                    | (RECORD_FEEDBACK_ADDRESS, RECORD_FEEDBACK_LENGTH)
            )
        {
            return;
        }
        let ids = &tx[7..tx.len() - 1];
        self.last_control_missing = ids
            .iter()
            .copied()
            .filter(|id| !found.iter().any(|frame| frame.id == *id))
            .collect();
        self.control_reads = self.control_reads.saturating_add(1);
        self.imu_missing_reads = self.imu_missing_reads
            .saturating_add(u64::from(self.last_control_missing.iter().any(|id| matches!(*id, 200 | 201))));
        self.servo_missing_reads = self
            .servo_missing_reads
            .saturating_add(u64::from(self.last_control_missing.iter().any(|id| !matches!(*id, 200 | 201))));
        let (checksum_errors, invalid_lengths) = parser.error_counts();
        self.host_checksum_errors = self
            .host_checksum_errors
            .saturating_add(checksum_errors as u64);
        self.host_invalid_lengths = self
            .host_invalid_lengths
            .saturating_add(invalid_lengths as u64);
    }

    pub(super) fn summary(&self) -> String {
        format!(
            "host_reads={} host_imu_miss={} host_servo_miss={} host_checksum={} host_bad_length={}",
            self.control_reads,
            self.imu_missing_reads,
            self.servo_missing_reads,
            self.host_checksum_errors,
            self.host_invalid_lengths
        )
    }
}



/// Same coefficient contract as the MCU maintenance interface and calibration tool.
pub(super) fn decode_accel_calibration(block: &[u8]) -> Result<(u16, u32, [f32; 3], [f32; 3])> {
    if block.len() != 32 {
        return Err(IoError::Bus("MCU accel calibration response must be 32 bytes".into()));
    }
    let schema = u16::from_le_bytes(block[0..2].try_into().unwrap());
    let flags = u16::from_le_bytes(block[2..4].try_into().unwrap());
    let generation = u32::from_le_bytes(block[4..8].try_into().unwrap());
    let values: [f32; 6] = std::array::from_fn(|i|
        f32::from_le_bytes(block[8 + i * 4..12 + i * 4].try_into().unwrap()));
    let bias_g = [values[0], values[1], values[2]];
    let scale = [values[3], values[4], values[5]];
    if schema != 2 || flags > 1 || values.iter().any(|v| !v.is_finite())
        || bias_g.iter().any(|v| v.abs() > 0.5)
        || scale.iter().any(|v| !(0.5..=1.5).contains(v))
        || (flags != 1 && values != [0., 0., 0., 1., 1., 1.])
    {
        return Err(IoError::Bus("invalid MCU RAM accel calibration protocol or parameters".into()));
    }
    Ok((flags, generation, bias_g, scale))
}
