//! Owned wire evidence, formatted only when a diagnostic consumer needs text.

use std::fmt;

pub fn servo_alarm_names(status: u8) -> String {
    [(1, "voltage"), (2, "sensor"), (4, "temperature"), (8, "current"),
        (16, "angle"), (32, "overload"), (192, "unknown")].into_iter()
        .filter_map(|(bit, name)| (status & bit != 0).then_some(name))
        .collect::<Vec<_>>().join(",")
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReadFailureReason {
    Timeout,
    Response(String),
    Status { id: u8, status: u8, payload_len: usize, expected_len: u8 },
}

#[derive(Debug, Clone, Default)]
pub struct WireEvidence {
    pub rx_bytes: usize,
    pub received_ids: Vec<u8>,
    pub valid_frames: usize,
    pub checksum_errors: usize,
    pub invalid_lengths: usize,
    pub discarded_bytes: usize,
    pub duplicate_frames: usize,
    pub unexpected_frames: usize,
    pub pending_bytes: usize,
    pub rx_prefix: [u8; 32],
    pub pending_prefix: Vec<u8>,
}

#[derive(Debug, Clone)]
pub struct ReadFailure {
    pub reason: ReadFailureReason,
    pub missing_ids: Vec<u8>,
    pub tx: Vec<u8>,
    pub elapsed_us: u64,
    pub wire: WireEvidence,
    pub host_reads: u64,
    pub host_imu_miss: u64,
    pub host_servo_miss: u64,
    pub host_checksum: u64,
    pub host_bad_length: u64,
}

impl ReadFailure {
    pub fn write_reason(&self, writer: &mut impl fmt::Write) -> fmt::Result {
        match &self.reason {
            ReadFailureReason::Timeout => write!(writer, "HX {} timeout; missing IDs {:?}",
                if matches!(self.tx[4], 0x82 | 0x93) { "sync_read" } else { "read" }, self.missing_ids),
            ReadFailureReason::Response(error) => write!(writer, "HX read response: {error}"),
            ReadFailureReason::Status { id, status, payload_len, expected_len } => {
                write!(writer, "HX ID {id} status=0x{status:02X}")?;
                // A valid status packet with an alarm is not a serial timeout or power loss.
                // These bits belong to servos; the MCU has a separate diagnostic contract.
                if *id != 200 && *status != 0 {
                    write!(writer, " servo_alarm={}", servo_alarm_names(*status))?;
                }
                write!(writer, " payload_len={payload_len} expected_len={expected_len}")
            }
        }
    }

    pub fn wire_prefix(&self) -> String {
        format!("rx_prefix={:02x?} pending_prefix={:02x?}",
            &self.wire.rx_prefix[..self.wire.rx_bytes.min(32)], self.wire.pending_prefix)
    }

    pub fn signature(&self) -> String {
        let mut reason = String::new();
        self.write_reason(&mut reason).expect("formatting a string");
        format!("{}:{}:{reason}", self.tx[4], self.tx[5])
    }
}

impl fmt::Display for ReadFailure {
    fn fmt(&self, writer: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.write_reason(writer)?;
        let wire = &self.wire;
        write!(writer,
            "; rx_bytes={} received_ids={:?} valid_frames={} checksum_errors={} invalid_lengths={} discarded_bytes={} duplicate_frames={} unexpected_frames={} pending_bytes={}; elapsed_us={} tx_len={} instruction=0x{:02x} address={}; host_reads={} host_imu_miss={} host_servo_miss={} host_checksum={} host_bad_length={}",
            wire.rx_bytes, wire.received_ids, wire.valid_frames, wire.checksum_errors,
            wire.invalid_lengths, wire.discarded_bytes, wire.duplicate_frames,
            wire.unexpected_frames, wire.pending_bytes, self.elapsed_us, self.tx.len(),
            self.tx[4], self.tx[5], self.host_reads, self.host_imu_miss, self.host_servo_miss,
            self.host_checksum, self.host_bad_length)
    }
}

impl std::error::Error for ReadFailure {}
