//! Optional servo diagnostics must not create shared-bus traffic during control.

use super::*;
use crate::servo_diagnostics::{self as runtime, Capture};

impl HxIo {
    /// The recording broadcast without startup configuration writes or motor commands.
    pub fn diagnostic_read_feedback(&mut self) -> Result<ServoFeedback> {
        let mut feedback = ServoFeedback {
            read_start_t_ns: duck_ipc_proto::clock::monotonic_ns(),
            ..ServoFeedback::default()
        };
        let blocks = self.read_control_blocks(RECORD_FEEDBACK_ADDRESS, RECORD_FEEDBACK_LENGTH)?;
        feedback.read_end_t_ns = duck_ipc_proto::clock::monotonic_ns();
        for (joint, block) in blocks.iter().take(NUM_JOINTS).enumerate() {
            decode_record_feedback(&mut feedback, joint, block)?;
        }
        Ok(feedback)
    }

    pub fn set_servo_diagnostics_ids(&mut self, ids: Vec<u8>) -> Result<()> {
        runtime::validate_ids(&ids)?;
        self.servo_diagnostics_ids = ids;
        if !self.servo_diagnostics_ids.is_empty() {
            tracing::warn!("additional servo diagnostics unavailable during control; primary feedback remains recorded");
        }
        Ok(())
    }

    pub(super) fn capture_servo_diagnostics(&mut self, budget: Duration) -> Option<Capture> {
        if self.servo_diagnostics_ids.is_empty() { return None; }
        let mut capture = Capture::pending(budget);
        capture.status = "unavailable";
        capture.ids = self.servo_diagnostics_ids.clone();
        capture.error = Some("additional servo diagnostics unavailable during active shared-bus control".into());
        Some(capture)
    }
}
