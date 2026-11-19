//! A priority torque-off request with a completion for every caller.

use std::sync::Mutex;
use tokio::sync::oneshot;

type Completion = oneshot::Sender<Result<(), String>>;

#[derive(Default)]
struct Pending {
    requested: bool,
    waiters: Vec<Completion>,
}

#[derive(Default)]
pub(crate) struct EmergencyRelax {
    pending: Mutex<Pending>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CompletionState { Verified, Pending, Unverified }

impl EmergencyRelax {
    pub(crate) fn request(&self, reply: bool) -> Option<oneshot::Receiver<Result<(), String>>> {
        let mut pending = self.pending.lock().expect("emergency release mutex poisoned");
        pending.requested = true;
        if reply {
            let (sender, receiver) = oneshot::channel();
            pending.waiters.push(sender);
            Some(receiver)
        } else {
            None
        }
    }

    pub(crate) fn take(&self) -> Option<Vec<Completion>> {
        let mut pending = self.pending.lock().expect("emergency release mutex poisoned");
        if !pending.requested {
            return None;
        }
        pending.requested = false;
        Some(std::mem::take(&mut pending.waiters))
    }

    /// Complete the taken batch, distinguishing an in-flight retry from an
    /// already completed but unverified release.
    /// The caller holds the human-control lock so another request cannot race
    /// the decision to release the gate. Dropped waiters do not cancel a release.
    pub(crate) fn complete(&self, waiters: Vec<Completion>, result: Result<(), String>) -> CompletionState {
        let pending = self.pending.lock().expect("emergency release mutex poisoned");
        let verified = result.is_ok();
        for waiter in waiters {
            let _ = waiter.send(result.clone());
        }
        if pending.requested { CompletionState::Pending }
        else if verified { CompletionState::Verified }
        else { CompletionState::Unverified }
    }


}
