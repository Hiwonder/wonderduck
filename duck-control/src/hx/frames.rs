/// The HX status packet is `[FF FF id length error data... checksum]`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct StatusFrame {
    pub(super) id: u8,
    pub(super) error: u8,
    pub(super) data: Vec<u8>,
}

#[derive(Debug, Default)]
pub(super) struct Frames {
    buffer: Vec<u8>,
    rx_bytes: usize,
    checksum_errors: usize,
    invalid_lengths: usize,
    discarded_bytes: usize,
    valid_frames: usize,
    pub(super) duplicate_frames: usize,
    pub(super) unexpected_frames: usize,
    prefix: [u8; 32],
}

impl Frames {
    pub(super) fn evidence(&self, found: &[StatusFrame]) -> crate::read_diagnostic::WireEvidence {
        crate::read_diagnostic::WireEvidence {
            rx_bytes: self.rx_bytes,
            received_ids: found.iter().map(|frame| frame.id).collect(),
            valid_frames: self.valid_frames,
            checksum_errors: self.checksum_errors,
            invalid_lengths: self.invalid_lengths,
            discarded_bytes: self.discarded_bytes,
            duplicate_frames: self.duplicate_frames,
            unexpected_frames: self.unexpected_frames,
            pending_bytes: self.buffer.len(),
            rx_prefix: self.prefix,
            pending_prefix: self.buffer[..self.buffer.len().min(16)].to_vec(),
        }
    }

    pub(super) fn error_counts(&self) -> (usize, usize) {
        (self.checksum_errors, self.invalid_lengths)
    }

    pub(super) fn feed(&mut self, bytes: &[u8]) {
        let offset = self.rx_bytes.min(self.prefix.len());
        let copy = bytes.len().min(self.prefix.len() - offset);
        self.prefix[offset..offset + copy].copy_from_slice(&bytes[..copy]);
        self.rx_bytes += bytes.len();
        self.buffer.extend_from_slice(bytes);
    }

    pub(super) fn pop(&mut self) -> Option<StatusFrame> {
        loop {
            if self.buffer.len() < 4 {
                return None;
            }
            let start = self
                .buffer
                .windows(2)
                .position(|window| window == [0xff, 0xff]);
            let Some(start) = start else {
                let keep_last = self.buffer.last() == Some(&0xff);
                self.discarded_bytes += self.buffer.len() - usize::from(keep_last);
                self.buffer.clear();
                if keep_last {
                    self.buffer.push(0xff);
                }
                return None;
            };
            if start > 0 {
                self.discarded_bytes += start;
                self.buffer.drain(..start);
            }
            if self.buffer.len() < 4 {
                return None;
            }
            let id = self.buffer[2];
            let length = self.buffer[3];
            if id == 0xff || length < 2 {
                self.invalid_lengths += 1;
                self.discarded_bytes += 1;
                self.buffer.drain(..1);
                continue;
            }
            let total = usize::from(length) + 4;
            if self.buffer.len() < total {
                return None;
            }
            let frame = &self.buffer[..total];
            if frame[2..]
                .iter()
                .fold(0u8, |sum, byte| sum.wrapping_add(*byte))
                != 0xff
            {
                self.checksum_errors += 1;
                self.discarded_bytes += 1;
                self.buffer.drain(..1);
                continue;
            }
            let error = frame[4];
            let data = frame[5..total - 1].to_vec();
            self.buffer.drain(..total);
            self.valid_frames += 1;
            return Some(StatusFrame { id, error, data });
        }
    }
}
