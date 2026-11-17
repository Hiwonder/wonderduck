//! Read-only firmware counters, separate from policy observations and motor commands.

use std::time::Duration;

use crate::io::{IoError, Result};
use crate::model::JOINT_IDS;

pub const ADDRESS: u8 = 0x80;
pub const LENGTH: u8 = 12;
// The preceding 53-byte goal write only queues its bytes. Allow its remaining
// 0.53ms wire time at 1Mbps plus servo servicing, as well as the three replies.
pub const READ_BUDGET: Duration = Duration::from_millis(4);
pub const FINISH_RESERVE: Duration = Duration::from_millis(2);
pub const MAX_IDS_PER_READ: usize = 3;

pub fn validate_ids(ids: &[u8]) -> Result<()> {
    if ids
        .iter()
        .enumerate()
        .any(|(i, id)| !JOINT_IDS.contains(id) || ids[..i].contains(id))
    {
        return Err(IoError::Bus(
            "servo diagnostics require distinct configured servo IDs".into(),
        ));
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub struct Counters {
    pub isr_max_us: u32,
    pub isr_overrun_count: u32,
    pub sensor_read_fail_count: u32,
}

impl Counters {
    pub fn decode(raw: &[u8]) -> Result<Self> {
        if raw.len() != usize::from(LENGTH) {
            return Err(IoError::ShortRead {
                what: "servo runtime diagnostics",
                expected: 12,
                got: raw.len(),
            });
        }
        let word = |start| u32::from_le_bytes(raw[start..start + 4].try_into().unwrap());
        Ok(Self {
            isr_max_us: word(0),
            isr_overrun_count: word(4),
            sensor_read_fail_count: word(8),
        })
    }
}

#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct ServoSample {
    pub id: u8,
    pub raw: [u8; 12],
    pub counters: Counters,
    pub previous_read_start_t_ns: Option<u64>,
    pub previous_read_end_t_ns: Option<u64>,
    pub isr_overrun_delta: Option<u32>,
    pub sensor_read_fail_delta: Option<u32>,
    pub counter_reset_or_discontinuity: bool,
    pub counter_saturated: bool,
}

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // Decoder retained for offline diagnostics when bus polling is resumed.
pub(crate) struct Previous {
    pub counters: Counters,
    pub read_start_t_ns: u64,
    pub read_end_t_ns: u64,
}

impl ServoSample {
    #[allow(dead_code)]
    pub(crate) fn decode(id: u8, raw: &[u8], previous: Option<Previous>) -> Result<Self> {
        let counters = Counters::decode(raw)?;
        let reset = previous.is_some_and(|p| {
            counters.isr_max_us < p.counters.isr_max_us
                || counters.isr_overrun_count < p.counters.isr_overrun_count
                || counters.sensor_read_fail_count < p.counters.sensor_read_fail_count
        });
        // Saturation hides further events; a zero difference would incorrectly claim none.
        let delta = |current: u32, before: u32| {
            if reset || current == u32::MAX || before == u32::MAX {
                None
            } else {
                current.checked_sub(before)
            }
        };
        Ok(Self {
            id,
            raw: raw.try_into().unwrap(),
            counters,
            previous_read_start_t_ns: previous.map(|p| p.read_start_t_ns),
            previous_read_end_t_ns: previous.map(|p| p.read_end_t_ns),
            isr_overrun_delta: previous
                .and_then(|p| delta(counters.isr_overrun_count, p.counters.isr_overrun_count)),
            sensor_read_fail_delta: previous.and_then(|p| {
                delta(
                    counters.sensor_read_fail_count,
                    p.counters.sensor_read_fail_count,
                )
            }),
            counter_reset_or_discontinuity: reset,
            counter_saturated: counters.isr_overrun_count == u32::MAX
                || counters.sensor_read_fail_count == u32::MAX,
        })
    }
}

/// Read bounds bracket host acquisition, not the time of an individual MCU failure.
#[derive(Debug, serde::Serialize)]
pub struct Capture {
    pub status: &'static str,
    pub ids: Vec<u8>,
    pub read_start_t_ns: Option<u64>,
    pub read_end_t_ns: Option<u64>,
    pub available_budget_us: u64,
    pub elapsed_us: u64,
    pub over_budget: bool,
    pub servos: Vec<ServoSample>,
    pub error: Option<String>,
}

impl Capture {
    pub(crate) fn pending(budget: Duration) -> Self {
        Self {
            status: "skipped_budget",
            ids: Vec::new(),
            read_start_t_ns: None,
            read_end_t_ns: None,
            available_budget_us: budget.as_micros() as u64,
            elapsed_us: 0,
            over_budget: false,
            servos: Vec::new(),
            error: None,
        }
    }
}
