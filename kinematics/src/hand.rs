//! The nearest thing in front of the beak, as an instrument wants it.
//!
//! This started out clever. It carried a background captured when the theremin armed, so
//! that a duck facing a wall could still tell a hand from the wall; a plane fit to exempt
//! walls from being mistaken for one enormous hand; a slow drift so the room could be
//! rearranged. All of it worked in tests and none of it survived a duck, for one reason
//! worth writing down: **the sensor's input is not stable enough to reason that hard
//! about.** Which zones carry a usable status varies frame to frame, so a background
//! captured over half a second was a different background every time, and the *same*
//! gesture would arm one moment and be refused the next. Cleverness on top of a noisy input
//! multiplies the noise; it does not filter it.
//!
//! So this is the simple thing, and the theremin is an explicit mode instead. There is no
//! background and no arming: while the instrument is up, the nearest return inside the
//! playable band is the hand. Point the duck at open space and it is silent; point it at a
//! wall 40 cm away and it plays a steady note, which is correct — you turned the mode on,
//! and a mode turned on deliberately is allowed to do the obvious thing.
//!
//! What is left is the two pieces of robustness that are about the *sensor* rather than
//! about the room:
//!
//!   - **Which status bytes count.** ST calls 5 and 9 "valid", and on this sensor at 15 Hz a
//!     hand past ~30 cm routinely comes back as 4 or 13 — *consistency failed*, sigma too
//!     high — carrying a distance perfectly good enough for a pitch. Accepting only 5 and 9
//!     is why the first version died at 30 cm. The set is [`Config::statuses`], and it is
//!     config rather than a constant because it is the one number a bench session needs to
//!     move.
//!   - **A hold.** A zone that flickers between usable and not, at 15 Hz, chops a note into
//!     gravel. [`Config::hold`] keeps the last hand for a few frames, so a dropout is
//!     inaudible while a hand actually leaving still stops the note promptly.
//!
//! There is no floor filter and no reprojection here, on purpose. Both need the head's
//! forward kinematics and the IMU, and both were another way for a note to disappear for a
//! reason the player cannot see. An instrument that plays the raw beam is one you can
//! predict.

use std::time::{Duration, Instant};

use crate::tof::{COLS, ROWS};

const N_ZONES: usize = ROWS * COLS;

/// What counts as a hand.
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    /// Nearest playable range, metres. Below ~10 cm the sensor's own cover-glass crosstalk
    /// invents short returns, so this is a floor on believability rather than on taste.
    pub near_m: f64,
    /// Farthest playable range, metres.
    pub far_m: f64,
    /// How many zones in the band make a hand. Low: a hand at the far end of the band is a
    /// handful of zones, and this sensor will only be giving usable status for some of them.
    pub min_zones: usize,
    /// ST status bytes whose distance is believed. See the module docs — accepting only the
    /// two ST calls "valid" is what made the first version stop working at 30 cm.
    pub statuses: Vec<u8>,
    /// How long the last hand is held through a dropout. Long enough to bridge the sensor's
    /// flicker, short enough that a hand actually withdrawn stops the note.
    pub hold: Duration,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            near_m: 0.10,
            far_m: 0.70,
            min_zones: 2,
            // 5 and 9 are ST's "valid" and "valid, large pulse". 6 is a first range whose
            // wrap-around check was not performed, 10 a valid range where the previous one
            // saw nothing, 12 a target blurred by a sharp edge — which is what the edge of a
            // hand *is*. 4 and 13 are the consistency failures a moving hand produces past
            // 30 cm, and they are in because a pitch does not need a millimetre.
            statuses: vec![4, 5, 6, 9, 10, 12, 13],
            hold: Duration::from_millis(250),
        }
    }
}

impl Config {
    /// Whether this status byte's distance is worth believing.
    pub fn believes(&self, status: u8) -> bool {
        self.statuses.contains(&status)
    }
}

/// A hand, as the theremin needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hand {
    /// Robust distance to it, metres — a low percentile of what is in the band rather than
    /// the single nearest zone, which on this sensor is regularly a flier.
    pub range_m: f64,
    /// Where that sits in the playable band: 0 at [`Config::far_m`], 1 at
    /// [`Config::near_m`]. Closer is *higher*, which is the direction the pitch and the
    /// mouth both move.
    pub closeness: f64,
    /// How many zones it covers. Reported because it is the number that says whether a
    /// dropout was the hand leaving or the sensor blinking.
    pub zones: usize,
    /// True when this hand is the held memory of an earlier frame rather than one measured
    /// now — so a readout can show a bridged dropout instead of pretending it saw something.
    pub held: bool,
}

/// Finds the hand, and remembers it briefly.
///
/// Stateful only for the hold: everything else about a frame's verdict depends on that frame
/// alone, which is what makes the instrument predictable.
#[derive(Debug)]
pub struct Tracker {
    config: Config,
    last: Option<(Hand, Instant)>,
}

impl Tracker {
    pub fn new(config: Config) -> Self {
        Self { config, last: None }
    }

    pub fn config(&self) -> &Config {
        &self.config
    }

    /// The hand in this frame, or the one from a frame just gone.
    ///
    /// `distance_mm` and `status` are the wire's own arrays — raw, parallel, row-major, and
    /// possibly shorter than the grid if the peer is from another release. Taken raw rather
    /// than pre-interpreted on purpose: which statuses count is this module's decision, and
    /// it is the decision that matters most.
    pub fn track(&mut self, distance_mm: &[i16], status: &[u8], now: Instant) -> Option<Hand> {
        let mut in_band: Vec<f64> = Vec::new();
        for zone in 0..N_ZONES {
            let (Some(&mm), Some(&code)) = (distance_mm.get(zone), status.get(zone)) else {
                continue;
            };
            // A negative distance comes back on a failed convergence whatever the status
            // says, and is not a measurement.
            if mm <= 0 || !self.config.believes(code) {
                continue;
            }
            let range = f64::from(mm) / 1000.0;
            if (self.config.near_m..=self.config.far_m).contains(&range) {
                in_band.push(range);
            }
        }

        // The band must hold at least one zone whatever `min_zones` says: at 0 the length
        // check passes on an empty band, and the percentile index below would panic on it.
        if in_band.len() < self.config.min_zones.max(1) {
            // Hold the last hand briefly. This is the whole anti-chop mechanism: the note
            // rides over a dropout, and stops when one lasts.
            return match self.last {
                Some((hand, at)) if now.duration_since(at) < self.config.hold => {
                    Some(Hand { held: true, ..hand })
                }
                _ => {
                    self.last = None;
                    None
                }
            };
        }

        in_band.sort_by(|a, b| a.partial_cmp(b).expect("ranges are finite"));
        // A low percentile, not the minimum: single-zone fliers a few centimetres short of
        // the truth are routine here, and as the pitch input one would be a chirp.
        let range_m = in_band[in_band.len() / 5];
        let span = (self.config.far_m - self.config.near_m).max(1e-6);
        let hand = Hand {
            range_m,
            closeness: ((self.config.far_m - range_m) / span).clamp(0.0, 1.0),
            zones: in_band.len(),
            held: false,
        };
        self.last = Some((hand, now));
        Some(hand)
    }

    /// Forget the held hand — for putting the instrument down, so picking it back up does
    /// not open with a note from before.
    pub fn reset(&mut self) {
        self.last = None;
    }
}

/// How many zones came back with each status byte, most common first.
///
/// Purely diagnostic, and the diagnostic that matters: "it stops working past 30 cm" and
/// "status 4 covers 31 zones of the frame" are the same sentence, but only the second tells
/// you what to change. Rendered into the live readout so a bench session never has to guess
/// what the sensor is actually saying.
pub fn status_histogram(status: &[u8]) -> Vec<(u8, usize)> {
    let mut counts: Vec<(u8, usize)> = Vec::new();
    for &code in status.iter().take(N_ZONES) {
        match counts.iter_mut().find(|(c, _)| *c == code) {
            Some((_, n)) => *n += 1,
            None => counts.push((code, 1)),
        }
    }
    counts.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    counts
}
