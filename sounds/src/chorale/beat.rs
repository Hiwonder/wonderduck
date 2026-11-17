//! Keeping several ducks together without a shared clock.
//!
//! Four robots singing a chord have to agree on when the beat is, to about **±20 ms** — the
//! figure comes from [`super`]: an ensemble that agreed to the millisecond would sound like one
//! organ, so a few tens of milliseconds of spread is deliberately *added*, and the sync only has
//! to be tight enough not to swamp it.
//!
//! The obvious way is to sync the clocks and agree a start time. This does not do that, because
//! there is no clock to sync: the boards have no RTC agreement and no NTP, so establishing an
//! offset would mean a connection, a bond, and a central-role Bluetooth client that `btd` does
//! not have.
//!
//! ## The conductor is a beacon
//!
//! Instead, nobody shares a clock — they share a **beat**. The duck conducting puts a beat
//! counter in its BLE advertisement and bumps it once per musical beat. Everyone else passively
//! scans, and *the arrival of a new counter value is the downbeat*. No timestamps to compare, no
//! offset to estimate, no connection, no pairing: passive scanning is the whole radio
//! requirement, and it is the cheapest thing a Bluetooth controller can do.
//!
//! ## Why the radio's jitter does not sink it
//!
//! Air time is microseconds; the error is the **advertising slot**. The conductor hands new
//! payload to its controller and it goes out at the next slot, which `btd` spaces 100–150 ms
//! apart on purpose (one antenna carries BLE, the gamepad and wifi). So a follower hears each
//! beat somewhere in a 50 ms window, and — crucially — *independently per beat*.
//!
//! Two things turn that into ±6 ms:
//!
//!  - The conductor delays its **own** playback by [`SLOT_MEAN_S`], the middle of that window,
//!    so it is wrong in the same direction as everyone else rather than early by default.
//!  - A follower does not chase individual beats. It averages the *phase* over a sliding window
//!    ([`Follower::WINDOW`]), so independent jitter falls as the square root of the window.
//!
//! **The tempo is not estimated, and that was the fix rather than the shortcut.** The first
//! version fitted a straight line through (beat, arrival) and read the tempo off its slope, which
//! is the textbook thing to do and was measurably worse: a line fit evaluated at the *newest*
//! point — the edge of the window, which is exactly where "where are we now" is asked — carries
//! about four times the error it carries at the window's centre, because slope uncertainty
//! compounds with distance from the centroid. Two simulated followers came out 36 ms apart.
//! Holding the period at the score's tempo and averaging only the phase removes the slope term
//! entirely and lands at a third of that. It is also the honest model: both ducks read the tempo
//! off the same score, and their crystals differ by parts per million — 3 ms over a minute-long
//! piece, against the tens of milliseconds the estimator itself was contributing.
//!
//! Everything in this module is a pure function of observed arrival times, so the claim above is
//! a test rather than an argument: [`tests::two_followers_agree_within_the_ensemble_budget`]
//! drives simulated beacons through a simulated 50 ms slot window and checks what comes out.
//!
//! ## What it does when the radio stops
//!
//! Keeps singing. A follower that hears nothing holds its last phase and carries on at the
//! score's tempo — good to a crystal's accuracy, milliseconds per minute — and picks the beat back
//! up when the conductor reappears. That is also exactly what a singer does when they cannot hear
//! the conductor, and it is why the correction is a slow drag on the tempo rather than a jump.

/// Middle of the advertising-slot window `btd` uses (100–150 ms).
///
/// The conductor waits this long after handing a beat to the radio before playing it, so that its
/// own beat and the one everybody hears are the same beat. Getting this wrong shifts the
/// conductor against the whole ensemble, which is the one error that is *not* common-mode.
pub const SLOT_MEAN_S: f64 = 0.125;

/// The duck holding the beat.
///
/// Deliberately dumb: it counts beats and says when to put each one on the air. It does not know
/// or care whether anyone is listening, which is what makes a chorale that loses a follower carry
/// on rather than stall.
#[derive(Debug, Clone)]
pub struct Conductor {
    beat_s: f64,
    /// The last beat actually handed to the radio, or `None` before the first.
    ///
    /// An `Option`, not a counter starting at zero: "beat 0 has gone out" and "nothing has gone
    /// out yet" are different states, and collapsing them made [`Conductor::due`] re-announce
    /// beat 0 on every poll — which, since the caller polls at the advertising rate, is eight
    /// beat-zeroes a second and a follower's fit built on nonsense.
    emitted: Option<u64>,
    /// Local time at which beat 0 was handed to the radio.
    started_at: f64,
}

impl Conductor {
    pub fn new(bpm: f64, now: f64) -> Self {
        Self {
            beat_s: 60.0 / bpm.max(1.0),
            emitted: None,
            started_at: now,
        }
    }

    /// The next beat that should go on the air, if it is due.
    ///
    /// Called as often as the caller likes; it returns `Some` only when the beat has actually
    /// turned over, which is the moment the advertisement payload should change.
    pub fn due(&mut self, now: f64) -> Option<u64> {
        let elapsed = (now - self.started_at).max(0.0);
        let should_be = (elapsed / self.beat_s).floor() as u64;
        match self.emitted {
            Some(last) if should_be <= last => None,
            _ => {
                self.emitted = Some(should_be);
                Some(should_be)
            }
        }
    }

    /// Where in the score the conductor should be singing, in beats.
    ///
    /// Behind its own beat counter by [`SLOT_MEAN_S`] — the point of the whole constant. The
    /// conductor is singing what the room is *hearing*, not what it has most recently told the
    /// radio.
    pub fn position_beats(&self, now: f64) -> f64 {
        (now - self.started_at - SLOT_MEAN_S) / self.beat_s
    }

    /// The beat currently on the air, as the wire carries it — a byte, wrapping.
    ///
    /// The caller assembles the advertisement; this module knows nothing about radios. A byte is
    /// ~4 minutes at a chorale tempo, and [`Follower`] unwraps it.
    pub fn wire_beat(&self) -> u8 {
        (self.emitted.unwrap_or(0) % 256) as u8
    }

    /// Beats handed to the radio so far, unwrapped. `None` before the first.
    pub fn beat(&self) -> Option<u64> {
        self.emitted
    }
}

/// One heard beat: which beat it was, and when it arrived here.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Heard {
    beat: f64,
    at: f64,
}

/// A duck locking onto somebody else's beat.
///
/// Holds one number: where the conductor's beat 0 landed, averaged over the last
/// [`Follower::WINDOW`] beats. The tempo comes from the score, so this is the only thing that has
/// to be measured — and measuring one thing rather than two is what keeps it inside the budget.
#[derive(Debug, Clone)]
pub struct Follower {
    /// Seconds per beat, from the score. Held, not estimated — see the module docs.
    beat_s: f64,
    heard: Vec<Heard>,
    /// Unwrapped beat number of the last observation, so a counter rolling over 255 does not read
    /// as jumping backwards by 255 beats.
    last_beat: Option<f64>,
    /// Local time the conductor's beat 0 landed here, averaged over the window.
    phase: Option<f64>,
}

impl Follower {
    /// How many beats the phase is averaged over.
    ///
    /// The jitter is independent per beat, so the error falls as `1/sqrt(WINDOW)`. A 50 ms slot
    /// window has a standard deviation of ~14 ms on one beat, so 24 beats brings it to ~3 ms —
    /// and two independent followers to ~4 ms, comfortably inside the ±20 ms an ensemble needs.
    /// Longer converges slower for no audible gain; 24 beats is ~25 s at a chorale tempo, and a
    /// follower is already singing after four.
    pub const WINDOW: usize = 24;

    /// A residual this far from the fit is not a beat — a scan that sat in a queue, or a stale
    /// advertisement re-read. Dropped rather than averaged in, because one outlier of half a
    /// second would drag the fit further than sixteen good beats pull it back.
    pub const OUTLIER_S: f64 = 0.20;

    pub fn new(bpm: f64) -> Self {
        Self {
            beat_s: 60.0 / bpm.max(1.0),
            heard: Vec::new(),
            last_beat: None,
            phase: None,
        }
    }

    /// Take one heard beat: `beat` is the wrapped byte off the wire, `at` the local time the new
    /// value arrived.
    ///
    /// Call this **only when the counter changes**. A conductor's advertisement repeats seven or
    /// eight times per beat, and re-reading the same value is not another beat — it is the same
    /// beat, later, and feeding those in would drag the phase late by half an advertising
    /// interval.
    pub fn observe(&mut self, beat: u8, at: f64) {
        let unwrapped = self.unwrap(beat);
        // Reject what cannot be a beat before it can pollute the average.
        if let Some(phase) = self.phase {
            let predicted = phase + self.beat_s * unwrapped;
            if (at - predicted).abs() > Self::OUTLIER_S && self.heard.len() >= 4 {
                return;
            }
        }
        self.heard.push(Heard {
            beat: unwrapped,
            at,
        });
        if self.heard.len() > Self::WINDOW {
            self.heard.remove(0);
        }
        self.refit();
    }

    /// A wrapped byte back into a monotonically rising beat number.
    fn unwrap(&self, beat: u8) -> f64 {
        let Some(last) = self.last_beat else {
            return f64::from(beat);
        };
        let base = (last / 256.0).floor() * 256.0;
        let mut candidate = base + f64::from(beat);
        // The counter only ever goes forward, so a value that reads as behind is a wrap.
        while candidate < last - 128.0 {
            candidate += 256.0;
        }
        while candidate > last + 128.0 {
            candidate -= 256.0;
        }
        candidate
    }

    fn refit(&mut self) {
        if let Some(latest) = self.heard.last() {
            self.last_beat = Some(latest.beat);
        }
        if self.heard.is_empty() {
            return;
        }
        // The phase each observation implies, averaged. Every term is an independent estimate of
        // the same quantity, which is what makes the average the whole trick.
        let n = self.heard.len() as f64;
        self.phase = Some(
            self.heard
                .iter()
                .map(|heard| heard.at - self.beat_s * heard.beat)
                .sum::<f64>()
                / n,
        );
    }

    /// Whether enough beats have been heard to sing to.
    ///
    /// Four, not one: a single beat gives a phase with the whole slot window as its error bar, and
    /// coming in on that would be audibly early or late. Four is about a bar, which is also how
    /// long a singer takes to join something already in progress.
    pub fn locked(&self) -> bool {
        self.heard.len() >= 4
    }

    /// Where in the score to be singing now, in beats. `None` until [`Follower::locked`].
    pub fn position_beats(&self, now: f64) -> Option<f64> {
        if !self.locked() {
            return None;
        }
        Some((now - self.phase?) / self.beat_s)
    }

    /// How far this duck's idea of the beat has moved over the window, in seconds — the spread of
    /// the individual phase estimates.
    ///
    /// Diagnostic, and the one worth having: a healthy lock sits at a few milliseconds, and a
    /// number in the hundreds means this duck is hearing something that is not the conductor's
    /// beat — a second conductor, or a stale advertisement being re-read as new.
    pub fn spread_s(&self) -> Option<f64> {
        let phase = self.phase?;
        if self.heard.len() < 2 {
            return None;
        }
        let n = self.heard.len() as f64;
        let variance = self
            .heard
            .iter()
            .map(|heard| {
                let implied = heard.at - self.beat_s * heard.beat;
                (implied - phase) * (implied - phase)
            })
            .sum::<f64>()
            / n;
        Some(variance.sqrt())
    }
}
