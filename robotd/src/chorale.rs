//! Several ducks singing one piece: who conducts, who sings what, and where in the score we are.
//!
//! The behaviour half of the duck chorale. `btd` owns the radio and does no thinking; this owns the
//! thinking and touches no radio. What passes between them is `chorale.*`: a beacon to advertise
//! going down, and beacons heard coming up.
//!
//! ## Nobody is in charge until somebody is
//!
//! A duck asked for a chorale starts *listening*: it advertises an idle beacon saying it is
//! willing, and watches for others. When two willing ducks see each other the lower id conducts —
//! deterministic, so there is no election to go wrong and no message to lose. A duck that hears a
//! beacon already carrying a piece does not argue about it; it joins.
//!
//! ## The conductor owns the seating, and that is not a convenience
//!
//! Seating depends on join order ([`sounds::chorale::seat`]), so a duck seating *itself* from
//! whatever it happened to hear will disagree with a duck that heard a different subset — and both
//! will sing alto. So the conductor keeps the roster, broadcasts it, and everyone replays
//! [`sounds::chorale::seat_all`] over it. One source of truth, which is what a conductor is.
//!
//! A duck that cannot find itself in the roster is not in the piece yet: it keeps listening, and
//! the conductor adds it on the next beat. That is why joining is free rather than negotiated.
//!
//! ## Where the score position comes from
//!
//! Not a start time. There is no clock to agree on, so the conductor's beat counter *is* the
//! timebase: [`sounds::chorale::beat::Conductor`] for the duck holding it and `Follower` for
//! everyone else, and both answer the same question — how many beats into the piece are we, right
//! now. The audio side reads that and renders the score at it, so a duck whose audio stalls
//! resumes in the right place rather than a bar behind.

use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use sounds::chorale::beat::{Conductor, Follower};
use sounds::chorale::{Part, Score, seat_all};

/// How long a heard beacon counts for.
///
/// A duck that has walked out of range stops being in the piece — otherwise a chorale would hold a
/// seat for a duck that left, and the roster it broadcasts would name someone who is not singing.
/// Generous against a missed advertisement: at the beacon's interval this is dozens of chances.
const PEER_STALE: Duration = Duration::from_secs(3);

/// How long to listen before starting a piece alone.
///
/// A duck that has heard nobody does not sing: a solo chorale is a duck quacking to itself. This is
/// only the settling time before *two* ducks that can both see each other agree who conducts —
/// long enough that they have both certainly heard the other, so they cannot both decide they are
/// alone.
const SETTLE: Duration = Duration::from_millis(1500);

/// The pieces a duck can sing, by the id the beacon carries.
///
/// The registry is what makes the beacon's `piece` byte mean something: the conductor picks an
/// id, followers load the same score by it, and a duck that does not know an id **keeps
/// listening rather than joining** — it cannot sing a piece it does not have, and guessing one
/// is how two ducks end up performing different songs at each other. That is also the right
/// degradation for a mixed-version flock: an old duck near new ones stays politely quiet.
const PIECE_WISTFUL: u8 = 1;
const PIECE_DUCK_STRUT: u8 = 2;

/// Every piece a conductor may pick from.
const PIECES: [u8; 2] = [PIECE_WISTFUL, PIECE_DUCK_STRUT];

/// Whether this build can sing a piece — what `robot.chorale` validates a `--piece` pin
/// against at the door.
pub fn known_piece(id: u8) -> bool {
    piece(id).is_some()
}

/// The catalogue, for the refusal that names what *is* on this robot.
pub fn piece_catalogue() -> String {
    PIECES
        .iter()
        .filter_map(|id| piece(*id).map(|score| format!("{id} ({})", score.name)))
        .collect::<Vec<_>>()
        .join(", ")
}

/// The score for a piece id, or `None` for one this build does not know.
fn piece(id: u8) -> Option<Score> {
    match id {
        PIECE_WISTFUL => Some(Score::wistful()),
        PIECE_DUCK_STRUT => Some(Score::duck_strut()),
        _ => None,
    }
}

/// How often an *idle* beacon changes, so that it is noticed at all.
///
/// A payload that never changes is an advertisement that is never re-registered, and on this
/// stack a duck is reported to a scanner mainly when it turns up at a new address — which,
/// with BLE privacy on, happens on the radio's schedule and not on ours. Two willing ducks in
/// a room took tens of seconds to find each other. So the idle beacon carries a slow counter
/// purely to make itself change; slow, because each change costs an advertisement
/// re-registration and a fresh random address.
const IDLE_HEARTBEAT: Duration = Duration::from_millis(1500);

/// Another duck, as last heard.
#[derive(Debug, Clone)]
struct Peer {
    beacon: proto::ChoraleBeacon,
    /// Local time the beacon was heard — `btd` reports an age, and this is that age subtracted
    /// from the clock on arrival. The two daemons share a machine but not an epoch.
    at: Instant,
}

/// What this duck is doing.
enum State {
    /// Not asked for, or not allowed. Nothing on the air.
    Off,
    /// Willing: an idle beacon out, and listening.
    Listening { since: Instant },
    /// Holding the beat for everyone.
    Conducting {
        conductor: Conductor,
        /// The roster, in seating order, as `(register, id)`. The conductor's own copy is the
        /// authority — everyone else reads it off the beacon.
        roster: Vec<(u8, u16)>,
    },
    /// Singing to somebody else's beat.
    Following {
        follower: Follower,
        /// Which duck is conducting, by its **beacon id** — never by its radio address.
        ///
        /// This was the address, and it is the bug that made a chorale never synchronise —
        /// twice, because the first fix was committed with a message describing it and a patch
        /// that had silently not applied. These robots advertise with BLE privacy on, so the
        /// address is a resolvable random one that rotates every few seconds: a follower that
        /// adopted the conductor at one address rejected every beat after the rotation, the
        /// phase lock starved on its single observation, and the duck never sang. The beacon
        /// carries `(register, id)` precisely so identity never comes from the radio layer.
        ///
        /// `None` until the first singing beacon names one.
        conductor: Option<u16>,
        roster: Vec<(u8, u16)>,
        /// The counter last taken, so a repeated beacon is not counted as another beat.
        last_beat: Option<u8>,
    },
}

/// What the loop should do about the chorale this tick.
#[derive(Debug, Clone, PartialEq)]
pub struct Tick {
    /// What to ask `btd` to advertise, when it has changed. `None` means "no change" — the beacon
    /// only needs resending when the beat turns over, not fifty times a second.
    pub advertise: Option<proto::ChoraleAdvertise>,
    /// Which part this duck is singing, and how far into the score, in beats. `None` when it is not
    /// singing — listening, alone, or off.
    pub singing: Option<(Part, f64)>,
    /// Following a conductor but not yet singing — the phase lock is still filling, or the
    /// roster has not seated this duck yet. The state a readout must not call "listening".
    pub joining: bool,
    /// How many ducks are actually singing, this one included.
    ///
    /// Not the roster's length: a duck that walks out of range keeps its *seat* — pruning it would
    /// shift everyone below it onto a different part, which is the one thing the roster exists to
    /// prevent — so its line simply goes unsung, exactly as it would in a choir somebody left.
    pub voices: usize,
}

pub struct Chorale {
    /// This duck's own beacon identity: its register, quantised, and a sixteen-bit tie-break.
    register: u8,
    id: u16,
    /// The epoch every time in this module is measured from.
    ///
    /// `sounds::chorale::beat` works in plain seconds and `robotd` works in `Instant`s, so one of
    /// them has to convert. Doing it here keeps the beat maths free of anything platform-shaped,
    /// which is what lets it be tested against simulated jitter on a laptop.
    started: Instant,
    /// The piece currently loaded — what [`Chorale::score`] serves the audio side, and what
    /// [`Self::piece_id`] names on the air. Swapped when a performance starts or is joined.
    score: Score,
    piece_id: u8,
    state: State,
    peers: Vec<Peer>,
    /// A piece the operator has pinned with `robotctl chorale --piece`.
    /// Only consulted when this duck conducts; a follower sings what the beacon names.
    forced_piece: Option<u8>,
    /// The beacon last handed to `btd`, so it is only resent when it changes.
    advertised: Option<proto::ChoraleBeacon>,
    listening: bool,
}

impl Chorale {
    /// `seed` is the robot's voice seed — the identity everything else is derived from, and which
    /// deliberately does not go on the air.
    pub fn new(pitch_center_hz: f64, seed: u32) -> Self {
        Self {
            register: proto::ChoraleBeacon::quantise_register(pitch_center_hz),
            // Sixteen bits of the seed, mixed — not the eight it used to be: this id is also
            // the own-reflection filter and the peer-merge key, and a real four-duck room
            // collided a byte-sized pair on day one (the fourth duck rolled the conductor's
            // byte, was merged into it by everyone, and could never join). Still not enough
            // to identify a robot, which is the point of it not being the seed.
            id: (seed.wrapping_mul(2_654_435_761) >> 16) as u16,
            started: Instant::now(),
            score: Score::wistful(),
            piece_id: PIECE_WISTFUL,
            forced_piece: None,
            state: State::Off,
            peers: Vec::new(),
            advertised: None,
            listening: false,
        }
    }

    pub fn active(&self) -> bool {
        !matches!(self.state, State::Off)
    }

    /// Start listening for other ducks, or stop and fall silent. `piece_pin` pins what this
    /// duck picks if it ends up conducting; `None` selects one of the available pieces.
    /// Validated at the door by `robot.chorale`, so an unknown id never gets here.
    pub fn set_active(&mut self, active: bool, now: Instant, piece_pin: Option<u8>) {
        if active {
            self.forced_piece = piece_pin.filter(|id| piece(*id).is_some());
        }
        self.set_active_inner(active, now);
    }

    fn set_active_inner(&mut self, active: bool, now: Instant) {
        match (active, &self.state) {
            (true, State::Off) => {
                tracing::warn!(register = self.register, id = self.id, "chorale: listening");
                self.state = State::Listening { since: now };
            }
            (true, _) => {}
            (false, _) => {
                if self.active() {
                    tracing::warn!("chorale: stopping");
                }
                self.state = State::Off;
                self.peers.clear();
            }
        }
    }

    /// A beacon `btd` heard.
    pub fn heard(&mut self, heard: &proto::ChoraleHeard, now: Instant) {
        if !self.active() {
            return;
        }
        // Our own beacon, reflected by something. Not a peer.
        if heard.beacon.register == self.register && heard.beacon.id == self.id {
            return;
        }
        // `btd` reports an age rather than a timestamp — the two daemons share a machine but not an
        // epoch — so the arrival time is that age subtracted from the clock here.
        let at = now
            .checked_sub(Duration::from_micros(heard.age_us))
            .unwrap_or(now);
        let peer = Peer {
            beacon: heard.beacon.clone(),
            at,
        };
        match self
            .peers
            .iter_mut()
            .find(|existing| existing.beacon.id == peer.beacon.id)
        {
            Some(existing) => *existing = peer,
            None => {
                tracing::warn!(
                    register = heard.beacon.register,
                    id = heard.beacon.id,
                    from = %heard.from,
                    "chorale: another duck"
                );
                self.peers.push(peer);
            }
        }

        // A beat from the duck we are following: hand it to the phase lock, stamped when it was
        // heard rather than when it was processed.
        let arrival = at.saturating_duration_since(self.started).as_secs_f64();
        if let State::Following {
            follower,
            conductor,
            roster,
            last_beat,
        } = &mut self.state
        {
            // The first singing beacon names the conductor; after that another duck's beacon
            // cannot pull this one off its beat — which matters when two pieces briefly
            // overlap in one room. By beacon id, never by `heard.from`: the radio address
            // rotates under BLE privacy, and keying on it is how a follower ends up rejecting
            // every beat its conductor sends after the first few seconds.
            if conductor.is_none() && heard.beacon.singing() {
                *conductor = Some(heard.beacon.id);
                tracing::warn!(conductor = heard.beacon.id, "chorale: following");
            }
            if *conductor != Some(heard.beacon.id) || !heard.beacon.singing() {
                return;
            }
            // The same conductor starting a *new performance* — a different piece, or the
            // beat counter jumping backwards (its `robotctl chorale` was restarted, which on
            // a bench happens constantly). Without this, a follower kept singing the old song
            // against the conductor's new one: it never re-read the piece byte, and the beat
            // reset poisoned its phase lock into free-running on a stale fit. A backwards
            // step of a few beats is a restart; a small forwards gap is just beats missed.
            let restarted = heard.beacon.piece != self.piece_id
                || last_beat.is_some_and(|last| {
                    let back = last.wrapping_sub(heard.beacon.beat);
                    (4..128).contains(&back)
                });
            if restarted {
                let Some(score) = piece(heard.beacon.piece) else {
                    // It now sings something this build does not know. Back to listening —
                    // quiet is better than wrong.
                    tracing::warn!(
                        piece = heard.beacon.piece,
                        "chorale: unknown piece; leaving"
                    );
                    self.state = State::Listening { since: at };
                    return;
                };
                tracing::warn!(
                    piece = heard.beacon.piece,
                    "chorale: the conductor started a new performance"
                );
                self.piece_id = heard.beacon.piece;
                self.state = State::Following {
                    follower: Follower::new(score.bpm),
                    conductor: Some(heard.beacon.id),
                    roster: heard.beacon.roster.clone(),
                    last_beat: Some(heard.beacon.beat),
                };
                self.score = score;
                if let State::Following { follower, .. } = &mut self.state {
                    follower.observe(heard.beacon.beat, arrival);
                }
                return;
            }
            roster.clone_from(&heard.beacon.roster);
            // **Only on a change of counter.** A beacon repeats several times per beat, and
            // re-reading the same value is not another beat — it is the same beat, later, and
            // feeding those in would drag the phase late by half an advertising interval.
            if *last_beat != Some(heard.beacon.beat) {
                *last_beat = Some(heard.beacon.beat);
                follower.observe(heard.beacon.beat, arrival);
            }
        }
    }

    /// A slowly-turning byte, so an idle beacon changes and is therefore noticed. See
    /// [`IDLE_HEARTBEAT`].
    fn heartbeat(&self, now: Instant) -> u8 {
        (self.seconds(now) / IDLE_HEARTBEAT.as_secs_f64()) as u64 as u8
    }

    /// Seconds since this module's epoch — the clock `sounds::chorale::beat` speaks in.
    fn seconds(&self, at: Instant) -> f64 {
        at.saturating_duration_since(self.started).as_secs_f64()
    }

    /// One tick. Cheap enough for every one; only the beacon is rate-limited, by changing rarely.
    pub fn tick(&mut self, now: Instant) -> Tick {
        self.peers
            .retain(|peer| now.saturating_duration_since(peer.at) < PEER_STALE);

        match &mut self.state {
            State::Off => {
                let advertise = self.publish(None, false);
                Tick {
                    advertise,
                    singing: None,
                    joining: false,
                    voices: 0,
                }
            }
            State::Listening { since } => {
                let since = *since;
                // Somebody is already singing: join it rather than starting a second piece —
                // but only a piece this build knows. An unknown id means a newer flock; the
                // right move is to keep listening, not to guess at a song.
                if let Some((peer, score)) = self
                    .peers
                    .iter()
                    .filter(|peer| peer.beacon.singing())
                    .find_map(|peer| piece(peer.beacon.piece).map(|score| (peer.clone(), score)))
                {
                    tracing::warn!(
                        piece = peer.beacon.piece,
                        conductor = peer.beacon.id,
                        "chorale: joining"
                    );
                    self.piece_id = peer.beacon.piece;
                    self.state = State::Following {
                        follower: Follower::new(score.bpm),
                        conductor: None,
                        roster: peer.beacon.roster.clone(),
                        last_beat: None,
                    };
                    self.score = score;
                    // The address is not in the beacon, so the first `heard` from this conductor
                    // adopts it — until then this duck listens without a lock, which is what
                    // `Follower` does anyway for its first few beats.
                    return self.tick(now);
                }
                // Nobody singing. If anyone is here and this duck has the lowest id, conduct.
                let alone = self.peers.is_empty();
                let lowest = self.peers.iter().all(|peer| self.id < peer.beacon.id);
                if !alone && lowest && now.saturating_duration_since(since) >= SETTLE {
                    let mut roster: Vec<(u8, u16)> = vec![(self.register, self.id)];
                    roster.extend(self.peers.iter().map(|p| (p.beacon.register, p.beacon.id)));
                    roster.truncate(proto::ChoraleBeacon::MAX_ROSTER);
                    // The conductor picks the piece — forced by the operator if they said so,
                    // otherwise from the clock's low bits at the moment the performance starts:
                    // as good as a coin for something that happens seconds after humans put
                    // ducks near each other, and deterministic under a test that controls the
                    // clock.
                    let pick = self.forced_piece.unwrap_or_else(|| {
                        PIECES[(self.seconds(now) * 997.0) as u64 as usize % PIECES.len()]
                    });
                    self.piece_id = pick;
                    self.score = piece(pick).expect("both built-in pieces exist");
                    tracing::warn!(voices = roster.len(), piece = pick, "chorale: conducting");
                    let roster = roster;
                    self.state = State::Conducting {
                        conductor: Conductor::new(self.score.bpm, self.seconds(now)),
                        roster,
                    };
                    return self.tick(now);
                }
                let idle = self.beacon(proto::ChoraleBeacon::IDLE, self.heartbeat(now), Vec::new());
                let advertise = self.publish(Some(idle), true);
                Tick {
                    advertise,
                    singing: None,
                    joining: false,
                    voices: self.peers.len(),
                }
            }
            State::Conducting { .. } => self.conduct(now),
            State::Following { .. } => self.follow(now),
        }
    }

    fn conduct(&mut self, now: Instant) -> Tick {
        // Everyone in range who is willing, in join order: the roster grows as ducks arrive and
        // never reorders, which is what keeps anyone already singing on their own part.
        let heard: Vec<(u8, u16)> = self
            .peers
            .iter()
            .map(|p| (p.beacon.register, p.beacon.id))
            .collect();
        let State::Conducting { conductor, roster } = &mut self.state else {
            unreachable!("checked by the caller");
        };
        for entry in heard {
            if roster.len() < proto::ChoraleBeacon::MAX_ROSTER && !roster.contains(&entry) {
                tracing::warn!(id = entry.1, "chorale: seating a new voice");
                roster.push(entry);
            }
        }
        let seconds = now.saturating_duration_since(self.started).as_secs_f64();
        conductor.due(seconds);
        let position = conductor.position_beats(seconds);
        let beat = conductor.wire_beat();
        let roster = roster.clone();

        // The piece ends. It used to not — position counted on past the last note forever,
        // ducks reporting "bar 48" of a piece eleven bars long, singing nothing, holding the
        // advertising instance. A finished performance goes back to listening (with the settle
        // timer, so it is a breath, not a gap), and the next one picks a piece afresh.
        let total = self.score.duration_s() / self.score.beat_s();
        if position > total + 2.0 {
            tracing::warn!(piece = self.piece_id, "chorale: the performance ended");
            self.state = State::Listening { since: now };
            return self.tick(now);
        }

        let beacon = self.beacon(self.piece_id, beat, roster.clone());
        let advertise = self.publish(Some(beacon), true);
        Tick {
            advertise,
            singing: self.my_part(&roster).map(|part| (part, position)),
            joining: false,
            voices: self.voices(&roster),
        }
    }

    /// Roster entries that are still audible: this duck, plus peers heard recently enough.
    ///
    /// The roster keeps a departed duck's seat so nobody else is reseated; this is the count that
    /// says how many are actually singing.
    fn voices(&self, roster: &[(u8, u16)]) -> usize {
        roster
            .iter()
            .filter(|(register, id)| {
                (*register == self.register && *id == self.id)
                    || self.peers.iter().any(|peer| peer.beacon.id == *id)
            })
            .count()
    }

    fn follow(&mut self, now: Instant) -> Tick {
        let State::Following {
            follower, roster, ..
        } = &mut self.state
        else {
            unreachable!("checked by the caller");
        };
        let roster = roster.clone();
        let seconds = now.saturating_duration_since(self.started).as_secs_f64();
        let position = follower.position_beats(seconds);
        // The follower's own end-of-piece, a few beats later than the conductor's so it is the
        // conductor's restart that leads when both are healthy — this path is for a follower
        // whose conductor vanished mid-final-chord.
        let total = self.score.duration_s() / self.score.beat_s();
        if position.is_some_and(|beats| beats > total + 6.0) {
            tracing::warn!(piece = self.piece_id, "chorale: the performance ended");
            self.state = State::Listening { since: now };
            return self.tick(now);
        }
        // A follower advertises an idle beacon: it is willing and findable, but it is not the one
        // holding the beat, and two beacons carrying a piece would be two conductors.
        let idle = self.beacon(proto::ChoraleBeacon::IDLE, self.heartbeat(now), Vec::new());
        let advertise = self.publish(Some(idle), true);
        let singing = position.and_then(|at| self.my_part(&roster).map(|part| (part, at)));
        Tick {
            advertise,
            joining: singing.is_none(),
            singing,
            voices: self.voices(&roster),
        }
    }

    /// This duck's part, from the roster the conductor published.
    ///
    /// `None` when it is not in the roster yet — which is not an error but the ordinary state of a
    /// duck that has just arrived. It keeps listening, and the conductor seats it on the next beat.
    fn my_part(&self, roster: &[(u8, u16)]) -> Option<Part> {
        let seat = roster
            .iter()
            .position(|(register, id)| *register == self.register && *id == self.id)?;
        let registers: Vec<f64> = roster
            .iter()
            .map(|(register, _)| {
                proto::ChoraleBeacon::REGISTER_LOW_HZ
                    + f64::from(*register) * proto::ChoraleBeacon::REGISTER_STEP_HZ
            })
            .collect();
        seat_all(&registers).get(seat).copied()
    }

    fn beacon(&self, piece: u8, beat: u8, roster: Vec<(u8, u16)>) -> proto::ChoraleBeacon {
        proto::ChoraleBeacon {
            piece,
            beat,
            register: self.register,
            id: self.id,
            roster,
        }
    }

    /// Hand `btd` a beacon, but only when it has actually changed.
    ///
    /// The loop runs at 50 Hz and the beat turns over about once a second, so this is the
    /// difference between two D-Bus round trips a second and a hundred.
    fn publish(
        &mut self,
        beacon: Option<proto::ChoraleBeacon>,
        listening: bool,
    ) -> Option<proto::ChoraleAdvertise> {
        if beacon == self.advertised && listening == self.listening {
            return None;
        }
        self.advertised = beacon.clone();
        self.listening = listening;
        Some(proto::ChoraleAdvertise { beacon, listening })
    }

    /// The score being sung, for the audio side to render.
    pub fn score(&self) -> &Score {
        &self.score
    }
}

/// Expressive head offsets while singing: `[neck_pitch, head_pitch, head_yaw, head_roll]`.
///
/// Driven by the **score position**, not by local time — and that is the trick: every duck in
/// the ensemble computes this from the same shared beat, so the whole group sways in phase
/// with nobody coordinating anything. Choreography falls out of the sync work for free.
///
/// `reach` is where the current note sits in this duck's own line (0 low, 1 high): the head
/// lifts on the high notes, which is what a singer actually does.
///
/// Amplitudes are deliberately small — the head carries the ToF and the policy's balance has
/// opinions about mass this high up. The pitch sign assumes negative is up, as `robot.look`'s
/// examples suggest; if hardware says otherwise, flip `REACH_LIFT`.
pub fn head_expression(beats: f64, reach: f64) -> [f64; 4] {
    const SWAY_ROLL: f64 = 0.08;
    const DRIFT_YAW: f64 = 0.05;
    const REACH_LIFT: f64 = -0.10;
    const BOB_PITCH: f64 = 0.025;
    let bar = std::f64::consts::TAU * beats / 4.0;
    let phrase = std::f64::consts::TAU * beats / 8.0;
    let beat = std::f64::consts::TAU * beats;
    [
        0.0,
        REACH_LIFT * reach + BOB_PITCH * beat.sin(),
        DRIFT_YAW * phrase.sin(),
        SWAY_ROLL * bar.sin(),
    ]
}
