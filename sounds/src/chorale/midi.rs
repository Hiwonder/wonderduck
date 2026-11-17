//! Standard MIDI file in, chorale out — so a notation editor is the score editor.
//!
//! This is the front end that actually unlocks writing music for the ducks. The text format
//! ([`super::text`]) is good for a chorale you write by hand and read back, and it cannot
//! express four voices moving in four different rhythms. A MIDI file can, every notation editor
//! and DAW exports one, and the entire public-domain four-part repertoire is already available
//! as one — Bach chorales most of all, which are literally this genre.
//!
//! **No dependency.** A Standard MIDI File is a length-prefixed chunk format with variable-length
//! delta times, and the subset a score needs is note-on, note-off, tempo and track name. That is
//! a couple of hundred lines and no supply chain, against a crate that would parse controller
//! automation and SysEx this will never look at.
//!
//! ## How tracks become parts
//!
//! By **mean pitch**, not by track order — the lowest group of notes sings bass. Track order
//! would be the obvious rule and is wrong twice over: notation editors write scores top staff
//! first (soprano, alto, tenor, bass — the reverse of what a `Voicing` lists), and a file from a
//! DAW may have the tempo track, empty tracks, or the parts in any order at all. Sorting by
//! pitch is right whatever produced the file, and it is the same rule [`super::cast`] uses to
//! seat the ducks, which keeps one idea in one place. A track *name* that says "Soprano" is
//! believed first, since a human wrote it down.
//!
//! A single polyphonic track — one piano staff with chords in it, which is what "export MIDI"
//! gives you from a lot of tools — is split by pitch rank within each chord. Crude and better
//! than refusing: the lowest note of each chord is the bass line.
//!
//! ## What is deliberately dropped
//!
//! Velocity becomes the note's dynamic, which is free and right. Everything else — controllers,
//! pitch bend, program changes, anything past the first tempo — is skipped: a duck has one
//! oscillator and a mouth, and a tempo map would have to be honoured identically by four robots
//! agreeing over a network, which is a promise this cannot keep. A file with a tempo change
//! parses and is sung at its first tempo, with the fact reported rather than hidden.

use std::collections::HashMap;
use std::fmt;

use super::{Note, Part, Score, Vowel};

/// What went wrong reading the file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MidiError {
    /// Not a Standard MIDI File at all.
    NotMidi,
    /// Truncated, or a chunk length that runs off the end.
    Truncated { at: usize },
    /// SMPTE timecode division. Vanishingly rare from a notation editor, and the conversion to
    /// beats is different enough that guessing would be worse than saying so.
    SmpteTiming,
    /// No note-on events anywhere in the file.
    NoNotes,
}

impl fmt::Display for MidiError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MidiError::NotMidi => write!(f, "not a MIDI file (no MThd header)"),
            MidiError::Truncated { at } => write!(f, "the file ends mid-event, at byte {at}"),
            MidiError::SmpteTiming => write!(
                f,
                "SMPTE timecode division is not supported — export with metrical (ticks-per-beat) timing"
            ),
            MidiError::NoNotes => write!(f, "there are no notes in this file"),
        }
    }
}

impl std::error::Error for MidiError {}

/// What was in the file, besides the notes — reported so an import is not silent about what it
/// dropped.
#[derive(Debug, Clone, PartialEq)]
pub struct Import {
    pub score: Score,
    /// Track names, in file order, for the ones that had notes.
    pub tracks: Vec<String>,
    /// How each part was decided.
    pub casting: Vec<(Part, String)>,
    /// Things skipped that someone might have expected to survive.
    pub dropped: Vec<String>,
}

/// One note as the file describes it, before parts are assigned.
#[derive(Debug, Clone, Copy)]
struct Raw {
    group: usize,
    start_tick: u64,
    end_tick: u64,
    midi: u8,
    velocity: u8,
}

/// Read a Standard MIDI File.
pub fn parse(bytes: &[u8]) -> Result<Import, MidiError> {
    // Checked before anything else can fail: a file that is not MIDI at all should say so,
    // not report a truncation at byte zero.
    if bytes.len() < 14 || bytes[..4] != *b"MThd" {
        return Err(MidiError::NotMidi);
    }
    let mut reader = Reader::new(bytes);
    reader.skip(4)?;
    let header_len = reader.u32()? as usize;
    let header_end = reader.at + header_len;
    let _format = reader.u16()?;
    let _tracks = reader.u16()?;
    let division = reader.u16()?;
    if division & 0x8000 != 0 {
        return Err(MidiError::SmpteTiming);
    }
    let ticks_per_beat = f64::from(division.max(1));
    reader.at = header_end.min(bytes.len());

    let mut raws: Vec<Raw> = Vec::new();
    let mut names: Vec<String> = Vec::new();
    let mut dropped: Vec<String> = Vec::new();
    // Microseconds per quarter note. 500_000 is MIDI's default — 120 bpm.
    let mut micros_per_beat = 500_000f64;
    let mut tempo_seen = false;
    // One group per (track, channel): a file may put two voices on one track and different
    // channels, and that is a real two-part export.
    let mut groups: HashMap<(usize, u8), usize> = HashMap::new();

    let mut track_index = 0usize;
    while reader.remaining() >= 8 {
        let tag = reader.tag()?;
        let length = reader.u32()? as usize;
        let end = reader.at + length;
        if end > bytes.len() {
            return Err(MidiError::Truncated { at: reader.at });
        }
        if tag != *b"MTrk" {
            // An unknown chunk is skippable by design — the spec says so, which is what the
            // length prefix is for.
            reader.at = end;
            continue;
        }

        let mut name = String::new();
        let mut tick = 0u64;
        let mut status = 0u8;
        // Sounding notes, keyed by (channel, pitch). A second note-on for a pitch already
        // sounding ends the first, which is what a sane sequencer means by it.
        let mut open: HashMap<(u8, u8), (u64, u8)> = HashMap::new();

        while reader.at < end {
            tick += reader.varint()?;
            let mut byte = reader.u8()?;
            if byte < 0x80 {
                // Running status: this byte is data, and the previous status still applies.
                reader.at -= 1;
                byte = status;
            } else {
                status = byte;
            }
            match byte & 0xF0 {
                0x80 | 0x90 => {
                    let channel = byte & 0x0F;
                    let pitch = reader.u8()? & 0x7F;
                    let velocity = reader.u8()? & 0x7F;
                    // Note-on with zero velocity is a note-off; every file in the world uses it.
                    let is_on = byte & 0xF0 == 0x90 && velocity > 0;
                    if is_on {
                        if let Some((start, velocity)) =
                            open.insert((channel, pitch), (tick, velocity))
                        {
                            push(
                                &mut raws,
                                &mut groups,
                                track_index,
                                channel,
                                start,
                                tick,
                                pitch,
                                velocity,
                            );
                        }
                    } else if let Some((start, velocity)) = open.remove(&(channel, pitch)) {
                        push(
                            &mut raws,
                            &mut groups,
                            track_index,
                            channel,
                            start,
                            tick,
                            pitch,
                            velocity,
                        );
                    }
                }
                // Two data bytes, none of them ours.
                0xA0 | 0xB0 | 0xE0 => {
                    reader.skip(2)?;
                }
                // One data byte.
                0xC0 | 0xD0 => {
                    reader.skip(1)?;
                }
                0xF0 => match byte {
                    0xFF => {
                        let kind = reader.u8()?;
                        let length = reader.varint()? as usize;
                        let data = reader.take(length)?;
                        match kind {
                            0x03 if name.is_empty() => {
                                name = String::from_utf8_lossy(data).trim().to_owned();
                            }
                            0x51 if data.len() == 3 => {
                                let value =
                                    f64::from(u32::from_be_bytes([0, data[0], data[1], data[2]]));
                                if tempo_seen {
                                    if (value - micros_per_beat).abs() > 1.0
                                        && !dropped.iter().any(|d| d.starts_with("a tempo change"))
                                    {
                                        dropped.push(
                                            "a tempo change (the whole piece is sung at the first tempo)"
                                                .to_owned(),
                                        );
                                    }
                                } else {
                                    micros_per_beat = value;
                                    tempo_seen = true;
                                }
                            }
                            _ => {}
                        }
                    }
                    // SysEx: length-prefixed, and nothing here cares what is in it.
                    0xF0 | 0xF7 => {
                        let length = reader.varint()? as usize;
                        reader.skip(length)?;
                    }
                    _ => {}
                },
                _ => {}
            }
        }
        // A note still sounding at the end of the track ends there rather than being lost.
        for ((channel, pitch), (start, velocity)) in open {
            push(
                &mut raws,
                &mut groups,
                track_index,
                channel,
                start,
                tick,
                pitch,
                velocity,
            );
        }
        if raws.iter().any(|r| {
            groups
                .iter()
                .any(|((t, _), g)| *t == track_index && *g == r.group)
        }) {
            names.push(if name.is_empty() {
                format!("track {}", track_index + 1)
            } else {
                name
            });
        }
        reader.at = end;
        track_index += 1;
    }

    if raws.is_empty() {
        return Err(MidiError::NoNotes);
    }
    let bpm = 60_000_000.0 / micros_per_beat.max(1.0);
    let (notes, casting, mut assign_dropped) = assign(&mut raws, &names, ticks_per_beat);
    dropped.append(&mut assign_dropped);

    Ok(Import {
        score: Score::from_notes("imported", bpm, notes),
        tracks: names,
        casting,
        dropped,
    })
}

#[allow(clippy::too_many_arguments)]
fn push(
    raws: &mut Vec<Raw>,
    groups: &mut HashMap<(usize, u8), usize>,
    track: usize,
    channel: u8,
    start_tick: u64,
    end_tick: u64,
    midi: u8,
    velocity: u8,
) {
    if end_tick <= start_tick {
        return;
    }
    let next = groups.len();
    let group = *groups.entry((track, channel)).or_insert(next);
    raws.push(Raw {
        group,
        start_tick,
        end_tick,
        midi,
        velocity,
    });
}

/// Turn raw note groups into four parts. See the module docs for why by pitch and not by order.
fn assign(
    raws: &mut [Raw],
    names: &[String],
    ticks_per_beat: f64,
) -> (Vec<Note>, Vec<(Part, String)>, Vec<String>) {
    let mut dropped = Vec::new();
    let group_count = raws.iter().map(|r| r.group).max().unwrap_or(0) + 1;

    // One group with chords in it is a piano staff: split it by pitch rank per chord.
    if group_count == 1 {
        split_by_rank(raws);
    }
    let group_count = raws.iter().map(|r| r.group).max().unwrap_or(0) + 1;

    // Mean pitch per group, lowest first.
    let mut order: Vec<(usize, f64)> = (0..group_count)
        .filter_map(|group| {
            let pitches: Vec<f64> = raws
                .iter()
                .filter(|r| r.group == group)
                .map(|r| f64::from(r.midi))
                .collect();
            if pitches.is_empty() {
                return None;
            }
            Some((group, pitches.iter().sum::<f64>() / pitches.len() as f64))
        })
        .collect();
    order.sort_by(|a, b| a.1.partial_cmp(&b.1).expect("pitches are finite"));

    if order.len() > 4 {
        dropped.push(format!(
            "{} voices past the first four (a duck chorale has four parts)",
            order.len() - 4
        ));
        order.truncate(4);
    }

    // A named part beats a measured one: a human wrote the name down.
    let mut seats: Vec<(usize, Part, String)> = Vec::new();
    let mut taken = [false; 4];
    for (rank, (group, _)) in order.iter().enumerate() {
        let named = names.get(*group).and_then(|name| part_named(name));
        if let Some(part) = named
            && !taken[part as usize]
        {
            taken[part as usize] = true;
            seats.push((*group, part, format!("named {:?}", names[*group])));
            continue;
        }
        seats.push((*group, Part::Bass, format!("rank {}", rank + 1)));
    }
    // Fill the unnamed seats with the parts nobody claimed, low to high, in pitch order.
    let mut spare: Vec<Part> = Part::ensemble(order.len())
        .into_iter()
        .filter(|p| !taken[*p as usize])
        .collect();
    for seat in seats.iter_mut() {
        if seat.2.starts_with("rank")
            && let Some(part) = spare.first().copied()
        {
            spare.remove(0);
            seat.1 = part;
            seat.2 = format!("{} by pitch", seat.2);
        }
    }

    let mut notes = Vec::new();
    let mut casting = Vec::new();
    for (group, part, why) in &seats {
        casting.push((*part, why.clone()));
        for raw in raws.iter().filter(|r| r.group == *group) {
            notes.push(Note {
                part: *part,
                start_beat: raw.start_tick as f64 / ticks_per_beat,
                beats: (raw.end_tick - raw.start_tick) as f64 / ticks_per_beat,
                midi: raw.midi,
                // Velocity is a dynamic, which is free and right. MIDI's nominal `mf` is 64.
                level: (f64::from(raw.velocity) / 100.0).clamp(0.15, 1.0),
                vowel: Vowel::Ah,
            });
        }
    }
    casting.sort_by_key(|(part, _)| *part);
    (notes, casting, dropped)
}

/// Split one polyphonic group into voices by pitch rank within each chord.
///
/// The lowest note sounding at a given onset is the bass line, the next is the tenor, and so on.
/// Crude — a real voice-splitter follows lines across time rather than ranking each chord
/// independently — and it turns a piano-staff export into something singable, which is the whole
/// bar it has to clear.
fn split_by_rank(raws: &mut [Raw]) {
    let mut onsets: Vec<u64> = raws.iter().map(|r| r.start_tick).collect();
    onsets.sort_unstable();
    onsets.dedup();
    for onset in onsets {
        let mut together: Vec<usize> = (0..raws.len())
            .filter(|i| raws[*i].start_tick == onset)
            .collect();
        together.sort_by_key(|i| raws[*i].midi);
        for (rank, index) in together.into_iter().enumerate() {
            raws[index].group = rank.min(3);
        }
    }
}

/// A part name a human might have typed on a staff.
fn part_named(name: &str) -> Option<Part> {
    let name = name.to_ascii_lowercase();
    // Longest first, so "bassoon" does not become the bass and "soprano 1" does.
    for (needle, part) in [
        ("soprano", Part::Soprano),
        ("tenor", Part::Tenor),
        ("alto", Part::Alto),
        ("bass", Part::Bass),
        ("sop", Part::Soprano),
    ] {
        if name.contains(needle) {
            return Some(part);
        }
    }
    None
}

/// A cursor over the file, whose every read is bounds-checked — this parses bytes from a file
/// somebody downloaded, and a panic on a truncated one would be a crash where an error belongs.
struct Reader<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, at: 0 }
    }

    fn remaining(&self) -> usize {
        self.bytes.len().saturating_sub(self.at)
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], MidiError> {
        let end = self
            .at
            .checked_add(n)
            .ok_or(MidiError::Truncated { at: self.at })?;
        let slice = self
            .bytes
            .get(self.at..end)
            .ok_or(MidiError::Truncated { at: self.at })?;
        self.at = end;
        Ok(slice)
    }

    fn skip(&mut self, n: usize) -> Result<(), MidiError> {
        self.take(n).map(|_| ())
    }

    fn u8(&mut self) -> Result<u8, MidiError> {
        Ok(self.take(1)?[0])
    }

    fn u16(&mut self) -> Result<u16, MidiError> {
        let b = self.take(2)?;
        Ok(u16::from_be_bytes([b[0], b[1]]))
    }

    fn u32(&mut self) -> Result<u32, MidiError> {
        let b = self.take(4)?;
        Ok(u32::from_be_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn tag(&mut self) -> Result<[u8; 4], MidiError> {
        let b = self.take(4)?;
        Ok([b[0], b[1], b[2], b[3]])
    }

    /// MIDI's variable-length quantity: seven bits per byte, high bit means "more follows".
    fn varint(&mut self) -> Result<u64, MidiError> {
        let mut value = 0u64;
        // Four bytes is the spec's maximum; more than that is a corrupt file, not a big number.
        for _ in 0..4 {
            let byte = self.u8()?;
            value = (value << 7) | u64::from(byte & 0x7F);
            if byte & 0x80 == 0 {
                return Ok(value);
            }
        }
        Err(MidiError::Truncated { at: self.at })
    }
}
