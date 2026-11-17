//! The text score format: a chorale you can write in an editor and diff in a review.
//!
//! Scores started as `Gesture` literals in Rust, which is fine for the one piece that ships
//! and wrong for making songs: every change is a recompile, and nobody who writes music is
//! going to touch a `vec![Gesture::Chord { .. }]`. This is the same gestures in a line-oriented
//! text file — one gesture per line, note names rather than MIDI numbers, running state for the
//! things a score marks once and means until further notice (the dynamic, the vowel).
//!
//! `scores/wistful.duckscore` is the grammar's own documentation and is embedded as
//! [`super::Score::wistful`], so the shipped piece and the worked example are the same file and
//! cannot drift apart.
//!
//! ## What this is not
//!
//! Not a general music format. It has no bars, no key signature, no time signature and no
//! per-voice rhythmic independence beyond what [`Gesture`] can express — a score where all four
//! voices move in different rhythms is not writable here, and is exactly what the MIDI importer
//! ([`super::midi`]) is for. The two front ends are deliberately different shapes: this one is
//! for writing a chorale by hand and reading it back, that one is for anything a notation
//! editor can produce.
//!
//! ## Errors name the line
//!
//! Every failure carries the line number and the text of the line, because the alternative —
//! "invalid score" — sends someone back to stare at forty lines of chords. A score is
//! hand-written data, so a parse error is the normal way to find out you mistyped a note name.

use std::fmt;

use super::{Gesture, Part, Score, SoloNote, Voicing, Vowel, dynamic};

/// What went wrong, and where.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseError {
    /// 1-based, as an editor counts.
    pub line: usize,
    pub text: String,
    pub problem: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "line {}: {}\n  {}",
            self.line,
            self.problem,
            self.text.trim()
        )
    }
}

impl std::error::Error for ParseError {}

/// Parse a score.
pub fn parse(source: &str) -> Result<Score, ParseError> {
    let mut name = "untitled".to_owned();
    let mut bpm = 60.0f64;
    let mut gestures: Vec<Gesture> = Vec::new();
    // Running state: what a score marks once and means until it says otherwise.
    let mut level = dynamic("mf").expect("mf is a dynamic");
    let mut vowel = Vowel::Ah;

    for (index, raw) in source.lines().enumerate() {
        let line = index + 1;
        let fail = |problem: String| ParseError {
            line,
            text: raw.to_owned(),
            problem,
        };
        // Comments run to end of line, so a chord can be annotated with what it is.
        let content = raw.split('#').next().unwrap_or("").trim();
        if content.is_empty() {
            continue;
        }

        if let Some(rest) = content.strip_prefix("name:") {
            name = rest.trim().to_owned();
            continue;
        }
        if let Some(rest) = content.strip_prefix("bpm:") {
            bpm = rest
                .trim()
                .parse()
                .map_err(|_| fail(format!("{:?} is not a tempo", rest.trim())))?;
            if !(20.0..=300.0).contains(&bpm) {
                return Err(fail(format!("{bpm} bpm is not a tempo anyone can sing")));
            }
            continue;
        }

        let words: Vec<&str> = content.split_whitespace().collect();
        match words[0] {
            "dynamic" => {
                let mark = words.get(1).ok_or_else(|| fail("dynamic what?".into()))?;
                level = dynamic(mark).ok_or_else(|| {
                    fail(format!("{mark:?} is not a dynamic (ppp pp p mp mf f ff)"))
                })?;
            }
            "vowel" => {
                let named = words.get(1).ok_or_else(|| fail("vowel what?".into()))?;
                vowel = Vowel::parse(named)
                    .ok_or_else(|| fail(format!("{named:?} is not a vowel (ah eh ee oh oo mm)")))?;
            }
            "rest" => {
                let beats = beats_of(words.get(1), &fail)?;
                gestures.push(Gesture::Rest { beats });
            }
            "chord" => {
                let beats = beats_of(words.get(1), &fail)?;
                let voicing = voicing_of(&words[2..], &fail)?;
                gestures.push(Gesture::Chord {
                    voicing,
                    beats,
                    vowel,
                    level,
                });
            }
            "build" => {
                let beats = beats_of(words.get(1), &fail)?;
                if words.get(2) != Some(&"stagger") {
                    return Err(fail("build needs `stagger <beats>`".into()));
                }
                let stagger = beats_of(words.get(3), &fail)?;
                // `top` is optional and sits between the stagger and the voicing.
                let (from_top, voicing_from) = match words.get(4) {
                    Some(&"top") => (true, 5),
                    _ => (false, 4),
                };
                let voicing = voicing_of(&words[voicing_from.min(words.len())..], &fail)?;
                gestures.push(Gesture::Build {
                    voicing,
                    beats,
                    stagger,
                    from_top,
                    vowel,
                    level,
                });
            }
            "solo" => gestures.push(solo(&words, vowel, level, &fail)?),
            other => {
                return Err(fail(format!(
                    "{other:?} is not a gesture (chord build solo rest dynamic vowel)"
                )));
            }
        }
    }

    if gestures.is_empty() {
        return Err(ParseError {
            line: 0,
            text: String::new(),
            problem: "a score with no gestures in it is not a score".into(),
        });
    }
    Ok(Score::from_gestures(&name, bpm, &gestures))
}

/// `solo <part> under B T A S [hum <vowel>] sing <note>[:beats][/vowel] ...`
fn solo(
    words: &[&str],
    running_vowel: Vowel,
    level: f64,
    fail: &impl Fn(String) -> ParseError,
) -> Result<Gesture, ParseError> {
    let named = words.get(1).ok_or_else(|| fail("solo who?".into()))?;
    let part = part_of(named)
        .ok_or_else(|| fail(format!("{named:?} is not a part (bass tenor alto soprano)")))?;
    let keyword = |word: &str| words.iter().position(|w| *w == word);
    let under_at = keyword("under").ok_or_else(|| fail("solo needs `under B T A S`".into()))?;
    let sing_at = keyword("sing").ok_or_else(|| fail("solo needs `sing <notes>`".into()))?;
    if sing_at <= under_at {
        return Err(fail("`under` comes before `sing`".into()));
    }
    // `hum <vowel>` is optional; a choir behind a soloist hums unless told otherwise.
    let (under_vowel, under_end) = match keyword("hum") {
        Some(at) if at > under_at && at < sing_at => {
            let named = words
                .get(at + 1)
                .ok_or_else(|| fail("hum on what vowel?".into()))?;
            let vowel =
                Vowel::parse(named).ok_or_else(|| fail(format!("{named:?} is not a vowel")))?;
            (vowel, at)
        }
        _ => (Vowel::Mm, sing_at),
    };
    let under = voicing_of(&words[under_at + 1..under_end], fail)?;

    let mut line = Vec::new();
    for word in &words[sing_at + 1..] {
        // `E4`, `E4:2`, `E4/ee`, `E4:2/ee` — a pitch, then optionally how long, then optionally
        // on what. Beats default to one, the vowel to whatever is running.
        let (pitch_and_beats, vowel) = match word.split_once('/') {
            Some((head, named)) => (
                head,
                Vowel::parse(named).ok_or_else(|| fail(format!("{named:?} is not a vowel")))?,
            ),
            None => (*word, running_vowel),
        };
        let (pitch, beats) = match pitch_and_beats.split_once(':') {
            Some((pitch, beats)) => (
                pitch,
                beats
                    .parse::<f64>()
                    .map_err(|_| fail(format!("{beats:?} is not a length")))?,
            ),
            None => (pitch_and_beats, 1.0),
        };
        if beats <= 0.0 {
            return Err(fail(format!("a note cannot last {beats} beats")));
        }
        let midi = midi_of(pitch)
            .ok_or_else(|| fail(format!("{pitch:?} is not a note name")))?
            .ok_or_else(|| fail("a soloist cannot sing a rest — use `rest`".into()))?;
        line.push(SoloNote { midi, beats, vowel });
    }
    if line.is_empty() {
        return Err(fail("a solo with no notes in it".into()));
    }
    Ok(Gesture::Solo {
        part,
        notes: line,
        under,
        under_vowel,
        level,
    })
}

fn beats_of(word: Option<&&str>, fail: &impl Fn(String) -> ParseError) -> Result<f64, ParseError> {
    let word = word.ok_or_else(|| fail("expected a number of beats".into()))?;
    let beats: f64 = word
        .parse()
        .map_err(|_| fail(format!("{word:?} is not a number of beats")))?;
    if beats <= 0.0 || !beats.is_finite() {
        return Err(fail(format!("{beats} is not a length")));
    }
    Ok(beats)
}

/// Four note names, low to high. `-` is a voice that does not sing.
fn voicing_of(words: &[&str], fail: &impl Fn(String) -> ParseError) -> Result<Voicing, ParseError> {
    if words.len() != 4 {
        return Err(fail(format!(
            "expected four voices low to high (bass tenor alto soprano), got {}",
            words.len()
        )));
    }
    let mut voicing: Voicing = [None; 4];
    for (slot, word) in voicing.iter_mut().zip(words) {
        *slot = midi_of(word).ok_or_else(|| fail(format!("{word:?} is not a note name")))?;
    }
    Ok(voicing)
}

fn part_of(word: &str) -> Option<Part> {
    Some(match word.to_ascii_lowercase().as_str() {
        "bass" | "b" => Part::Bass,
        "tenor" | "t" => Part::Tenor,
        "alto" | "a" => Part::Alto,
        "soprano" | "s" => Part::Soprano,
        _ => return None,
    })
}

/// A scientific pitch name to MIDI. `Ok(None)` is the tacet marker `-`.
///
/// C4 is middle C, which is MIDI 60 — the convention a notation editor uses, so a note copied
/// off a staff means what it looks like. (The other convention in circulation puts middle C at
/// C3; picking the editor's is the one that avoids an octave of confusion per score.)
fn midi_of(word: &str) -> Option<Option<u8>> {
    if word == "-" {
        return Some(None);
    }
    let mut chars = word.chars();
    let step = match chars.next()?.to_ascii_uppercase() {
        'C' => 0,
        'D' => 2,
        'E' => 4,
        'F' => 5,
        'G' => 7,
        'A' => 9,
        'B' => 11,
        _ => return None,
    };
    let mut rest = chars.as_str();
    let mut accidental = 0i32;
    while let Some(first) = rest.chars().next() {
        match first {
            '#' | 's' => accidental += 1,
            'b' | 'f' => accidental -= 1,
            _ => break,
        }
        rest = &rest[first.len_utf8()..];
    }
    let octave: i32 = rest.parse().ok()?;
    let midi = (octave + 1) * 12 + step + accidental;
    if !(0..=127).contains(&midi) {
        return None;
    }
    Some(Some(midi as u8))
}
