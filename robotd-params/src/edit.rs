//! Editing `robotd.toml` without disturbing it.
//!
//! The lossless writer for this crate's own schema: parse with `toml_edit`, set or remove exactly
//! the keys asked for, re-parse the candidate through [`Params::load`] before anything reaches the
//! disk, and write atomically. Comments, ordering and keys from releases this build does not know
//! all survive.
//!
//! ## Why it is here rather than in `robotctl`
//!
//! It began as part of `robotctl configure`, which was the only thing that wrote this file. Then
//! two more callers appeared: `robotctl policy` and `robotctl pad` write keys of their own, and a
//! daemon serving `pad.bind` over the radio has to write the same file — `padd` re-reads `[pad]`
//! every second, so a binding that only changed something in memory would be reverted before
//! anybody let go of the phone.
//!
//! A second implementation would drift from this one, and the thing it would drift on is the
//! validation: the guarantee worth keeping is that **nothing writes a file `robotd` would refuse
//! to start on**, and that guarantee is only as good as its least careful writer. So the writer
//! lives beside the schema it enforces, in the crate that defines it.
//!
//! `toml_edit` is pure Rust, so this crate's rule that nothing here grows a C toolchain or a
//! network stack still holds — which matters, because this is the crate on the recovery path.
//!
//! ## What is not here
//!
//! Which systemd unit a change needs restarted, how to restart it, and how any of this is
//! presented. Those are a host's business and an operator tool's business, and they stay in
//! `robotctl`.

use std::collections::BTreeMap;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use crate::Params;
use crate::registry::{Entry, Kind, REGISTRY, RENAMED_SECTIONS};
use toml_edit::DocumentMut;

/// One key's place in the world: what the file says, what the default is.
#[derive(Debug, Clone)]
pub struct Row {
    pub entry: &'static Entry,
    /// The value in the file, rendered, if the file sets it.
    pub set: Option<String>,
    /// The built-in default, rendered the same way.
    pub default: String,
    /// What an *unset* optional key actually resolves to — per mode, per release — when the
    /// daemon can say. The bare word `unset` told nobody anything.
    pub resolved: Option<String>,
}

impl Row {
    /// What the daemon would actually run with.
    pub fn effective(&self) -> &str {
        self.set.as_deref().unwrap_or(&self.default)
    }

    /// Whether the file overrides the default.
    pub fn overridden(&self) -> bool {
        self.set.is_some()
    }

    /// Whether the value *differs* from the default — the thing worth a colour. A file that
    /// writes the default out explicitly (the shipped example does) is not a divergence.
    pub fn differs(&self) -> bool {
        self.set.as_deref().is_some_and(|set| set != self.default)
    }
}

/// A pending edit: set the key to a value, or clear its override.
#[derive(Debug, Clone)]
pub enum Edit {
    Set(toml_edit::Value),
    Clear,
}

/// The editable state of one file: the parsed document, and the rows over it.
pub struct Model {
    pub path: PathBuf,
    doc: DocumentMut,
    defaults: toml::Value,
    /// Keyed by `section.key`. Applied to the document only on save.
    pub pending: BTreeMap<&'static str, Edit>,
    /// What has actually been written, across every save this session.
    ///
    /// Kept because `pending` is *cleared* by a save, and what wants restarting is decided after
    /// the editor has closed — reading `pending` there found nothing every time, so nothing was
    /// ever restarted and a `[duck_detector]` change looked like a no-op.
    written: Vec<String>,
}

impl Model {
    /// Every key written this session, in the order it was first written.
    pub fn written(&self) -> &[String] {
        &self.written
    }

    /// Load the file — or start from an empty document when there is none, which is a real
    /// state: a robot may run entirely on defaults with no file at all.
    pub fn load(path: &Path) -> Result<Self, String> {
        let text = match std::fs::read_to_string(path) {
            Ok(text) => text,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => String::new(),
            Err(e) => return Err(format!("cannot read {}: {e}", path.display())),
        };
        Self::from_text(path, &text)
    }

    /// A model over text already in hand, for a caller that has read the file itself — and for
    /// testing any caller, which is what it is mostly for: building a config from a string beats
    /// a temporary file for saying what a case is about. [`Model::load`] is this plus the read.
    pub fn from_text(path: &Path, text: &str) -> Result<Self, String> {
        // The daemon's own parse first: a file robotd would refuse is not a file to edit
        // blind, and the error names the line.
        toml::from_str::<Params>(text).map_err(|e| format!("{}: {e}", path.display()))?;
        let mut doc: DocumentMut = text
            .parse()
            .map_err(|e| format!("{}: {e}", path.display()))?;
        Self::migrate_renamed_sections(&mut doc);
        Ok(Self {
            path: path.to_path_buf(),
            doc,
            defaults: toml::Value::try_from(Params::default()).expect("Params serializes"),
            pending: BTreeMap::new(),
            written: Vec::new(),
        })
    }

    /// Sections that changed name, carried to the new one in the document itself.
    ///
    /// The loader accepts the old name through a serde alias, so a file written before the rename
    /// keeps working untouched. This is the other half: an *editor* that then set a key under the new
    /// name would leave the file with both sections, which the loader refuses as a duplicate — so the
    /// old header is renamed here, once, and the next save writes the file under the new name only.
    /// Nothing is written until something else is saved; a read-only browse changes no file.
    ///
    /// Called from [`Model::from_text`] and again in [`Model::save`], which re-reads the file
    /// from disk before rendering and would otherwise write the old header straight back.
    fn migrate_renamed_sections(doc: &mut DocumentMut) {
        for (old, new) in RENAMED_SECTIONS {
            if doc.contains_key(new) {
                // Both present: the loader's duplicate-field error already named it. Not ours to
                // guess which wins.
                continue;
            }
            if let Some(item) = doc.remove(old) {
                doc.insert(new, item);
            }
        }
    }

    /// Every key the daemon knows, in registry order, with pending edits shown as if applied.
    pub fn rows(&self) -> Vec<Row> {
        REGISTRY
            .iter()
            .map(|entry| {
                let set = match self.pending.get(entry.key) {
                    Some(Edit::Set(value)) => Some(render(value)),
                    Some(Edit::Clear) => None,
                    None => self.file_value(entry.key).map(|v| render(&v)),
                };
                let default = self.default_for(entry.key);
                let resolved = (set.is_none() && default == "unset")
                    .then(|| self.resolved_hint(entry.key))
                    .flatten();
                Row {
                    entry,
                    set,
                    default,
                    resolved,
                }
            })
            .collect()
    }

    /// The value the file currently sets for a key, if any.
    fn file_value(&self, key: &str) -> Option<toml_edit::Value> {
        let (section, name) = key.split_once('.').expect("registry keys are section.key");
        self.doc.get(section)?.get(name)?.as_value().cloned()
    }

    /// The built-in default, rendered — `unset` for the Option fields that resolve elsewhere.
    fn default_for(&self, key: &str) -> String {
        let (section, name) = key.split_once('.').expect("registry keys are section.key");
        match self.defaults.get(section).and_then(|s| s.get(name)) {
            Some(toml::Value::String(s)) => s.clone(),
            Some(value) => value.to_string(),
            // Not serialized: an `Option` at `None`. The registry doc says what unset means.
            None => "unset".to_owned(),
        }
    }

    /// What an unset key resolves to, through the daemon's own resolution — per-mode policy
    /// defaults, release-relative paths, the mic's mode-dependent switch. Parsed from the
    /// pending state, so flipping `mode` updates every hint that depends on it.
    fn resolved_hint(&self, key: &str) -> Option<String> {
        let params: Params = toml::from_str(&self.rendered()).ok()?;
        let policy = params.policy.resolved();
        let path = |p: Option<std::path::PathBuf>| {
            Some(match p {
                Some(p) => p.display().to_string(),
                None => "disabled".to_owned(),
            })
        };
        let float = |f: f64| Some(f.to_string());
        match key {
            "policy.walk" => path(policy.walk),
            "policy.stand" => path(policy.stand),
            "policy.sitstand" => path(policy.sitstand),
            "policy.ground_pick" => path(policy.ground_pick),
            "policy.kick_left" => path(policy.kick_left),
            "policy.kick_right" => path(policy.kick_right),
            "policy.roulade" => path(policy.roulade),
            "policy.action_scale" => float(policy.action_scale),
            "policy.head_lowpass" => policy.head_lowpass.and_then(float),
            "policy.legs_lowpass" => policy.legs_lowpass.and_then(float),
            "policy.ground_pick_period" => float(policy.ground_pick_period),
            "policy.ground_pick_end_phase" => float(policy.ground_pick_end_phase),
            "policy.ground_pick_action_scale" => float(policy.ground_pick_action_scale),
            "media.bitrate" => Some(params.media.bitrate_resolved().to_string()),
            "audio.pet_detect" => Some(
                params
                    .audio
                    .pet_detect_resolved(params.policy.mode)
                    .to_string(),
            ),
            "audio.pet_model" => path(params.audio.pet_model_resolved()),
            _ => None,
        }
    }

    /// Queue an edit, from the string a user typed or a toggle produced.
    ///
    /// Typing the default (or `unset`, for the optional kinds) clears the override instead of
    /// pinning it — a file full of explicitly-written defaults is the unreadable thing this
    /// tool exists to avoid.
    pub fn edit(&mut self, entry: &'static Entry, input: &str) -> Result<(), String> {
        let input = input.trim();
        let optional = matches!(
            entry.kind,
            Kind::TriBool | Kind::OptionalFloat | Kind::OptionalInteger | Kind::OptionalPath
        );
        if input == self.default_for(entry.key)
            || (optional && (input == "unset" || input.is_empty()))
        {
            self.pending.insert(entry.key, Edit::Clear);
            return Ok(());
        }
        let value: toml_edit::Value = match entry.kind {
            Kind::Bool | Kind::TriBool => match input {
                "true" | "on" | "yes" => true.into(),
                "false" | "off" | "no" => false.into(),
                _ => return Err(format!("{input:?} is not on/off")),
            },
            Kind::Integer | Kind::OptionalInteger => input
                .parse::<i64>()
                .map(Into::into)
                .map_err(|_| format!("{input:?} is not a whole number"))?,
            Kind::Float | Kind::OptionalFloat => input
                .parse::<f64>()
                .map(Into::into)
                .map_err(|_| format!("{input:?} is not a number"))?,
            Kind::Choice(choices) => {
                if !choices.contains(&input) {
                    return Err(format!("{input:?} is not one of {choices:?}"));
                }
                input.into()
            }
            Kind::Text | Kind::OptionalPath => input.into(),
            // A repeating table is not one value with one cursor, so this editor lists it and
            // points at the commands that do manage it. See `Kind::Table`.
            Kind::Table => {
                return Err("edit the one-shot skills with `robotctl policy`".to_owned());
            }
            // Six numbers from a calibration, where a typo is a plausible wrong answer rather
            // than an error. Written by whatever measured them, not typed into an editor.
            Kind::Record(_) => {
                return Err(
                    "camera intrinsics come from a calibration; write them into robotd.toml \
                     directly, or leave them absent to publish the module's design figures"
                        .to_owned(),
                );
            }
            Kind::IntegerList => {
                let mut array = toml_edit::Array::new();
                for word in input.split(',') {
                    let word = word.trim();
                    if word.is_empty() {
                        continue;
                    }
                    let number: i64 = word
                        .parse()
                        .map_err(|_| format!("{word:?} is not a whole number"))?;
                    array.push(number);
                }
                if array.is_empty() {
                    return Err("an empty list — give comma-separated numbers".to_owned());
                }
                array.into()
            }
        };
        self.pending.insert(entry.key, Edit::Set(value));
        Ok(())
    }

    /// The next value a toggle key produces — what SPACE does. `None` for kinds that want
    /// typed input instead.
    pub fn toggled(&self, row: &Row) -> Option<String> {
        match row.entry.kind {
            Kind::Bool => Some(
                if row.effective() == "true" {
                    "false"
                } else {
                    "true"
                }
                .into(),
            ),
            // auto → on → off → auto. `unset` is the auto state.
            Kind::TriBool => Some(match (row.overridden(), row.effective()) {
                (false, _) => "true".into(),
                (true, "true") => "false".into(),
                (true, _) => "unset".into(),
            }),
            Kind::Choice(choices) => {
                let current = row.effective();
                let at = choices.iter().position(|c| *c == current).unwrap_or(0);
                Some(choices[(at + 1) % choices.len()].into())
            }
            _ => None,
        }
    }

    /// The document with every pending edit applied, as text — what save writes.
    ///
    /// Comments and unknown keys survive untouched: `toml_edit` only changes what is set or
    /// removed, and clearing a key removes the key alone, never its section or its comments.
    pub fn rendered(&self) -> String {
        let mut doc = self.doc.clone();
        for (key, edit) in &self.pending {
            let (section, name) = key.split_once('.').expect("section.key");
            match edit {
                Edit::Set(value) => {
                    let table = doc
                        .entry(section)
                        .or_insert(toml_edit::Item::Table(toml_edit::Table::new()));
                    table[name] = toml_edit::Item::Value(value.clone());
                }
                Edit::Clear => {
                    if let Some(table) = doc.get_mut(section).and_then(|i| i.as_table_mut()) {
                        table.remove(name);
                    }
                }
            }
        }
        doc.to_string()
    }

    /// Add or replace one `[[policy.skill]]` entry, by name.
    ///
    /// Separate from [`Self::edit`] because a repeating table is not a key with a value: there
    /// is no cursor position for it, and the editor lists it rather than editing it in place
    /// (`registry::Kind::Table`). This is the writing half, and `robotctl policy` is what calls
    /// it — the same document, the same atomic save, and the same validation through the
    /// daemon's own loader, so the comments and everything else in the file survive.
    ///
    /// By name rather than by appending, so running the same command twice retunes a skill
    /// instead of giving a robot two of it.
    fn put_skill(&mut self, skill: &crate::SkillDef) -> Result<(), String> {
        // Here rather than only in the caller, because a skill can be added from three places
        // and a rule one of them enforces is not a rule. `robot.do` matches the two the daemon
        // drives itself before it looks at the table, so an entry answering to either name is
        // one this file offers and nothing can ever run.
        if crate::DAEMON_OWNED_SKILLS.contains(&skill.name.as_str()) {
            return Err(format!(
                "{} is driven by the robot itself and cannot be a table entry",
                skill.name
            ));
        }
        let mut entry = toml_edit::Table::new();
        entry["name"] = toml_edit::value(skill.name.clone());
        if let Some(path) = &skill.path {
            entry["path"] = toml_edit::value(path.display().to_string());
        }
        entry["duration"] = toml_edit::value(skill.duration);
        if skill.chain {
            entry["chain"] = toml_edit::value(true);
        }
        // Only what differs from a plain zero-command one-shot. A file full of explicit
        // defaults is the unreadable thing this whole editor exists to avoid.
        if skill.command != [0.0; 3] {
            entry["command"] = toml_edit::value(array_of(skill.command));
        }
        if skill.unwind_s > 0.0 {
            entry["unwind"] = toml_edit::value(array_of(skill.unwind));
            entry["unwind_s"] = toml_edit::value(skill.unwind_s);
        }

        let mut overrides = toml_edit::Table::new();
        for (key, value) in [
            ("action_scale", skill.params.action_scale),
            ("gain_ratio", skill.params.gain_ratio),
        ] {
            if let Some(value) = value {
                overrides[key] = toml_edit::value(value);
            }
        }
        if !overrides.is_empty() {
            overrides.set_implicit(false);
            entry["params"] = toml_edit::Item::Table(overrides);
        }

        let policy = self
            .doc
            .entry("policy")
            .or_insert(toml_edit::Item::Table(toml_edit::Table::new()));
        let policy = policy
            .as_table_mut()
            .ok_or_else(|| "[policy] is not a table".to_owned())?;
        let skills = policy
            .entry("skill")
            .or_insert(toml_edit::Item::ArrayOfTables(
                toml_edit::ArrayOfTables::new(),
            ));
        let skills = skills
            .as_array_of_tables_mut()
            .ok_or_else(|| "[[policy.skill]] is not a table array".to_owned())?;

        // By position, so the borrow ends before the push.
        let existing = skills
            .iter()
            .position(|t| t.get("name").and_then(|n| n.as_str()) == Some(skill.name.as_str()));
        match existing {
            Some(at) => *skills.get_mut(at).expect("just found it") = entry,
            None => skills.push(entry),
        }
        Ok(())
    }

    /// Remove a `[[policy.skill]]` entry by name. `false` if there was none.
    fn drop_skill(&mut self, name: &str) -> Result<bool, String> {
        let Some(skills) = self
            .doc
            .get_mut("policy")
            .and_then(|p| p.as_table_mut())
            .and_then(|p| p.get_mut("skill"))
            .and_then(|s| s.as_array_of_tables_mut())
        else {
            return Ok(false);
        };
        let before = skills.len();
        skills.retain(|t| t.get("name").and_then(|n| n.as_str()) != Some(name));
        Ok(skills.len() != before)
    }

    /// Validate the pending edits through the daemon's own gate, then write atomically.
    ///
    /// Validation goes through a real file and [`Params::load`] rather than a bare parse,
    /// because `load` is what `robotd` runs at startup — range checks included. What this tool
    /// writes, the daemon starts on.
    ///
    /// **The pending edits are re-applied to the file as it is now**, under the lock, rather
    /// than to the copy read when this model was loaded. A model can be open for as long as
    /// somebody leaves `robotctl configure` on screen, and `robotd` writes the same file for
    /// `pad.bind` while they do — saving the old document back would silently revert that.
    /// Only the keys this model actually edited are carried across; everything else is
    /// whatever the file says.
    pub fn save(&mut self) -> Result<(), String> {
        let lock = lock(&self.path)?;
        if let Ok(fresh) = std::fs::read_to_string(&self.path)
            && let Ok(mut doc) = fresh.parse::<DocumentMut>()
        {
            Self::migrate_renamed_sections(&mut doc);
            self.doc = doc;
        }
        let text = self.rendered();
        self.write_through(&lock, &text)?;
        // The document on disk is now the rendered one; fold the edits in.
        self.doc = text.parse().expect("just validated");
        for key in self.pending.keys() {
            let key = (*key).to_owned();
            if !self.written.contains(&key) {
                self.written.push(key);
            }
        }
        self.pending.clear();
        Ok(())
    }

    /// Stage, validate through the daemon's own loader, and rename into place.
    ///
    /// Takes the lock as proof rather than acquiring it, so the caller's read-modify-write is
    /// covered by one hold rather than by two that another writer can slip between.
    fn write_through(&self, _lock: &Lock, text: &str) -> Result<(), String> {
        let staged = self.path.with_extension("toml.new");
        let write = |path: &Path| -> std::io::Result<()> {
            let mut file = std::fs::File::create(path)?;
            file.write_all(text.as_bytes())?;
            file.sync_all()
        };
        write(&staged).map_err(|e| writable_hint(&staged, &e))?;
        if let Err(e) = Params::load(&staged, true) {
            let _ = std::fs::remove_file(&staged);
            return Err(format!(
                "refusing to write a config robotd would reject: {e}"
            ));
        }
        std::fs::rename(&staged, &self.path).map_err(|e| writable_hint(&self.path, &e))?;
        // The rename is only durable once the directory entry is. A robot switched off at the
        // wall is the normal case here, not the exceptional one — same discipline as `configd`'s
        // store and the update journal.
        if let Some(dir) = self.path.parent()
            && let Ok(dir) = std::fs::File::open(dir)
        {
            let _ = dir.sync_all();
        }
        Ok(())
    }
}

/// An exclusive hold on the config file, for the duration of one read-modify-write.
type Lock = std::fs::File;

/// Take it.
///
/// **Four writers share `robotd.toml`**: `robotctl configure`, `robotctl policy`, `robotctl pad`,
/// and `robotd` itself, which writes it serving `robot.loadPolicy`, `robot.setSkill` and
/// `pad.bind` from a phone. Two of them staging into the same `robotd.toml.new` at the same
/// moment is a file with one writer's half of the work, renamed into place by the other.
///
/// On a lock file beside the config rather than on the config itself, so the hold outlives the
/// rename that replaces it — the arrangement `configd`'s store uses, for the same reason.
fn lock(path: &Path) -> Result<Lock, String> {
    let lock_path = path.with_extension("lock");
    let file = std::fs::OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(false)
        .open(&lock_path)
        .map_err(|e| writable_hint(&lock_path, &e))?;
    file.lock()
        .map_err(|e| format!("cannot lock {}: {e}", lock_path.display()))?;
    Ok(file)
}

/// Add or replace one `[[policy.skill]]` entry, by name.
///
/// Load, edit and write under one hold of the lock, because a repeating table cannot be queued
/// as a keyed edit and so cannot be rebased the way [`Model::save`] rebases one. `robotctl
/// policy add` and `robot.setSkill` both come through here.
pub fn set_skill(path: &Path, skill: &crate::SkillDef) -> Result<(), String> {
    let lock = lock(path)?;
    let mut model = Model::load(path)?;
    model.put_skill(skill)?;
    let text = model.doc.to_string();
    model.write_through(&lock, &text)
}

/// Take a skill out of the table. `false` if there was none, which is not a failure — it is the
/// answer to "is this robot's own back", and the caller says so differently.
pub fn remove_skill(path: &Path, name: &str) -> Result<bool, String> {
    let lock = lock(path)?;
    let mut model = Model::load(path)?;
    if !model.drop_skill(name)? {
        return Ok(false);
    }
    let text = model.doc.to_string();
    model.write_through(&lock, &text).map(|()| true)
}

/// Three floats as a TOML array.
fn array_of(values: [f64; 3]) -> toml_edit::Array {
    let mut array = toml_edit::Array::new();
    for value in values {
        array.push(value);
    }
    array
}

/// A value as the UI shows it — the data alone. Strings lose their quotes, and everything
/// loses its decor: `to_string` on a `toml_edit` value carries the whitespace and any inline
/// comment along, which is how `50 # do not touch` once ended up in a value cell.
pub fn render(value: &toml_edit::Value) -> String {
    match value {
        toml_edit::Value::String(s) => s.value().clone(),
        toml_edit::Value::Integer(v) => v.value().to_string(),
        toml_edit::Value::Float(v) => v.value().to_string(),
        toml_edit::Value::Boolean(v) => v.value().to_string(),
        toml_edit::Value::Datetime(v) => v.value().to_string(),
        other => {
            let mut bare = other.clone();
            bare.decor_mut().clear();
            bare.to_string().trim().to_owned()
        }
    }
}

/// Sections in registry order, for headers.
pub fn sections() -> Vec<&'static str> {
    let mut out: Vec<&'static str> = Vec::new();
    for entry in REGISTRY {
        let section = entry.key.split_once('.').expect("section.key").0;
        if out.last() != Some(&section) {
            out.push(section);
        }
    }
    out
}

/// Permission errors say what to do about it, because the file is root-owned by design.
///
/// It does not name a command any more. Four callers write this file and `robotd` is one of
/// them — it runs as root, so telling whoever asked over the radio to "run `sudo robotctl
/// configure`" would send them to a terminal to fix something sudo cannot reach.
fn writable_hint(path: &Path, e: &std::io::Error) -> String {
    if e.kind() == std::io::ErrorKind::PermissionDenied {
        format!(
            "cannot write {}: permission denied — it is root-owned, so this needs sudo",
            path.display()
        )
    } else {
        format!("cannot write {}: {e}", path.display())
    }
}

/// The pad's button bindings as the daemon resolves them — config over the defaults.
pub fn pad_bindings(path: &Path) -> Result<crate::PadParams, String> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => String::new(),
        Err(e) => return Err(format!("cannot read {}: {e}", path.display())),
    };
    toml::from_str::<Params>(&text)
        .map(|params| params.pad)
        .map_err(|e| format!("{}: {e}", path.display()))
}

/// Put a skill on a button, or clear it.
///
/// Through the same document and the same validation every other edit uses, so comments survive
/// and nothing lands that `robotd` would refuse to start on. A binding set to the default is
/// *removed* rather than written out, keeping the file a list of decisions — which is also what
/// makes `robotctl configure --list` mean something.
pub fn bind_pad(path: &Path, button: &str, skill: &str) -> Result<(), String> {
    let mut model = Model::load(path)?;
    let key = format!("pad.{button}");
    let entry = REGISTRY
        .iter()
        .find(|e| e.key == key)
        .ok_or_else(|| format!("{key} is not a key robotd knows"))?;
    model.edit(entry, skill)?;
    model.save()
}

/// Record which file a policy slot runs — or clear the key, which is what a reset is.
///
/// **The daemon's half of `robot.loadPolicy`.** A slot is `[policy] <slot>` in the config file
/// and nothing else, so a load that did not write here would be gone at the next restart; the
/// method persists, and this is where. `robotctl policy load` used to write it from the other
/// end, which made the same words mean different things depending on who asked.
///
/// `Ok(true)` when the file changed. `Ok(false)` is a request the file already said — worth
/// telling apart, because a slot the loop is already running *and* the file already names is a
/// command with nothing left to do.
///
/// Every slot is written in one document and one save, so a whole-robot reset cannot leave four
/// keys cleared and three not.
pub fn set_slots(path: &Path, slots: &[crate::Slot], to: Option<&Path>) -> Result<bool, String> {
    let mut model = Model::load(path)?;
    let before = model.rendered();
    let value = match to {
        Some(path) => path.display().to_string(),
        None => "unset".to_owned(),
    };
    for slot in slots {
        let key = slot.config_key();
        let entry = REGISTRY
            .iter()
            .find(|e| e.key == key)
            .ok_or_else(|| format!("{key} is not a key robotd knows"))?;
        model
            .edit(entry, &value)
            .map_err(|e| format!("{key}: {e}"))?;
    }
    // The rendered document rather than the keys: it is what `save` would write, so it cannot
    // disagree with itself about whether writing would change anything.
    if model.rendered() == before {
        return Ok(false);
    }
    model.save().map(|()| true)
}
