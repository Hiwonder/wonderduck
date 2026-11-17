//! The tail of one daemon's journal, for a client that has no shell on the robot.
//!
//! [`units`](crate::units) answers "is `robotd` running"; this answers the question that follows a
//! `failed` — *why*. Until it existed the only answer was `journalctl` over ssh, which needs a
//! network the robot may not have and an address a phone cannot reach. The robot whose journal
//! somebody actually needs is the one whose wifi never came up.
//!
//! **`configd` reads it because reading the journal needs privilege**, the same reason
//! [`units`](crate::units) is here: it runs as root (see `systemd/configd.service`), and
//! `robotctl` does not. `robotctl logs` asks for the same tail over the local socket, so an operator
//! with a shell gets it without `sudo`. For anything more — `-g`, `--since`, `-f` — they use
//! `journalctl`; this call is a tail and nothing else.
//!
//! ## Why it shells out to `journalctl`
//!
//! Reading the journal in-process means linking `libsystemd`, which means vendored C in a
//! cross-compiled build for one read-only query. `journalctl` is on every board this ships to,
//! it is already how the updater talks to systemd (`updater::engine` spawns `systemctl`), and
//! the sandbox in this service's unit permits it: `ProtectSystem=strict` leaves `/usr/bin`
//! readable, and root can read the journal without any capability.
//!
//! ## Why it takes a unit name and not arguments
//!
//! This call is reachable from a phone in radio range (`btd::route`). `journalctl` arguments are
//! not a language to hand such a peer — `-D` reads another journal directory, `_PID=` matches
//! anything on the box — so the unit is chosen from a fixed list and everything else is refused
//! by name. What that costs is the searching a person with a shell would do. What it buys is that
//! the worst any client can ask for is the tail of a daemon this project ships.

use duck_ipc_proto as proto;

/// Units that are not part of a daemon release but whose journals answer *our* failures.
///
/// Both earn their place by being the thing that is broken when the robot cannot be reached at
/// all: BlueZ, when no phone can see the robot, and NetworkManager, when wifi never came up.
/// Neither is ours, which is exactly why their logs are otherwise unreachable — nothing in
/// `system.services` reports on them either.
pub const ALSO_LOGGABLE: [&str; 2] = ["bluetooth.service", "NetworkManager.service"];

/// Every unit whose journal this service will read, in the order a reader wants them.
///
/// Derived from [`crate::units::MANAGED`] rather than repeated, so a unit added to a release
/// becomes readable here without anyone remembering this file — the mistake that list's own doc
/// records having made once.
pub fn loggable() -> impl Iterator<Item = &'static str> {
    crate::units::MANAGED.into_iter().chain(ALSO_LOGGABLE)
}

/// Longest single line kept, in bytes.
//
// Off Linux there is no `read` to call these, and only the tests do — the same shape
// `units::describe` takes, where the Linux half is the real one and the other half exists so a
// laptop can still build and test the crate.
#[cfg(any(target_os = "linux", test))]
///
/// journald accepts a stdout line up to its own `LineMax` (48 KiB by default), so one daemon
/// logging a serialised blob could otherwise fill a whole reply with one entry and push out the
/// hundred lines around it — which is the opposite of what a tail is for. Truncated with an
/// ellipsis, so a reader can see that the line went on.
const MAX_LINE_BYTES: usize = 2 * 1024;

/// Resolve a caller's unit name against [`loggable`].
///
/// `robotd` and `robotd.service` both work: a caller typing the daemon's name should not have to
/// know that systemd spells it with a suffix. The answer is the canonical name, so the reply says
/// what was actually read.
pub fn resolve(name: &str) -> Option<&'static str> {
    let wanted = name.trim();
    let wanted = wanted.strip_suffix(".service").unwrap_or(wanted);
    // Case-insensitive for `NetworkManager` alone, really: nobody types that capitalisation from
    // memory, and refusing `networkmanager` would be a puzzle rather than a guard.
    loggable().find(|unit| {
        unit.strip_suffix(".service")
            .unwrap_or(unit)
            .eq_ignore_ascii_case(wanted)
    })
}

/// What to tell a caller that named something else. Names the real list, the way `pad.bind`
/// refuses an unknown skill: a typo should come back as the answer, not as a shrug.
pub fn refusal(name: &str) -> proto::Error {
    proto::Error::new(
        proto::code::INVALID_PARAMS,
        format!(
            "no journal here for {name:?}; readable units are {}",
            loggable().collect::<Vec<_>>().join(", ")
        ),
    )
}

/// Read the tail of a unit's journal.
///
/// `unit` must have come from [`resolve`] — that is what keeps a caller's string out of
/// `journalctl`'s argument list.
#[cfg(target_os = "linux")]
pub async fn read(
    unit: &'static str,
    lines: usize,
    boot: i32,
) -> Result<proto::LogsResult, String> {
    let lines = lines.clamp(1, proto::MAX_LOG_LINES);

    let output = tokio::process::Command::new("journalctl")
        .arg("--unit")
        .arg(unit)
        // `--boot=-1` as one argument rather than `-b -1`: an offset that starts with a dash is
        // otherwise ambiguous with an option, and which of the two `journalctl` decides it is has
        // changed between versions.
        .arg(format!("--boot={boot}"))
        .arg("--lines")
        .arg(lines.to_string())
        .arg("--no-pager")
        // The timestamp is the point — a log line with no time answers nothing — and the
        // `robotd[1234]:` prefix carries the PID, which is how a restart shows up in a tail.
        // The hostname is the one field that is the same on every line and known already.
        .arg("--output=short-iso")
        .arg("--no-hostname")
        .output()
        .await
        .map_err(|e| format!("could not run journalctl: {e}"))?;

    if !output.status.success() {
        // journalctl's own words, which for the case a caller will actually hit — a boot that is
        // not in the journal — are already the right answer ("Data from the specified boot (-1) is
        // not available"). Ours would be a worse paraphrase.
        let stderr = String::from_utf8_lossy(&output.stderr);
        let detail = stderr.lines().next().unwrap_or("no output").trim();
        return Err(format!("journalctl refused: {detail}"));
    }

    Ok(assemble(unit, &String::from_utf8_lossy(&output.stdout)))
}

/// Off the board there is no journal to read, and inventing one would make a laptop look like a
/// robot. An error rather than an empty answer, for the same reason [`crate::units::describe`]
/// reports `Unknown` off Linux: "there is nothing here to read" and "this daemon said nothing"
/// are different answers.
#[cfg(not(target_os = "linux"))]
pub async fn read(
    unit: &'static str,
    _lines: usize,
    _boot: i32,
) -> Result<proto::LogsResult, String> {
    Err(format!(
        "no systemd journal on this host, so nothing to read for {unit}"
    ))
}

/// The pid a line was written by, if the line is the daemon's own.
///
/// `short-iso` puts the writer in the second field, as `robotd[3227]:` — or as `systemd[1]:` for
/// the lines systemd writes *about* the unit, which `journalctl -u` returns alongside it. Those
/// are the reason this checks the identifier: `Stopping`, `Stopped` and `Started` all come from
/// pid 1, so a bare "the pid changed" would announce a new process three times per restart and
/// name the wrong one.
#[cfg(any(target_os = "linux", test))]
fn pid_of(line: &str, service: &str) -> Option<u32> {
    let field = line.split_whitespace().nth(1)?.strip_suffix(':')?;
    let (identifier, pid) = field.strip_suffix(']')?.split_once('[')?;
    if identifier != service {
        return None;
    }
    pid.parse().ok()
}

/// Turn `journalctl`'s output into a reply that fits the wire.
///
/// Split out from [`read`] so the trimming is testable without a journal, which is the half with
/// the decisions in it: **oldest lines go first**, because the newest are the ones somebody asked
/// for, and the budget is measured on the serialised form rather than estimated, since JSON
/// escaping is what makes an estimate wrong.
#[cfg(any(target_os = "linux", test))]
fn assemble(unit: &'static str, stdout: &str) -> proto::LogsResult {
    let service = crate::units::service_of(unit);
    let mut lines: Vec<String> = Vec::new();
    let mut writer: Option<u32> = None;

    for line in stdout.lines().map(str::trim_end).filter(|l| !l.is_empty()) {
        if let Some(pid) = pid_of(line, service) {
            // The tail a person reads spans restarts without saying so: a release is installed,
            // every daemon restarts, and forty lines carry two different builds with nothing
            // between them. systemd's own `Started …` line is in there, but it is one line among
            // forty identical warnings and it reads as just another one.
            //
            // `-- … --` is `journalctl`'s own shape for a line it inserted rather than recorded
            // (`-- Reboot --`, `-- No entries --`), so it is the form a reader already knows not
            // to attribute to the daemon.
            if writer.is_some_and(|previous| previous != pid) {
                lines.push(format!("-- new {service} process, pid {pid} --"));
            }
            writer = Some(pid);
        }
        lines.push(truncate_line(line));
    }

    let mut truncated = false;
    while serde_json::to_string(&lines).map_or(0, |json| json.len()) > proto::MAX_LOG_BYTES
        && !lines.is_empty()
    {
        lines.remove(0);
        truncated = true;
    }

    proto::LogsResult {
        unit: unit.to_owned(),
        lines,
        truncated,
    }
}

/// One line, bounded. Cuts on a character boundary, since the reply has to be valid UTF-8.
#[cfg(any(target_os = "linux", test))]
fn truncate_line(line: &str) -> String {
    if line.len() <= MAX_LINE_BYTES {
        return line.to_owned();
    }
    let mut end = MAX_LINE_BYTES;
    while end > 0 && !line.is_char_boundary(end) {
        end -= 1;
    }
    format!("{}…", &line[..end])
}
