//! Concise stderr and complete diagnostic files, with IO off the control thread.
//!
//! Bounded queues never wait for a sink. Repeated bus faults are summarized by the workers.
use std::collections::BTreeMap;
use std::fmt;
use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError, SyncSender, TrySendError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use tracing::{field::{Field, Visit}, Event, Subscriber};
use tracing_subscriber::fmt::MakeWriter;
use tracing_subscriber::layer::{Context, Layer};
use tracing_subscriber::{filter::filter_fn, prelude::*, EnvFilter};

const FILE_BYTES: u64 = 10 * 1024 * 1024;
const FILE_BACKUPS: usize = 3;
const BUS_SUMMARY_WAIT: Duration = Duration::from_millis(100);
const QUEUE_CAPACITY: usize = 1024;
const REPEAT_INTERVAL: Duration = Duration::from_secs(1);

const READ_LATE: &str = "HX control read exceeded cycle deadline; awaiting replies";
const READ_SUCCEEDED_LATE: &str = "bus read completed after cycle deadline";
const READ_FAILED: &str = "bus read failed";
const FAULT_LOCATED: &str = "HX control read fault";
const FAULT_UNKNOWN: &str = "HX control read location unknown";

enum Message {
    Record(Vec<u8>),
    Captured(CapturedEvent),
    Finish,
}

#[derive(Clone)]
pub(crate) struct LogOutput {
    sender: SyncSender<Message>,
    dropped: Arc<AtomicU64>,
}

impl LogOutput {
    fn enqueue(&self, message: Message) {
        match self.sender.try_send(message) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => { self.dropped.fetch_add(1, Ordering::Relaxed); }
            Err(TrySendError::Disconnected(_)) => {
                self.dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

pub(crate) struct EventWriter {
    output: LogOutput,
    bytes: Vec<u8>,
}

pub(crate) struct Guard {
    output: LogOutput,
    worker: Option<JoinHandle<io::Result<()>>>,
}

pub(crate) struct LoggingGuard {
    _terminal: Guard,
    _file: Guard,
}

pub(crate) fn init(path: &Path) -> io::Result<LoggingGuard> {
    let file = RotatingFile::open(path, FILE_BYTES, FILE_BACKUPS)?;
    let (file, file_guard) = start(file)?;
    let (terminal, terminal_guard) = start(io::stderr())?;
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    subscriber(terminal, file, filter).try_init().map_err(io::Error::other)?;
    Ok(LoggingGuard { _terminal: terminal_guard, _file: file_guard })
}

fn subscriber(terminal: LogOutput, file: LogOutput, filter: EnvFilter)
    -> impl Subscriber + Send + Sync
{
    tracing_subscriber::registry()
        .with(CaptureLayer { output: terminal, console: true }
            .with_filter(filter))
        .with(CaptureLayer { output: file, console: false }
            // Preserve bus evidence and timing summaries without per-tick DEBUG traffic.
            .with_filter(filter_fn(|meta| *meta.level() <= tracing::Level::INFO
                || (*meta.level() == tracing::Level::DEBUG
                    && (meta.target().starts_with("duck_control::hx")
                        || meta.target() == "robotd::control_timing")))))
}

fn start<W: Write + Send + 'static>(mut writer: W) -> io::Result<(LogOutput, Guard)> {
    let (sender, input) = mpsc::sync_channel(QUEUE_CAPACITY);
    let output = LogOutput { sender, dropped: Arc::new(AtomicU64::new(0)) };
    let dropped = Arc::clone(&output.dropped);
    let worker = thread::Builder::new().name("log-output".into()).spawn(move || {
        let mut pending = None::<BusSummary>;
        let mut repeats = RepeatGate::default();
        loop {
            let lost = dropped.swap(0, Ordering::Relaxed);
            if lost > 0 { writeln!(writer, "WARN log queue overflow dropped={lost}")?; }
            for event in repeats.due(false) {
                write_captured(&mut writer, &mut pending, event)?;
            }
            if pending.as_ref().and_then(|summary| summary.deadline).is_some_and(|deadline| deadline <= Instant::now()) {
                flush_summary(&mut writer, &mut pending)?;
            }
            let deadline = pending.as_ref().and_then(|summary| summary.deadline)
                .into_iter().chain(repeats.deadline()).min();
            let message = match deadline {
                Some(deadline) => input.recv_timeout(deadline.saturating_duration_since(Instant::now())),
                None => input.recv().map_err(|_| RecvTimeoutError::Disconnected),
            };
            match message {
                Ok(Message::Record(bytes)) => writer.write_all(&bytes)?,
                Ok(Message::Captured(event)) => {
                    for event in repeats.observe(event) {
                        write_captured(&mut writer, &mut pending, event)?;
                    }
                }
                Ok(Message::Finish) | Err(RecvTimeoutError::Disconnected) => {
                    for event in repeats.due(true) {
                        write_captured(&mut writer, &mut pending, event)?;
                    }
                    flush_summary(&mut writer, &mut pending)?;
                    break;
                }
                Err(RecvTimeoutError::Timeout) => flush_summary(&mut writer, &mut pending)?,
            }
        }
        writer.flush()
    })?;
    Ok((output.clone(), Guard { output, worker: Some(worker) }))
}

impl<'a> MakeWriter<'a> for LogOutput {
    type Writer = EventWriter;

    fn make_writer(&'a self) -> Self::Writer {
        EventWriter { output: self.clone(), bytes: Vec::with_capacity(512) }
    }
}

impl Write for EventWriter {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        self.bytes.extend_from_slice(bytes);
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> { Ok(()) }
}

impl Drop for EventWriter {
    fn drop(&mut self) {
        let bytes = std::mem::take(&mut self.bytes);
        if !bytes.is_empty() { self.output.enqueue(Message::Record(bytes)); }
    }
}

impl Drop for Guard {
    fn drop(&mut self) {
        // Only shutdown may wait for the output worker; producers always use try_send.
        let _ = self.output.sender.send(Message::Finish);
        if let Some(worker) = self.worker.take() {
            match worker.join() {
                Ok(Ok(())) => {}
                Ok(Err(error)) => eprintln!("log-output write failed: {error}"),
                Err(_) => eprintln!("log-output worker panicked"),
            }
        }
    }
}

struct RotatingFile {
    path: PathBuf,
    file: File,
    bytes: u64,
    limit: u64,
    backups: usize,
}

impl RotatingFile {
    fn open(path: &Path, limit: u64, backups: usize) -> io::Result<Self> {
        let file = Self::append(path)?;
        let bytes = file.metadata()?.len();
        Ok(Self { path: path.into(), file, bytes, limit, backups })
    }

    fn append(path: &Path) -> io::Result<File> {
        let mut options = OpenOptions::new();
        options.create(true).append(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            options.mode(0o600);
        }
        options.open(path)
    }

    fn backup(&self, index: usize) -> PathBuf {
        let mut name = self.path.as_os_str().to_os_string();
        name.push(format!(".{index}"));
        name.into()
    }

    fn rotate(&mut self) -> io::Result<()> {
        self.file.flush()?;
        for index in (1..self.backups).rev() {
            match fs::rename(self.backup(index), self.backup(index + 1)) {
                Ok(()) => {}
                Err(error) if error.kind() == io::ErrorKind::NotFound => {}
                Err(error) => return Err(error),
            }
        }
        fs::rename(&self.path, self.backup(1))?;
        self.file = Self::append(&self.path)?;
        self.bytes = 0;
        Ok(())
    }
}

impl Write for RotatingFile {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        // The worker passes one complete event, so rotation never splits a record.
        if self.bytes != 0 && self.bytes.saturating_add(bytes.len() as u64) > self.limit {
            self.rotate()?;
        }
        self.file.write_all(bytes)?;
        self.bytes += bytes.len() as u64;
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> { self.file.flush() }
}

struct CaptureLayer { output: LogOutput, console: bool }

impl<S: Subscriber> Layer<S> for CaptureLayer {
    fn on_event(&self, event: &Event<'_>, _: Context<'_, S>) {
        let mut fields = CapturedFields::default();
        event.record(&mut fields);
        let millis = SystemTime::now().duration_since(UNIX_EPOCH)
            .expect("system clock predates the Unix epoch").as_millis();
        self.output.enqueue(Message::Captured(CapturedEvent {
            millis, level: *event.metadata().level(), target: event.metadata().target(),
            console: self.console, fields: fields.0,
        }));
    }
}

#[derive(Clone)]
enum Value {
    String(String), Debug(String), Float(f64), Unsigned(u64), Signed(i64), Bool(bool),
    Failure(Arc<duck_control::read_diagnostic::ReadFailure>),
}

impl Value {
    fn text(&self, field: &str) -> String {
        match self {
            Self::String(value) | Self::Debug(value) => value.clone(),
            Self::Float(value) if field == "gravity_z" || field.ends_with("_ms") => format!("{value:.2}"),
            Self::Float(value) => value.to_string(),
            Self::Unsigned(value) => value.to_string(),
            Self::Signed(value) => value.to_string(),
            Self::Bool(value) => value.to_string(),
            Self::Failure(value) => format!("bus transaction failed: {value}"),
        }
    }
}

#[derive(Default)]
struct CapturedFields(Vec<(&'static str, Value)>);

impl Visit for CapturedFields {
    fn record_str(&mut self, field: &Field, value: &str) { self.0.push((field.name(), Value::String(value.into()))); }
    fn record_f64(&mut self, field: &Field, value: f64) { self.0.push((field.name(), Value::Float(value))); }
    fn record_u64(&mut self, field: &Field, value: u64) { self.0.push((field.name(), Value::Unsigned(value))); }
    fn record_i64(&mut self, field: &Field, value: i64) { self.0.push((field.name(), Value::Signed(value))); }
    fn record_bool(&mut self, field: &Field, value: bool) { self.0.push((field.name(), Value::Bool(value))); }
    fn record_debug(&mut self, field: &Field, value: &dyn fmt::Debug) {
        self.0.push((field.name(), Value::Debug(format!("{value:?}"))));
    }
    fn record_error(&mut self, field: &Field, error: &(dyn std::error::Error + 'static)) {
        let value = match error.downcast_ref::<duck_control::IoError>() {
            Some(duck_control::IoError::ReadFailure(failure)) => Value::Failure(Arc::clone(failure)),
            _ => Value::String(error.to_string()),
        };
        self.0.push((field.name(), value));
    }
}

#[derive(Clone)]
struct CapturedEvent {
    millis: u128, level: tracing::Level, target: &'static str, console: bool,
    fields: Vec<(&'static str, Value)>,
}

impl CapturedEvent {
    fn message(&self) -> String {
        self.fields.iter().find(|(name, _)| *name == "message")
            .map(|(_, value)| value.text("message")).unwrap_or_default()
    }
    fn console(&self) -> ConsoleEvent {
        ConsoleEvent {
            millis: self.millis, level: self.level,
            fields: ConsoleFields {
                message: self.message(),
                values: self.fields.iter().filter(|(name, _)| *name != "message")
                    .map(|(name, value)| (*name, value.text(name))).collect(),
            },
        }
    }
    fn file(&self) -> Vec<u8> {
        let seconds = (self.millis / 1000) as libc::time_t;
        let mut calendar = std::mem::MaybeUninit::<libc::tm>::uninit();
        // The capture timestamp is owned; UTC conversion happens only in the output worker.
        let converted = unsafe { libc::gmtime_r(&seconds, calendar.as_mut_ptr()) };
        assert!(!converted.is_null(), "log timestamp is outside the UTC calendar range");
        let calendar = unsafe { calendar.assume_init() };
        let mut text = format!("{:04}-{:02}-{:02}T{:02}:{:02}:{:02}.{:03}Z {:>5} {}: {}",
            calendar.tm_year + 1900, calendar.tm_mon + 1, calendar.tm_mday,
            calendar.tm_hour, calendar.tm_min, calendar.tm_sec, self.millis % 1000,
            self.level, self.target, self.message());
        use std::fmt::Write as _;
        for (name, value) in &self.fields {
            if *name == "message" { continue; }
            match value {
                Value::String(value) => write!(text, " {name}={value:?}").unwrap(),
                Value::Float(value) if !name.ends_with("_ms") => write!(text, " {name}={value}").unwrap(),
                Value::Failure(failure) => write!(text, " {name}={:?} {} tx={:02x?}",
                    value.text(name), failure.wire_prefix(), failure.tx).unwrap(),
                _ => write!(text, " {name}={}", value.text(name)).unwrap(),
            }
        }
        text.push('\n');
        text.into_bytes()
    }
    fn repeat_key(&self) -> Option<String> {
        let message = self.message();
        if !matches!(message.as_str(), READ_LATE | READ_FAILED | READ_SUCCEEDED_LATE | "HX transaction failure detail" | "HX control read completed after cycle deadline") {
            return None;
        }
        let key = self.fields.iter().filter(|(name, _)| matches!(*name, "error" | "missing_ids"))
            .map(|(name, value)| match value {
                Value::Failure(failure) => failure.signature(),
                _ => value.text(name),
            }).collect::<Vec<_>>().join("|");
        Some(format!("{}:{message}:{key}", self.target))
    }
}

#[derive(Default)]
struct RepeatGate { entries: Vec<(String, Instant, u64, CapturedEvent)> }

impl RepeatGate {
    fn observe(&mut self, event: CapturedEvent) -> Vec<CapturedEvent> {
        if matches!(event.message().as_str(), READ_SUCCEEDED_LATE | "bus reads recovered after a run" | "control loop stopped" | "shutting down") {
            let mut output = self.due(true);
            self.entries.clear();
            output.push(event);
            return output;
        }
        let Some(key) = event.repeat_key() else { return vec![event] };
        if let Some(entry) = self.entries.iter_mut().find(|entry| entry.0 == key) {
            entry.2 += 1;
            entry.3 = event;
            return Vec::new();
        }
        let mut output = Vec::new();
        if self.entries.len() >= 16 {
            let (_, _, count, mut previous) = self.entries.remove(0);
            if count > 0 {
                previous.fields.push(("repeated", Value::Unsigned(count)));
                output.push(previous);
            }
        }
        self.entries.push((key, Instant::now(), 0, event.clone()));
        output.push(event);
        output
    }
    fn deadline(&self) -> Option<Instant> {
        self.entries.iter().filter(|entry| entry.2 > 0)
            .map(|entry| entry.1 + REPEAT_INTERVAL).min()
    }
    fn due(&mut self, all: bool) -> Vec<CapturedEvent> {
        let mut output = Vec::new();
        for (_, emitted, count, event) in &mut self.entries {
            if *count > 0 && (all || emitted.elapsed() >= REPEAT_INTERVAL) {
                let mut summary = event.clone();
                summary.fields.push(("repeated", Value::Unsigned(*count)));
                output.push(summary);
                *count = 0;
                *emitted = Instant::now();
            }
        }
        output
    }
}

fn write_captured(writer: &mut impl Write, pending: &mut Option<BusSummary>, event: CapturedEvent) -> io::Result<()> {
    if !event.console { return writer.write_all(&event.file()); }
    let event = event.console();
    if event.level == tracing::Level::INFO
        && event.fields.message == "servo voltage window summary"
        && event.fields.values.get("missing_ids").is_some_and(|ids| ids == "[]")
    {
        return Ok(());
    }
    match event.fields.message.as_str() {
        READ_LATE => {
            flush_summary(writer, pending)?;
            *pending = Some(BusSummary::new(Some(event), None));
        }
        READ_FAILED => {
            if pending.as_ref().is_some_and(|summary| summary.failure.is_some()) { flush_summary(writer, pending)?; }
            let late = pending.take().and_then(|summary| summary.late);
            *pending = Some(BusSummary::new(late, Some(event)));
        }
        FAULT_LOCATED | FAULT_UNKNOWN => {
            if let Some(mut summary) = pending.take() {
                if summary.failure.is_some() {
                    summary.diagnostic = Some(event);
                    return summary.write_to(writer);
                }
                *pending = Some(summary);
            }
            if event.fields.message == FAULT_LOCATED { event.write_to(writer)?; }
        }
        READ_SUCCEEDED_LATE => {
            if pending.as_ref().is_some_and(|summary| summary.failure.is_some()) { flush_summary(writer, pending)?; }
            *pending = None;
            event.write_to(writer)?;
        }
        "bus reads recovered after a run" | "control loop stopped" | "shutting down" => {
            flush_summary(writer, pending)?;
            event.write_to(writer)?;
        }
        _ => event.write_to(writer)?,
    }
    Ok(())
}

struct ConsoleEvent {
    millis: u128,
    level: tracing::Level,
    fields: ConsoleFields,
}

impl ConsoleEvent {
    fn write_to(&self, writer: &mut impl Write) -> io::Result<()> {
        let mut bytes = Vec::with_capacity(256);
        let seconds = self.millis / 1000;
        let message = if self.fields.message == "servo voltage window summary" {
            self.fields.values.get("voltage_summary").ok_or_else(||
                io::Error::new(io::ErrorKind::InvalidData, "voltage summary field missing"))?.as_str()
        } else {
            brief_message(&self.fields.message)
        };
        write!(&mut bytes, "{:02}:{:02}:{:02}.{:02} {:>5} {}",
            seconds / 3600 % 24, seconds / 60 % 60, seconds % 60, self.millis / 10 % 100,
            self.level, message)?;
        for name in console_fields(&self.fields.message) {
            if let Some(value) = self.fields.values.get(name) {
                if self.fields.message == "servo voltage window summary" && *name == "missing_ids" && value == "[]" {
                    continue;
                }
                let label = match *name {
                    "sensor_reported_healthy" => "imu_ok",
                    "host_imu_miss" => "imu_miss",
                    "host_servo_miss" => "servo_miss",
                    "host_missing_ids" | "missing_ids" => "missing",
                    "blind_ms" => "gap_ms",
                    "error" => "reason",
                    _ => name,
                };
                if *name == "error" {
                    // IoError puts packet evidence after semicolons. Keep the reason here.
                    let reason = value.split_once(';').map_or(value.as_str(), |(reason, _)| reason);
                    write!(&mut bytes, " {label}={reason:?}")?;
                } else {
                    write!(&mut bytes, " {label}={value}")?;
                }
            }
        }
        if let Some(repeated) = self.fields.values.get("repeated") {
            write!(&mut bytes, " repeated={repeated}")?;
        }
        writeln!(&mut bytes)?;
        writer.write_all(&bytes)
    }
}

struct BusSummary {
    late: Option<ConsoleEvent>,
    failure: Option<ConsoleEvent>,
    diagnostic: Option<ConsoleEvent>,
    deadline: Option<Instant>,
}

impl BusSummary {
    fn new(late: Option<ConsoleEvent>, failure: Option<ConsoleEvent>) -> Self {
        let deadline = failure.as_ref().map(|_| Instant::now() + BUS_SUMMARY_WAIT);
        Self { late, failure, diagnostic: None, deadline }
    }

    fn write_to(self, writer: &mut impl Write) -> io::Result<()> {
        let Some(mut failure) = self.failure else {
            if let Some(late) = self.late {
                late.write_to(writer)?;
            }
            return Ok(());
        };
        if let Some(late) = self.late {
            if let Some(value) = late.fields.values.get("missing_ids") {
                failure.fields.values.insert("missing_ids", value.clone());
            }
        }
        failure.fields.values.insert("location", "unknown".into());
        if let Some(diagnostic) = self.diagnostic {
            if let Some(value) = diagnostic.fields.values.get("location") {
                failure.fields.values.insert("location", value.clone());
            }
            if let Some(value) = diagnostic.fields.values.get("host_missing_ids") {
                failure.fields.values.insert("missing_ids", value.clone());
            }
        }
        failure.write_to(writer)
    }
}

fn flush_summary(writer: &mut impl Write, pending: &mut Option<BusSummary>) -> io::Result<()> {
    if let Some(summary) = pending.take() {
        summary.write_to(writer)?;
    }
    Ok(())
}

fn brief_message(message: &str) -> &str {
    match message {
        "joint HOME loaded from policy metadata" => "joint HOME loaded",
        "HX torque state verified" => "torque verified",
        "HX control read exceeded cycle deadline; awaiting replies" => "bus read late; still waiting",
        READ_SUCCEEDED_LATE => "bus read late; succeeded",
        "HX control read location unknown" => "bus fault location unknown",
        "HX control read fault" => "bus fault located",
        "bus reads recovered after a run" => "bus reads recovered",
        "the fall verdict changed" => "fall state changed",
        "policy IMU input stale" => "IMU input old",
        "policy IMU payload unchanged; sensor freshness unknown" => "IMU payload unchanged; freshness unknown",
        "policy IMU input recovered" => "IMU input fresh again",
        "IMU diagnostic baseline contains historical errors" => "IMU baseline: historical errors",
        "IMU diagnostic baseline contains faults" => "IMU baseline: faults recorded",
        "no voice bank — sounds are skipped (run `sounds ensure-bank`)" => "voice bank missing; sounds skipped",
        _ => message,
    }
}

fn console_fields(message: &str) -> &'static [&'static str] {
    match message {
        "MCU accel calibration active" | "MCU accel calibration not installed" |
        "MCU accel calibration bypassed" | "MCU acceleration calibration runtime switch" |
        "MCU accel calibration storage fault; identity correction active" => &["enabled", "generation"],
        "cannot confirm MCU accel calibration; state unknown" => &["error"],
        "detailed log" => &["path"],
        "servo alarm reported; valid feedback retained" => &["id", "status", "alarm"],
        "servo voltage window summary" => &["missing_ids"],
        "startup device check failed; waiting" => &["reason", "error", "attempt"],
        "cannot ping the servos; waiting" => &["error"],
        "starting" => &["build", "pid"],
        "policy loaded" => &["mode"],
        "control loop running" => &["joints", "hz", "driving"],
        "policy IMU host median filters" => &["gyro_enabled", "gravity_enabled"],
        "policy IMU input stale" => &["source", "reason", "age_ms", "limit_ms", "run"],
        "policy IMU payload unchanged; sensor freshness unknown" => &["source", "payload_unchanged_run"],
        "policy IMU input recovered" => &["source", "age_ms"],
        READ_FAILED => &["consecutive", "error", "read_ms", "missing_ids", "location"],
        "MCU UART receive fault" => &["sensor_reported_healthy", "host_imu_miss", "host_servo_miss"],
        "HX control read fault" => &["location", "host_missing_ids"],
        "HX control read location unknown" => &["host_missing_ids"],
        READ_SUCCEEDED_LATE => &["read_ms", "over_ms"],
        "IMU diagnostic baseline contains historical errors" |
        "IMU diagnostic baseline contains faults" => &["ready", "stale", "initialized"],
        "MCU sensor sampling fault" => &["ready", "stale", "last_error_stage"],
        "IMU diagnostic fault" => &["ready", "stale", "last_error_stage"],
        "bus reads recovered after a run" => &["failures", "blind_ms"],
        _ => &["id", "on", "fallen", "gravity_z", "enabled", "corrected", "consecutive",
            "missing_ids", "elapsed_ms", "skill", "mode", "reason", "stage", "mean_ms",
            "p95_ms", "max_ms", "duration", "HOME_RAMP"],
    }
}

#[derive(Default)]
struct ConsoleFields {
    message: String,
    values: BTreeMap<&'static str, String>,
}
