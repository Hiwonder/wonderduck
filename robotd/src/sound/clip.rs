use std::os::unix::fs::OpenOptionsExt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};

use super::{Child, Duration, Path, Ride, Sound, Stdio, signal_player, wait_bounded};


/// Keep `SIGPIPE` off a writer thread.
///
/// `robotd` restores its default disposition at startup; blocking it here makes writes to a
/// dead player fail instead of terminating the daemon.
pub(super) fn block_sigpipe() {
    // SAFETY: masking one signal on the calling thread; nothing else observes the mask.
    unsafe {
        let mut set: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut set);
        libc::sigaddset(&mut set, libc::SIGPIPE);
        libc::pthread_sigmask(libc::SIG_BLOCK, &set, std::ptr::null_mut());
    }
}

pub(super) struct Clip {
    name: Arc<str>,
    looping: bool,
    pub(super) cancel: Arc<AtomicBool>,
    state: ClipState,
}

enum ClipState {
    Preparing(mpsc::Receiver<std::io::Result<PreparedClip>>),
    Playing,
    Failed,
}

enum PreparedClip {
    Once(std::fs::File),
    Loop {
        reader: hound::WavReader<std::io::BufReader<std::fs::File>>,
        format: ClipFormat,
    },
}

#[derive(Debug, Clone, Copy)]
struct ClipFormat {
    spec: hound::WavSpec,
    sample_format: &'static str,
}

impl Sound {
    fn prepare_clip(&mut self, name: Arc<str>, looping: bool) {
        let dir = self.clips_dir.clone();
        let worker_name = Arc::clone(&name);
        let cancel = Arc::new(AtomicBool::new(false));
        let worker_cancel = Arc::clone(&cancel);
        let (sender, receiver) = mpsc::channel();
        let spawned = std::thread::Builder::new().name("clip-prepare".into()).spawn(move || {
            let result = open_clip(&dir, &worker_name, looping);
            if !worker_cancel.load(Ordering::Relaxed) {
                let _ = sender.send(result);
            }
        });
        let state = match spawned {
            Ok(_) => ClipState::Preparing(receiver),
            Err(error) => {
                tracing::warn!(%name, %error, "cannot start clip preparation");
                ClipState::Failed
            }
        };
        self.clip = Some(Clip { name, looping, cancel, state });
    }

    pub fn play_clip(&mut self, name: Arc<str>) {
        if self.ride != Ride::Off {
            tracing::debug!(%name, "audio clip skipped: a held sound has the PCM");
            return;
        }
        self.stop_child();
        self.prepare_clip(name, false);
    }

    pub fn hold_clip(&mut self, name: Arc<str>) {
        if matches!(self.ride, Ride::Theremin | Ride::Singing) {
            return;
        }
        if self.ride == Ride::Clip
            && self.clip.as_ref().is_some_and(|clip| clip.name == name && clip.looping)
        {
            return;
        }
        self.stop_child();
        self.ride = Ride::Clip;
        self.prepare_clip(name, true);
    }

    pub fn release_clip(&mut self) {
        if self.ride == Ride::Clip {
            self.stop_child();
        }
    }

    /// No clip file is opened or read on the control thread.
    pub fn poll_clip(&mut self) {
        let Some(mut clip) = self.clip.take() else { return };
        match &clip.state {
            ClipState::Preparing(receiver) => match receiver.try_recv() {
                Ok(Ok(prepared)) => match self.start_prepared_clip(prepared, &clip) {
                    Ok(()) => clip.state = ClipState::Playing,
                    Err(error) => {
                        tracing::warn!(name = %clip.name, %error, "cannot start audio clip");
                        clip.state = ClipState::Failed;
                    }
                },
                Ok(Err(error)) => {
                    tracing::warn!(name = %clip.name, %error, "cannot prepare audio clip");
                    clip.state = ClipState::Failed;
                }
                Err(mpsc::TryRecvError::Empty) => {}
                Err(mpsc::TryRecvError::Disconnected) => {
                    tracing::warn!(name = %clip.name, "audio clip preparation stopped");
                    clip.state = ClipState::Failed;
                }
            },
            ClipState::Playing => {
                match self.child.as_mut().map(Child::try_wait) {
                    Some(Ok(None)) => {}
                    Some(Ok(Some(status))) => {
                        self.child = None;
                        if !status.success() || clip.looping {
                            tracing::warn!(name = %clip.name, %status, "audio clip player exited");
                            clip.state = ClipState::Failed;
                        } else {
                            return;
                        }
                    }
                    Some(Err(error)) => {
                        tracing::warn!(name = %clip.name, %error, "cannot inspect audio clip player");
                        clip.state = ClipState::Failed;
                    }
                    None => {
                        tracing::warn!(name = %clip.name, "audio clip player is missing");
                        clip.state = ClipState::Failed;
                    }
                }
            }
            ClipState::Failed => {}
        }
        self.clip = Some(clip);
    }

    fn start_prepared_clip(&mut self, prepared: PreparedClip, clip: &Clip) -> std::io::Result<()> {
        let mut command = self.player_command();
        command.args(["-q", "-D", &self.device]);
        command.stdout(Stdio::null()).stderr(Stdio::inherit());
        let child = match prepared {
            PreparedClip::Loop { reader, format } => {
                command.args(["-t", "raw", "-f", format.sample_format, "-c", &format.spec.channels.to_string(),
                    "-r", &format.spec.sample_rate.to_string()]);
                let mut child = command.stdin(Stdio::piped()).spawn()?;
                let stdin = child.stdin.take().expect("piped clip player stdin");
                let cancel = Arc::clone(&clip.cancel);
                let name = Arc::clone(&clip.name);
                if let Err(error) = std::thread::Builder::new().name("clip-loop".into()).spawn(move || {
                    stream_clip(reader, format, stdin, cancel, &name);
                }) {
                    signal_player(&child, libc::SIGTERM);
                    std::thread::spawn(move || wait_bounded(&mut child, Duration::from_millis(500)));
                    return Err(error);
                }
                child
            }
            PreparedClip::Once(file) => command.args(["-t", "wav"]).stdin(Stdio::from(file)).spawn()?,
        };
        self.child = Some(child);
        Ok(())
    }
}

fn open_clip(dir: &Path, name: &str, looping: bool) -> std::io::Result<PreparedClip> {
    use std::io::Seek;

    if !(duck_ipc_proto::SoundClipParams { name: name.to_owned(), hold: None }).valid() {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, "invalid clip name"));
    }
    let file = std::fs::OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK)
        .open(dir.join(format!("{name}.wav")))?;
    if !file.metadata()?.is_file() {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "clip is not a regular file"));
    }
    let reader = hound::WavReader::new(std::io::BufReader::new(file)).map_err(wav_error)?;
    if !looping {
        let mut file = reader.into_inner().into_inner();
        file.rewind()?;
        return Ok(PreparedClip::Once(file));
    }
    let spec = reader.spec();
    if reader.len() == 0 || spec.sample_rate == 0 {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "empty WAV or invalid sample rate"));
    }
    let sample_format = match (spec.sample_format, spec.bits_per_sample) {
        (hound::SampleFormat::Int, 8 | 16) => "S16_LE",
        (hound::SampleFormat::Int, 24 | 32) => "S32_LE",
        (hound::SampleFormat::Float, 32) => "FLOAT_LE",
        _ => return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "unsupported WAV sample format")),
    };
    Ok(PreparedClip::Loop { reader, format: ClipFormat { spec, sample_format } })
}

fn stream_clip(
    mut reader: hound::WavReader<std::io::BufReader<std::fs::File>>,
    format: ClipFormat,
    mut stdin: std::process::ChildStdin,
    cancel: Arc<AtomicBool>,
    name: &str,
) {
    use std::io::{Seek, Write};
    block_sigpipe();
    let result = (|| -> std::io::Result<()> {
        let mut buffer = Vec::with_capacity(8192);
        while !cancel.load(Ordering::Relaxed) {
            macro_rules! write_samples {
                ($sample:ty, $encode:expr) => {
                    for sample in reader.samples::<$sample>() {
                        if cancel.load(Ordering::Relaxed) { return Ok(()); }
                        let bytes = ($encode)(sample.map_err(wav_error)?);
                        buffer.extend_from_slice(&bytes);
                        if buffer.len() >= 8192 {
                            stdin.write_all(&buffer)?;
                            buffer.clear();
                        }
                    }
                };
            }
            match (format.spec.sample_format, format.spec.bits_per_sample) {
                (hound::SampleFormat::Int, bits @ (8 | 16)) => {
                    write_samples!(i16, |sample: i16| (sample << (16 - bits)).to_le_bytes());
                }
                (hound::SampleFormat::Int, bits @ (24 | 32)) => {
                    write_samples!(i32, |sample: i32| (sample << (32 - bits)).to_le_bytes());
                }
                (hound::SampleFormat::Float, 32) => write_samples!(f32, |sample: f32| sample.to_le_bytes()),
                _ => return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "unsupported WAV sample format")),
            }
            stdin.write_all(&buffer)?;
            buffer.clear();
            if format.spec.bits_per_sample == 24 {
                // hound's seek uses valid bits, which can differ from the container width.
                let mut file = reader.into_inner().into_inner();
                file.rewind()?;
                reader = hound::WavReader::new(std::io::BufReader::new(file)).map_err(wav_error)?;
            } else {
                reader.seek(0)?;
            }
        }
        Ok(())
    })();
    if let Err(error) = result && !cancel.load(Ordering::Relaxed) {
        tracing::warn!(%name, %error, "audio clip stream failed");
    }
}

fn wav_error(error: hound::Error) -> std::io::Error {
    std::io::Error::new(std::io::ErrorKind::InvalidData, error)
}
