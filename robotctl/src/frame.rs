//! Local binary snapshots deliberately bypass the JSON-only service router.
use super::{Client, Failure, exit, proto};
use std::io::{BufRead, Read};
use std::path::Path;
use std::time::Duration;

pub(super) fn run(socket: &Path, output: &Path) -> Result<(), Failure> {
    let (header, bytes) = fetch(socket).map_err(failed)?;
    // Never leave a success-looking file behind after an incomplete camera reply.
    std::fs::write(output, bytes).map_err(failed)?;
    eprintln!("{}", serde_json::to_string(&header).map_err(failed)?);
    Ok(())
}

/// One frame off `media.frame`: connect, handshake, ask, read the binary tail.
///
/// Shared with [`crate::camera`], which asks for one of these twice a second while the monitor's
/// camera block is open. One connection per frame is the endpoint's own shape — `mediad` answers a
/// request and hangs up — and one reader for both callers is how the monitor cannot drift from
/// what `robotctl frame` writes to disk.
pub(super) fn fetch(
    socket: &Path,
) -> Result<(proto::MediaFrameHeader, Vec<u8>), Box<dyn std::error::Error>> {
    let mut client = Client::connect_to("mediad", socket).map_err(|e| e.message)?;
    let timeout = Some(Duration::from_secs(3));
    client.reader.get_ref().set_read_timeout(timeout)?;
    client.writer.set_write_timeout(timeout)?;
    client.hello().map_err(|e| e.message)?;
    let id = proto::Id::Number(client.next_id);
    client
        .send(&proto::Request {
            jsonrpc: "2.0".into(),
            id: Some(id.clone()),
            method: proto::method::MEDIA_FRAME.into(),
            params: None,
        })
        .map_err(|e| e.message)?;
    read_frame(&mut client.reader, id)
}

fn failed(error: impl std::fmt::Display) -> Failure {
    Failure::new(exit::FAILED, format!("camera snapshot: {error}"))
}

fn read_frame(
    reader: &mut impl BufRead,
    id: proto::Id,
) -> Result<(proto::MediaFrameHeader, Vec<u8>), Box<dyn std::error::Error>> {
    let mut line = Vec::new();
    reader.by_ref().take(4097).read_until(b'\n', &mut line)?;
    if line.len() > 4096 || !line.ends_with(b"\n") {
        return Err("invalid snapshot header length".into());
    }
    let response: proto::Response = serde_json::from_slice(&line)?;
    if response.id != Some(id) || response.jsonrpc != "2.0" {
        return Err("snapshot response does not match request".into());
    }
    if let Some(error) = response.error {
        return Err(format!("{}: {}", error.code, error.message).into());
    }
    let header: proto::MediaFrameHeader =
        serde_json::from_value(response.result.ok_or("missing snapshot result")?)?;
    if !header.valid_uyvy() {
        return Err("invalid UYVY geometry or size".into());
    }
    let mut bytes = vec![0; header.bytes];
    // Reuse the header reader: it may already have buffered the beginning of the binary tail.
    reader.read_exact(&mut bytes)?;
    Ok((header, bytes))
}
