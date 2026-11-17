//! Signature and hash verification.
//!
//! The trust anchor is a *set* of minisign public keys on disk, not one baked-in
//! key, so a lost or compromised key is survivable.
//!
//!
//! Invariant: **no unsigned bytes are ever extracted to a live path or executed.**
//! Verification order is manifest signature → artifact hash → artifact
//! signature; extraction happens only after all three pass.
//!
//! We depend on `minisign-verify` (zero dependencies, **verify-only**) rather than
//! the full `minisign` crate: this process has no business being able to sign, so
//! it shouldn't link the code that can.

use std::fs::File;
use std::io::Read;
use std::path::Path;

use minisign_verify::{PublicKey, Signature};
use sha2::{Digest, Sha256};

use crate::Error;

/// Read buffer for streaming hash/verify. Kind to a small board while still
/// amortising syscalls.
const CHUNK: usize = 64 * 1024;

/// Bounds on what an archive may expand to.
///
/// Not a security boundary — the signature already established provenance — but a
/// guard against an accidentally enormous artifact filling the eMMC. Configurable
/// because a model bundle of several ONNX policies is legitimately much larger than
/// a daemon binary, and a too-low ceiling would
/// reject a genuine release.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArchiveLimits {
    pub max_uncompressed_bytes: u64,
    pub max_entries: usize,
}

impl Default for ArchiveLimits {
    fn default() -> Self {
        Self {
            max_uncompressed_bytes: 2 * 1024 * 1024 * 1024,
            max_entries: 50_000,
        }
    }
}

/// Keys whose filename ends with this are usable only when `allow_dev_keys` is
/// set, so a production robot won't install a team member's local build.
const DEV_KEY_SUFFIX: &str = ".dev.pub";

pub struct TrustedKey {
    /// Filename, so the update log can record which key admitted an artifact.
    pub id: String,
    pub dev_only: bool,
    key: PublicKey,
}

/// Hand-written so the key material is never printed by accident — only its id.
impl std::fmt::Debug for TrustedKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TrustedKey")
            .field("id", &self.id)
            .field("dev_only", &self.dev_only)
            .finish_non_exhaustive()
    }
}

#[derive(Debug)]
pub struct KeyRing {
    keys: Vec<TrustedKey>,
    allow_dev_keys: bool,
}

impl KeyRing {
    /// Load every `*.pub` in `dir`.
    ///
    /// An empty keyring is an **error**, not an empty allow-list: silently
    /// trusting nothing looks identical to a misconfigured path, and guessing
    /// wrong here is catastrophic in either direction.
    pub fn load(dir: &Path, allow_dev_keys: bool) -> Result<Self, Error> {
        let entries = std::fs::read_dir(dir).map_err(|e| Error::Io {
            path: dir.to_path_buf(),
            source: e,
        })?;

        let mut keys = Vec::new();
        for entry in entries {
            let entry = entry.map_err(|e| Error::Io {
                path: dir.to_path_buf(),
                source: e,
            })?;
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) != Some("pub") {
                continue;
            }
            let name = path
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or_default()
                .to_owned();

            let text = std::fs::read_to_string(&path).map_err(|e| Error::Io {
                path: path.clone(),
                source: e,
            })?;

            // `decode` handles minisign's two-line format (comment + key); fall
            // back to a bare base64 key for convenience.
            let key = PublicKey::decode(&text)
                .or_else(|_| PublicKey::from_base64(text.trim()))
                .map_err(|e| {
                    Error::Config(format!(
                        "{}: not a minisign public key: {e}",
                        path.display()
                    ))
                })?;

            keys.push(TrustedKey {
                dev_only: name.ends_with(DEV_KEY_SUFFIX),
                id: name,
                key,
            });
        }

        if keys.is_empty() {
            return Err(Error::Config(format!(
                "no trusted keys in {} — refusing to run with an empty trust anchor",
                dir.display()
            )));
        }

        Ok(Self {
            keys,
            allow_dev_keys,
        })
    }

    /// Construct directly. For tests, and for callers that already hold keys.
    pub fn from_keys(keys: Vec<TrustedKey>, allow_dev_keys: bool) -> Self {
        Self {
            keys,
            allow_dev_keys,
        }
    }

    /// Build a trusted key from a base64 minisign public key.
    pub fn key_from_base64(id: &str, b64: &str, dev_only: bool) -> Result<TrustedKey, Error> {
        let key = PublicKey::from_base64(b64)
            .map_err(|e| Error::Config(format!("{id}: invalid public key: {e}")))?;
        Ok(TrustedKey {
            id: id.to_owned(),
            dev_only,
            key,
        })
    }

    fn usable(&self) -> impl Iterator<Item = &TrustedKey> {
        self.keys
            .iter()
            .filter(move |k| self.allow_dev_keys || !k.dev_only)
    }

    fn parse_signature(sig: &[u8]) -> Result<Signature, Error> {
        let text = std::str::from_utf8(sig)
            .map_err(|_| Error::Verification("signature is not valid UTF-8".into()))?;
        Signature::decode(text)
            .map_err(|e| Error::Verification(format!("malformed signature: {e}")))
    }

    /// Verify a detached signature over in-memory bytes (manifests).
    ///
    /// Returns the key that matched, so the update log can record which key
    /// admitted the artifact — useful for spotting a robot still relying on a key
    /// we meant to retire.
    pub fn verify_bytes(&self, data: &[u8], signature: &[u8]) -> Result<&TrustedKey, Error> {
        let sig = Self::parse_signature(signature)?;
        let mut tried = 0;
        for key in self.usable() {
            tried += 1;
            if key.key.verify(data, &sig, false).is_ok() {
                return Ok(key);
            }
        }
        Err(Self::no_key_error(tried))
    }

    /// Verify a detached signature over a file, streaming it.
    ///
    /// Streaming rather than reading it whole: this runs on a board with limited
    /// RAM and artifacts are not small.
    pub fn verify_file(&self, path: &Path, signature: &[u8]) -> Result<&TrustedKey, Error> {
        let sig = Self::parse_signature(signature)?;

        let mut tried = 0;
        for key in self.usable() {
            tried += 1;
            let Ok(mut verifier) = key.key.verify_stream(&sig) else {
                // Wrong key id, or a legacy non-prehashed signature. Try the next.
                continue;
            };

            let mut file = File::open(path).map_err(|e| Error::Io {
                path: path.to_path_buf(),
                source: e,
            })?;
            let mut buf = vec![0u8; CHUNK];
            loop {
                let n = file.read(&mut buf).map_err(|e| Error::Io {
                    path: path.to_path_buf(),
                    source: e,
                })?;
                if n == 0 {
                    break;
                }
                verifier.update(&buf[..n]);
            }

            if verifier.finalize().is_ok() {
                return Ok(key);
            }
        }
        Err(Self::no_key_error(tried))
    }

    fn no_key_error(tried: usize) -> Error {
        Error::Verification(format!(
            "signature did not verify against any of {tried} usable trusted key(s)"
        ))
    }
}

/// Check a file's SHA-256 against the manifest's hex digest.
///
/// Integrity only — authenticity comes from the signature — so a plain comparison
/// is fine; case-insensitive because hex casing varies between tools.
pub fn verify_sha256(path: &Path, expected_hex: &str) -> Result<(), Error> {
    let actual = sha256_hex(path)?;
    if actual.eq_ignore_ascii_case(expected_hex.trim()) {
        Ok(())
    } else {
        Err(Error::Verification(format!(
            "sha256 mismatch: manifest says {expected_hex}, artifact is {actual}"
        )))
    }
}

pub fn sha256_hex(path: &Path) -> Result<String, Error> {
    let mut file = File::open(path).map_err(|e| Error::Io {
        path: path.to_path_buf(),
        source: e,
    })?;
    let mut hasher = Sha256::new();
    let mut buf = vec![0u8; CHUNK];
    loop {
        let n = file.read(&mut buf).map_err(|e| Error::Io {
            path: path.to_path_buf(),
            source: e,
        })?;
        if n == 0 {
            break;
        }
        hasher.update(&buf[..n]);
    }
    Ok(hex(&hasher.finalize()))
}

fn hex(bytes: &[u8]) -> String {
    use std::fmt::Write;
    bytes.iter().fold(String::new(), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}

/// Extract a `.tar.zst` artifact into `dest`.
///
/// **Only call this after signature and hash verification have both passed.**
///
/// Path-traversal safety comes from `tar`'s own `unpack_in`, which refuses
/// absolute paths and entries that would escape the destination, rather than from
/// a hand-rolled check. On top of that we cap total uncompressed size and entry
/// count, which the library does not do — a zip bomb would otherwise fill the
/// eMMC.
pub fn extract_artifact(archive: &Path, dest: &Path, limits: ArchiveLimits) -> Result<(), Error> {
    let file = File::open(archive).map_err(|e| Error::Io {
        path: archive.to_path_buf(),
        source: e,
    })?;
    let decoder = zstd::Decoder::new(file).map_err(|e| Error::Io {
        path: archive.to_path_buf(),
        source: e,
    })?;

    std::fs::create_dir_all(dest).map_err(|e| Error::Io {
        path: dest.to_path_buf(),
        source: e,
    })?;

    let mut tar = tar::Archive::new(decoder);
    tar.set_preserve_permissions(true); // hooks and binaries need the exec bit

    let entries = tar.entries().map_err(|e| Error::Io {
        path: archive.to_path_buf(),
        source: e,
    })?;

    let mut total: u64 = 0;
    let mut count = 0usize;

    for entry in entries {
        let mut entry = entry.map_err(|e| Error::Io {
            path: archive.to_path_buf(),
            source: e,
        })?;

        count += 1;
        if count > limits.max_entries {
            // Deliberately not a verification error: the signature checked out, so
            // this is an oversized artifact or a too-tight limit, not tampering.
            return Err(Error::ArchiveTooLarge(format!(
                "archive has more than {} entries",
                limits.max_entries
            )));
        }

        total = total.saturating_add(entry.size());
        if total > limits.max_uncompressed_bytes {
            return Err(Error::ArchiveTooLarge(format!(
                "archive expands beyond {} bytes (raise max_uncompressed_bytes if this \
                 release is legitimately this big)",
                limits.max_uncompressed_bytes
            )));
        }

        let name = entry
            .path()
            .map(|p| p.display().to_string())
            .unwrap_or_default();

        // `unpack_in` returns Ok(false) when the entry path is unsafe (absolute,
        // or escaping `dest`). Treat that as tampering, not something to skip:
        // we have already verified the signature, so a hostile entry means one of
        // our own keys signed it, which we must surface loudly.
        let unpacked = entry.unpack_in(dest).map_err(|e| Error::Io {
            path: dest.to_path_buf(),
            source: e,
        })?;
        if !unpacked {
            return Err(Error::Verification(format!(
                "archive entry {name:?} would escape the destination; refusing"
            )));
        }
    }

    Ok(())
}
