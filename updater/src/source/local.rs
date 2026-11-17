//! Local-directory source.
//!
//! Two jobs:
//!  - CI tests drive the **real** engine code path with no network, so tests can't
//!    drift from production behaviour;
//!  - the dev sideload flow, where a locally-built artifact signed with the dev
//!    key is applied without touching prod signing (§15).
//!
//! Signature verification is **not** relaxed here. A local artifact is verified
//! exactly like a downloaded one; sideloading works because the dev key is in the
//! trusted set, not because checks are skipped.
//!
//! Layout expected under `path`:
//! ```text
//!   <version>.manifest.json           e.g. 1.4.2.manifest.json
//!   <version>.manifest.json.minisig
//!   <whatever the manifest's `url` names>
//!   <that name>.minisig
//! ```

use std::path::{Path, PathBuf};

use crate::Error;
use crate::manifest::Manifest;
use crate::source::{FetchedArtifact, ProgressSink, SignedBytes, Source};

const MANIFEST_SUFFIX: &str = ".manifest.json";
const SIG_SUFFIX: &str = ".minisig";

pub struct LocalDir {
    root: PathBuf,
}

impl LocalDir {
    pub fn new(root: PathBuf) -> Self {
        Self { root }
    }

    fn manifest_path(&self, version: &semver::Version) -> PathBuf {
        self.root.join(format!("{version}{MANIFEST_SUFFIX}"))
    }

    /// Versions present, newest first. Entries whose name isn't a version are
    /// ignored rather than treated as errors — the directory also holds artifacts
    /// and signatures.
    fn versions(&self) -> Result<Vec<semver::Version>, Error> {
        let entries = std::fs::read_dir(&self.root).map_err(|e| Error::Io {
            path: self.root.clone(),
            source: e,
        })?;

        let mut versions: Vec<_> = entries
            .filter_map(Result::ok)
            .filter_map(|e| {
                let name = e.file_name().to_str()?.to_owned();
                let stem = name.strip_suffix(MANIFEST_SUFFIX)?;
                semver::Version::parse(stem).ok()
            })
            .collect();
        versions.sort_by(|a, b| b.cmp(a));
        Ok(versions)
    }

    fn read_signed(&self, path: &Path) -> Result<SignedBytes<Manifest>, Error> {
        let bytes = std::fs::read(path).map_err(|e| Error::Io {
            path: path.to_path_buf(),
            source: e,
        })?;

        let sig_path = sig_path_for(path);
        let signature = std::fs::read(&sig_path).map_err(|e| Error::Io {
            path: sig_path,
            source: e,
        })?;

        // Parsed for convenience; the *bytes* are what gets verified, since the
        // signature covers exactly what was received, not a re-serialization.
        let parsed: Manifest = serde_json::from_slice(&bytes)
            .map_err(|e| Error::Corrupt(format!("{}: {e}", path.display())))?;

        Ok(SignedBytes {
            bytes,
            signature,
            parsed,
        })
    }

    /// Artifact path from a manifest `url`.
    ///
    /// Only a bare filename is accepted: a manifest must not be able to point the
    /// engine at an arbitrary path on the robot.
    fn artifact_path(&self, manifest: &Manifest) -> Result<PathBuf, Error> {
        let name = Path::new(&manifest.url);
        let is_plain_filename = name.components().count() == 1 && name.file_name().is_some();
        if !is_plain_filename {
            return Err(Error::Verification(format!(
                "local manifest url must be a bare filename, got {:?}",
                manifest.url
            )));
        }
        Ok(self.root.join(name))
    }
}

fn sig_path_for(path: &Path) -> PathBuf {
    let mut s = path.as_os_str().to_owned();
    s.push(SIG_SUFFIX);
    PathBuf::from(s)
}

#[async_trait::async_trait]
impl Source for LocalDir {
    async fn latest_manifest(&self) -> Result<SignedBytes<Manifest>, Error> {
        let newest =
            self.versions()?.into_iter().next().ok_or_else(|| {
                Error::Network(format!("no manifests in {}", self.root.display()))
            })?;
        self.read_signed(&self.manifest_path(&newest))
    }

    async fn manifest_for(
        &self,
        version: &semver::Version,
    ) -> Result<SignedBytes<Manifest>, Error> {
        let path = self.manifest_path(version);
        if !path.exists() {
            return Err(Error::Network(format!(
                "no manifest for version {version} in {}",
                self.root.display()
            )));
        }
        self.read_signed(&path)
    }

    /// A ref names a manifest directly: `my-branch` → `my-branch.manifest.json`.
    ///
    /// Supported here, not just on the GitHub source, for two reasons: it makes the whole
    /// `--ref` path testable through the real engine with no network, and it is the offline
    /// sideload story — copy a directory onto a board and install by name.
    ///
    /// **The ref becomes a filename, so it is validated.** Unlike the GitHub source, where
    /// `feature/foo` is a legitimate tag component, a separator here would escape the
    /// directory — `../../etc/anything` is a path, not a branch. Rejected rather than
    /// sanitised, because silently rewriting a caller's ref would install something other
    /// than what they named.
    async fn manifest_at_ref(&self, git_ref: &str) -> Result<SignedBytes<Manifest>, Error> {
        if git_ref.is_empty()
            || git_ref.contains('/')
            || git_ref.contains('\\')
            || git_ref.contains("..")
        {
            return Err(Error::Verification(format!(
                "refusing ref {git_ref:?}: a local_dir ref becomes a filename, so it may not \
                 contain a path separator or `..`"
            )));
        }

        let path = self.root.join(format!("{git_ref}{MANIFEST_SUFFIX}"));
        if !path.exists() {
            return Err(Error::Network(format!(
                "no manifest for ref {git_ref:?} in {}",
                self.root.display()
            )));
        }
        self.read_signed(&path)
    }

    async fn fetch_artifact(
        &self,
        manifest: &Manifest,
        dest_dir: &Path,
        progress: ProgressSink,
    ) -> Result<FetchedArtifact, Error> {
        let src = self.artifact_path(manifest)?;
        let src_sig = sig_path_for(&src);

        std::fs::create_dir_all(dest_dir).map_err(|e| Error::Io {
            path: dest_dir.to_path_buf(),
            source: e,
        })?;

        let file_name = src
            .file_name()
            .ok_or_else(|| Error::Corrupt("artifact path has no file name".into()))?;
        let artifact = dest_dir.join(file_name);
        let signature = sig_path_for(&artifact);

        // Copy rather than symlink, so staging behaves identically to a real
        // download and the caller can safely delete the staging tree.
        let bytes = std::fs::copy(&src, &artifact).map_err(|e| Error::Io {
            path: src.clone(),
            source: e,
        })?;
        std::fs::copy(&src_sig, &signature).map_err(|e| Error::Io {
            path: src_sig,
            source: e,
        })?;

        // Local copies are instant, but emit the terminal progress so subscribers
        // see the same phase sequence they would for a network fetch.
        let _ = progress.send((bytes, Some(bytes)));

        Ok(FetchedArtifact {
            artifact,
            signature,
            bytes,
        })
    }
}
