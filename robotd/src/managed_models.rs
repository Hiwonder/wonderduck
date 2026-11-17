//! Transaction state and durable paths for online policy changes.

use std::collections::{BTreeMap, HashMap};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use arc_swap::{ArcSwap, ArcSwapOption};
use duck_ipc_proto as proto;

pub struct Operation {
    pub id: String,
    pub result: ArcSwap<proto::ModelUpdateResult>,
}

pub struct Request {
    pub operation: Arc<Operation>,
    pub model: String,
    pub path: PathBuf,
    pub expected_sha256: String,
    pub candidate_sha256: String,
}

pub struct Shared {
    pub models: ArcSwap<Vec<proto::ManagedModel>>,
    pub active: ArcSwapOption<String>,
    pub busy: AtomicBool,
    uncertain: AtomicBool,
    pub gate: Mutex<()>,
    request: ArcSwapOption<Request>,
    operations: Mutex<HashMap<String, Arc<Operation>>>,
    counter: AtomicU64,
}

impl Default for Shared {
    fn default() -> Self {
        Self {
            models: ArcSwap::from_pointee(Vec::new()),
            active: ArcSwapOption::empty(),
            busy: AtomicBool::new(false),
            uncertain: AtomicBool::new(false),
            gate: Mutex::new(()),
            request: ArcSwapOption::empty(),
            operations: Mutex::new(HashMap::new()),
            counter: AtomicU64::new(0),
        }
    }
}

impl Shared {
    pub fn unresolved(&self) -> bool {
        self.busy.load(Ordering::Acquire) || self.uncertain.load(Ordering::Acquire)
    }

    pub fn submit(&self, params: &proto::ApplyModelParams) -> proto::ApplyModelResult {
        let refuse = |reason: String| proto::ApplyModelResult {
            accepted: false, operation_id: None, reason: Some(reason),
        };
        if self.uncertain.load(Ordering::Acquire) {
            return refuse("上次模型更新状态未知，请重启服务后再操作".into());
        }
        if !Path::new(&params.path).is_absolute() {
            return refuse("模型路径必须是绝对路径".into());
        }
        match self.models.load().iter().find(|model| model.id == params.id) {
            None => return refuse(format!("模型 {} 当前未加载", params.id)),
            Some(model) if model.sha256 != params.expected_sha256 => return refuse("模型已变化，请刷新后重试".into()),
            Some(_) => {}
        }
        if self.busy.compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire).is_err() {
            return refuse("另一项模型更新正在进行".into());
        }
        let count = self.counter.fetch_add(1, Ordering::Relaxed);
        let epoch = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)
            .expect("system clock before Unix epoch").as_nanos();
        let id = format!("{}-{epoch}-{count}", std::process::id());
        let operation = Arc::new(Operation {
            id: id.clone(),
            result: ArcSwap::from_pointee(proto::ModelUpdateResult {
                operation_id: id.clone(), state: "pending".into(), error: None,
            }),
        });
        self.operations.lock().expect("operation mutex").insert(id.clone(), Arc::clone(&operation));
        self.request.store(Some(Arc::new(Request {
            operation, model: params.id.clone(), path: PathBuf::from(&params.path),
            expected_sha256: params.expected_sha256.clone(),
            candidate_sha256: params.candidate_sha256.clone(),
        })));
        proto::ApplyModelResult { accepted: true, operation_id: Some(id), reason: None }
    }

    pub fn take(&self) -> Option<Arc<Request>> { self.request.swap(None) }

    pub fn finish(&self, operation: &Operation, status: &str, error: Option<String>) {
        operation.result.store(Arc::new(proto::ModelUpdateResult {
            operation_id: operation.id.clone(), state: status.into(), error,
        }));
        if status == "uncertain" { self.uncertain.store(true, Ordering::Release); }
        self.busy.store(false, Ordering::Release);
    }

    pub fn status(&self, id: &str) -> Option<proto::ModelUpdateResult> {
        self.operations.lock().expect("operation mutex").get(id)
            .map(|operation| operation.result.load().as_ref().clone())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Entry {
    pub path: PathBuf,
    pub sha256: String,
}

pub type Paths = BTreeMap<String, Entry>;

fn config_path(params_path: &Path) -> PathBuf { params_path.with_extension("models.json") }

pub fn load(params_path: &Path) -> Result<Paths, String> {
    let path = config_path(params_path);
    match std::fs::read(&path) {
        Ok(bytes) => serde_json::from_slice(&bytes).map_err(|e| format!("{}: {e}", path.display())),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(Paths::new()),
        Err(error) => Err(format!("{}: {error}", path.display())),
    }
}

/// Prepare a replacement configuration without changing what a restart would load.
pub fn stage(params_path: &Path, id: &str, model: &str, path: &Path, sha256: &str) -> Result<(PathBuf, PathBuf), String> {
    let mut paths = load(params_path)?;
    paths.insert(model.to_owned(), Entry { path: path.to_owned(), sha256: sha256.to_owned() });
    stage_paths(params_path, id, &paths)
}

fn stage_paths(params_path: &Path, id: &str, paths: &Paths) -> Result<(PathBuf, PathBuf), String> {
    let final_path = config_path(params_path);
    let staged = final_path.with_extension(format!("models.{id}.tmp"));
    let data = serde_json::to_vec_pretty(paths).map_err(|e| e.to_string())?;
    let mut file = std::fs::OpenOptions::new().write(true).create_new(true).open(&staged)
        .map_err(|e| format!("{}: {e}", staged.display()))?;
    file.write_all(&data).and_then(|()| file.sync_all())
        .map_err(|e| format!("{}: {e}", staged.display()))?;
    Ok((staged, final_path))
}

/// Keep the established slot API and managed paths consistent on later restarts.
pub fn record_legacy(params_path: &Path, slots: &[crate::params::Slot], path: Option<&Path>) -> Result<(), String> {
    let mut paths = load(params_path)?;
    let before = paths.clone();
    for slot in slots {
        match path {
            Some(path) if !crate::params::is_none_sentinel(path) => {
                use sha2::{Digest, Sha256};
                let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
                let sha256 = Sha256::digest(&bytes).iter().map(|b| format!("{b:02x}")).collect();
                paths.insert(slot.as_str().to_owned(), Entry { path: path.to_owned(), sha256 });
            }
            _ => { paths.remove(slot.as_str()); }
        }
    }
    if paths == before { return Ok(()); }
    let nonce = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)
        .map_err(|e| e.to_string())?.as_nanos();
    let id = format!("legacy-{}-{nonce}", std::process::id());
    let (staged, destination) = stage_paths(params_path, &id, &paths)?;
    commit(&staged, &destination).map_err(|error| match error {
        CommitError::NotApplied(error) | CommitError::Uncertain(error) => error,
    })
}

pub enum CommitError { NotApplied(String), Uncertain(String) }

pub fn commit(staged: &Path, final_path: &Path) -> Result<(), CommitError> {
    std::fs::rename(staged, final_path)
        .map_err(|e| CommitError::NotApplied(format!("{}: {e}", final_path.display())))?;
    let directory = final_path.parent().unwrap_or_else(|| Path::new("."));
    std::fs::File::open(directory).and_then(|directory| directory.sync_all())
        .map_err(|e| CommitError::Uncertain(format!("配置已替换，但无法同步目录 {}: {e}", directory.display())))
}

pub fn commit_checked(staged: &Path, final_path: &Path, candidate: &Path, expected: &str) -> Result<(), CommitError> {
    use sha2::{Digest, Sha256};
    let bytes = std::fs::read(candidate)
        .map_err(|error| CommitError::NotApplied(format!("{}: {error}", candidate.display())))?;
    let digest = Sha256::digest(&bytes).iter().map(|b| format!("{b:02x}")).collect::<String>();
    if digest != expected {
        return Err(CommitError::NotApplied("模型文件在校验期间发生变化".into()));
    }
    commit(staged, final_path)
}

pub fn apply_paths(policy: &mut crate::params::PolicyParams, paths: &Paths) {
    for (id, entry) in paths {
        if let Some(slot) = crate::params::Slot::parse(id) {
            let enabled = policy.resolved().slot(slot).is_some();
            if enabled { policy.set_slot(slot, Some(entry.path.clone())); }
        } else if let Some(skill) = policy.skills.iter_mut().find(|skill| skill.name == *id) {
            skill.path = Some(entry.path.clone());
        } else if let Some(mut skill) = policy.resolved_skills().into_iter().find(|skill| skill.name == *id) {
            skill.path = Some(entry.path.clone());
            policy.skills.push(skill);
        }
    }
}

pub fn verify_loaded(models: &[proto::ManagedModel], paths: &Paths) -> Result<(), String> {
    for model in models {
        if let Some(entry) = paths.get(&model.id)
            && (model.path != entry.path.display().to_string() || model.sha256 != entry.sha256)
        {
            return Err(format!("{} 内容与已保存的模型摘要不一致", model.id));
        }
    }
    Ok(())
}
