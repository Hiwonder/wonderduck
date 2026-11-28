use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

pub const PATH: &str = "/etc/robot/device-version.json";
const WING_HARDWARE_VERSION: &str = "1.1.0";

pub fn supports_wing_flap(path: &Path) -> bool {
    let Ok(data) = fs::read(path) else {
        return false;
    };
    let Ok(versions) = serde_json::from_slice::<serde_json::Value>(&data) else {
        return false;
    };
    versions["hardware_version"].as_str() == Some(WING_HARDWARE_VERSION)
}

pub fn record_wing_hardware(path: &Path) -> io::Result<bool> {
    let mut versions: serde_json::Map<String, serde_json::Value> =
        serde_json::from_slice(&fs::read(path)?)?;
    let current = versions
        .get("hardware_version")
        .and_then(|value| value.as_str())
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "hardware_version is missing"))?;
    let numbers: Vec<u32> = current
        .split('.')
        .map(str::parse)
        .collect::<Result<_, _>>()
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let [major, minor, patch] = numbers.as_slice() else {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "hardware_version must have three components",
        ));
    };
    if (*major, *minor, *patch) >= (1, 1, 0) {
        return Ok(false);
    }
    versions.insert("hardware_version".into(), WING_HARDWARE_VERSION.into());

    let parent = path
        .parent()
        .ok_or_else(|| io::Error::other("device version directory is missing"))?;
    let suffix = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(io::Error::other)?
        .as_nanos();
    let staged = path.with_extension(format!("json.{}.{suffix}.tmp", std::process::id()));
    let permissions = fs::metadata(path)?.permissions();
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&staged)?;
    let result = (|| {
        file.set_permissions(permissions)?;
        serde_json::to_writer_pretty(&mut file, &versions)?;
        file.write_all(b"\n")?;
        file.sync_all()?;
        fs::rename(&staged, path)?;
        File::open(parent)?.sync_all()
    })();
    if result.is_err() {
        let _ = fs::remove_file(staged);
    }
    result.map(|()| true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};

    #[test]
    fn records_wing_hardware_and_preserves_other_version_data() {
        let directory =
            std::env::temp_dir().join(format!("robotd-device-version-{}", std::process::id()));
        fs::create_dir_all(&directory).unwrap();
        let path = directory.join("device-version.json");
        assert!(!supports_wing_flap(&path));
        fs::write(
            &path,
            br#"{"hardware_version":"1.0.0","image_version":"1.2.3","other":true}"#,
        )
        .unwrap();
        assert!(!supports_wing_flap(&path));
        assert!(record_wing_hardware(&path).unwrap());
        assert!(supports_wing_flap(&path));
        let updated: Value = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
        assert_eq!(
            updated,
            json!({"hardware_version": "1.1.0", "image_version": "1.2.3", "other": true})
        );
        assert!(!record_wing_hardware(&path).unwrap());

        fs::write(
            &path,
            br#"{"hardware_version":"2.0.0","image_version":"1.2.3"}"#,
        )
        .unwrap();
        assert!(!record_wing_hardware(&path).unwrap());
        assert!(fs::read_to_string(&path).unwrap().contains("2.0.0"));
        assert!(!supports_wing_flap(&path));
        fs::write(&path, b"invalid JSON").unwrap();
        assert!(!supports_wing_flap(&path));
        fs::remove_dir_all(&directory).unwrap();
    }
}
