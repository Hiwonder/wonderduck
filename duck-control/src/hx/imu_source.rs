//! Select the independent sampler or the IMU sharing the servo transaction.

use super::{HX_BUS_JOINT_IDS, IMU_DXL_ID};
use crate::i2c_imu::I2cImu;
use crate::imu::{IMU_BLOCK_LEN, ImuData, SflpDecoder, rotate};
use crate::io::{
    AccelerationCalibration, AccelerationSample, ImuSampleTiming, ImuStale, ImuTimingSource,
    IoError, Result,
};

const CONTROL_IDS: [u8; 16] = [
    IMU_DXL_ID, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14,
];
const CONTROL_IDS_IMU_LAST: [u8; 16] = [
    20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14, IMU_DXL_ID,
];

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum ImuSource {
    #[default]
    I2c,
    Id200,
    Id200Timing,
    /// Second shared-bus IMU; timing firmware is required.
    Id201,
    Id200Host,
}

impl ImuSource {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::I2c => "i2c",
            Self::Id200 | Self::Id200Timing => "id200",
            Self::Id201 => "id201",
            Self::Id200Host => "id200-host",
        }
    }

    pub fn shared_bus(self) -> bool { self != Self::I2c }

    pub fn bus_id(self) -> Option<u8> {
        match self { Self::I2c => None, Self::Id201 => Some(201), _ => Some(200) }
    }

    pub fn timed(self) -> bool { matches!(self, Self::Id200Timing | Self::Id201) }

    pub fn timing_source(self) -> ImuTimingSource {
        match self {
            Self::I2c => ImuTimingSource::I2c,
            Self::Id201 => ImuTimingSource::Id201,
            Self::Id200Host => ImuTimingSource::Id200Host,
            _ => ImuTimingSource::Id200,
        }
    }
}

impl std::str::FromStr for ImuSource {
    type Err = String;

    fn from_str(value: &str) -> std::result::Result<Self, Self::Err> {
        match value {
            "i2c" => Ok(Self::I2c),
            "id200" => Ok(Self::Id200),
            "id200-timing" => Ok(Self::Id200Timing),
            "id201" => Ok(Self::Id201),
            "id200-host" => Ok(Self::Id200Host),
            _ => Err("expected i2c, id200, id200-timing, id201 or id200-host".into()),
        }
    }
}

pub(super) struct BusImu {
    last: bool,
    source: ImuSource,
    decoder: SflpDecoder,
    previous: Option<[u8; IMU_BLOCK_LEN]>,
    stale: ImuStale,
    replies: u64,
    previous_mcu_sequence: Option<u32>,
    acceleration_calibration: Option<AccelerationCalibration>,
    acceleration_calibration_enabled: bool,
}

pub(super) enum ImuReader {
    I2c(Option<I2cImu>),
    Bus(BusImu),
    Id200Host { last: bool, fusion: crate::imu_host::HostImu },
}

impl ImuReader {
    pub(super) fn begin_control_read(&self) {
        if let Self::I2c(Some(sampler)) = self { sampler.begin_control_read(); }
    }

    pub(super) fn end_control_read(&self) {
        if let Self::I2c(Some(sampler)) = self { sampler.end_control_read(); }
    }

    pub(super) fn new(source: ImuSource) -> Self {
        match source {
            ImuSource::I2c => Self::I2c(None),
            ImuSource::Id200Host => Self::Id200Host {
                last: false, fusion: crate::imu_host::HostImu::default(),
            },
            ImuSource::Id200 | ImuSource::Id200Timing | ImuSource::Id201 => Self::Bus(BusImu {
                last: false,
                source,
                // Preserve the former HX mounting, shared by the calibration tools.
                decoder: SflpDecoder::with_gyro_unit([0.5; 4], 0.035),
                previous: None,
                stale: ImuStale::default(),
                replies: 0,
                previous_mcu_sequence: None,
                acceleration_calibration: None,
                acceleration_calibration_enabled: true,
            }),
        }
    }

    pub(super) fn source(&self) -> ImuSource {
        match self {
            Self::I2c(_) => ImuSource::I2c,
            Self::Bus(bus) => bus.source,
            Self::Id200Host { .. } => ImuSource::Id200Host,
        }
    }

    pub(super) fn control_ids(&self) -> &'static [u8] {
        match self {
            Self::I2c(_) => &HX_BUS_JOINT_IDS,
            Self::Id200Host { .. } => &HX_BUS_JOINT_IDS,
            Self::Bus(bus) if bus.source.timed() => &HX_BUS_JOINT_IDS,
            Self::Bus(bus) if bus.last => &CONTROL_IDS_IMU_LAST,
            Self::Bus(_) => &CONTROL_IDS,
        }
    }

    pub(super) fn timing_last(&self) -> bool {
        match self {
            Self::Bus(bus) => bus.last,
            Self::Id200Host { last, .. } => *last,
            Self::I2c(_) => false,
        }
    }

    pub(super) fn accept_mcu_sequence(&mut self, sequence: u32) -> Result<()> {
        let Self::Bus(bus) = self else {
            return Err(IoError::Bus("MCU timing requires a shared-bus IMU".into()));
        };
        if bus.previous_mcu_sequence.is_some_and(|previous|
            sequence.wrapping_sub(previous) >= (1 << 31)) {
            return Err(IoError::Bus("MCU fusion sequence regressed; restart and inspect reset evidence".into()));
        }
        bus.previous_mcu_sequence = Some(sequence);
        Ok(())
    }

    pub(super) fn set_last(&mut self, enabled: bool) -> Result<()> {
        match self {
            Self::Bus(bus) => bus.last = enabled,
            Self::Id200Host { last, .. } => *last = enabled,
            Self::I2c(_) if enabled => {
                return Err(IoError::Bus("IMU-last requires a shared-bus IMU".into()));
            }
            Self::I2c(_) => {}
        }
        Ok(())
    }

    pub(super) fn start(&mut self) -> Result<()> {
        if let Self::I2c(sampler @ None) = self {
            *sampler = Some(I2cImu::open()?);
        }
        Ok(())
    }

    pub(super) fn set_acceleration_calibration(&mut self, value: AccelerationCalibration) {
        if let Self::Bus(bus) = self {
            bus.acceleration_calibration = Some(value);
        }
    }

    pub(super) fn set_acceleration_enabled(&mut self, enabled: bool) {
        if let Self::Bus(bus) = self {
            bus.acceleration_calibration_enabled = enabled;
        }
    }

    pub(super) fn acceleration_enabled(&self) -> bool {
        matches!(self, Self::Bus(bus) if bus.acceleration_calibration_enabled)
    }

    pub(super) fn record_acceleration(&self, block: &[u8]) -> Result<AccelerationSample> {
        let Self::Bus(bus) = self else {
            return Err(IoError::Bus("acceleration broadcast requires a shared-bus IMU".into()));
        };
        if block.len() != 28 {
            return Err(IoError::ShortRead {
                what: "ID200 acceleration recording block",
                expected: 28,
                got: block.len(),
            });
        }
        let calibration = bus.acceleration_calibration.ok_or_else(|| {
            IoError::Bus("shared-bus IMU acceleration calibration was not read at startup".into())
        })?;
        let full_scale_g = block[12];
        let saturated = block[13];
        let gyro_full_scale_dps = u16::from_le_bytes([block[14], block[15]]);
        let raw_sensor_g: [f64; 3] = std::array::from_fn(|i| {
            f64::from(f32::from_le_bytes(
                block[i * 4..i * 4 + 4].try_into().unwrap(),
            ))
        });
        if ![2, 4, 8, 16].contains(&full_scale_g)
            || saturated & 0xc0 != 0
            || !(1..=1146).contains(&gyro_full_scale_dps)
            || raw_sensor_g
                .iter()
                .any(|v| !v.is_finite() || v.abs() > f64::from(full_scale_g))
        {
            return Err(IoError::Bus(
                "invalid shared-bus IMU acceleration or range descriptor".into(),
            ));
        }
        let corrected_sensor_g = std::array::from_fn(|i| {
            if bus.acceleration_calibration_enabled {
                f64::from((raw_sensor_g[i] as f32 - calibration.bias_g[i]) * calibration.scale[i])
            } else {
                raw_sensor_g[i]
            }
        });
        Ok(AccelerationSample {
            raw_sensor_g,
            corrected_sensor_g,
            raw_trunk_g: rotate([0.5; 4], raw_sensor_g),
            corrected_trunk_g: rotate([0.5; 4], corrected_sensor_g),
            full_scale_g,
            gyro_full_scale_dps,
            accel_saturated_axes: std::array::from_fn(|i| saturated & (1 << i) != 0),
            gyro_saturated_axes: std::array::from_fn(|i| saturated & (1 << (i + 3)) != 0),
            calibration,
        })
    }

    pub(super) fn sample(
        &mut self,
        bus_block: Option<&[u8]>,
        acquired_t_ns: u64,
    ) -> Result<(ImuData, ImuSampleTiming, Option<[f64; 3]>)> {
        let (imu, sequence, acquired_t_ns, acceleration_g, i2c_timing) = match self {
            Self::Id200Host { .. } => return Err(IoError::Bus(
                "host IMU requires the timestamped raw batch reader".into())),
            Self::I2c(sampler) => {
                let sample = sampler.as_ref().expect("I2C sampler started").sample()?;
                (sample.imu, sample.sequence, sample.acquired_t_ns, Some(sample.accel_g), sample.read_timing)
            }
            Self::Bus(bus) => {
                let block = bus_block.unwrap_or(&[]);
                let raw: &[u8; IMU_BLOCK_LEN] =
                    block.try_into().map_err(|_| IoError::ShortRead {
                        what: "ID200 IMU block",
                        expected: IMU_BLOCK_LEN,
                        got: block.len(),
                    })?;
                if bus.previous.replace(*raw) == Some(*raw) {
                    bus.stale.total = bus.stale.total.saturating_add(1);
                    bus.stale.run = bus.stale.run.saturating_add(1);
                } else {
                    bus.stale.run = 0;
                }
                // Count host replies, not unobserved MCU sensor samples.
                bus.replies = bus.replies.saturating_add(1);
                (bus.decoder.decode_packed(raw).map_err(|error| IoError::Bus(error.into()))?, bus.replies, acquired_t_ns, None, None)
            }
        };
        let consumed_t_ns = duck_ipc_proto::clock::monotonic_ns();
        Ok((
            imu,
            ImuSampleTiming {
                source: match self {
                    Self::I2c(_) => ImuTimingSource::I2c,
                    Self::Bus(bus) => bus.source.timing_source(),
                    Self::Id200Host { .. } => unreachable!("raw batch has an independent decoder"),
                },
                sequence,
                acquired_t_ns,
                consumed_t_ns,
                age_us: consumed_t_ns.saturating_sub(acquired_t_ns) / 1000,
                payload_unchanged_run: match self {
                    Self::I2c(_) => 0,
                    Self::Bus(bus) => bus.stale.run,
                    Self::Id200Host { .. } => 0,
                },
                mcu: None,
                host: None,
                i2c: i2c_timing,
            },
            acceleration_g,
        ))
    }

    pub(super) fn stale(&self) -> ImuStale {
        match self {
            Self::I2c(sampler) => sampler
                .as_ref()
                .map_or_else(ImuStale::default, I2cImu::stale),
            Self::Bus(bus) => bus.stale,
            Self::Id200Host { .. } => ImuStale::default(),
        }
    }

    pub(super) fn ready(&self) -> bool {
        match self {
            Self::I2c(sampler) => sampler.as_ref().is_some_and(I2cImu::ready),
            Self::Bus(bus) => bus.decoder.ready(),
            Self::Id200Host { fusion, .. } => fusion.ready(),
        }
    }

    pub(super) fn consume_host_batch(
        &mut self, data: &[u8], request_ns: u64, reply_ns: u64,
    ) -> Result<crate::imu_host::HostImuOutput> {
        match self {
            Self::Id200Host { fusion, .. } => fusion.consume(data, request_ns, reply_ns),
            _ => Err(IoError::Bus("raw IMU batch requires id200-host source".into())),
        }
    }

    pub(super) fn invalidate_host(&mut self) {
        if let Self::Id200Host { fusion, .. } = self {
            fusion.restart();
        }
    }
}
