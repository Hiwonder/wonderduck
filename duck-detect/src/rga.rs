//! RGA conversion of camera frames into a letterboxed model input.

use std::ffi::{CStr, c_char, c_int, c_void};

use anyhow::{Context, Result, bail, ensure};
use uyvy::{Letterbox, Turn, letterbox_rect_layout};

const UYVY: c_int = 0x1e << 8;
const RGB: c_int = 0x02 << 8;
const BT601_LIMITED: c_int = 1;
const SYNC: c_int = 1 << 19;

#[repr(C)]
#[derive(Clone, Copy)]
struct Buffer {
    virtual_address: *mut c_void,
    physical_address: *mut c_void,
    fd: c_int,
    width: c_int,
    height: c_int,
    width_stride: c_int,
    height_stride: c_int,
    format: c_int,
    color_space: c_int,
    alpha: c_int,
    read_mode: c_int,
    color: c_int,
    color_key: [c_int; 2],
    neural_network: [c_int; 6],
    rop_code: c_int,
    handle: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct Rect {
    x: c_int,
    y: c_int,
    width: c_int,
    height: c_int,
}

type Wrap = unsafe extern "C" fn(*mut c_void, c_int, c_int, c_int, c_int, c_int) -> Buffer;
type Process = unsafe extern "C" fn(Buffer, Buffer, Buffer, Rect, Rect, Rect, c_int) -> c_int;
type ErrorText = unsafe extern "C" fn(c_int) -> *const c_char;

pub struct Converter {
    _library: libloading::Library,
    wrap: Wrap,
    process: Process,
    error_text: ErrorText,
}

impl Converter {
    pub fn open() -> Result<Self> {
        // SAFETY: symbols use the C ABI declared by the installed RGA headers.
        unsafe {
            let library =
                libloading::Library::new("librga.so.2").context("cannot load RGA runtime")?;
            let wrap = *library
                .get::<Wrap>(b"wrapbuffer_virtualaddr_t\0")
                .context("RGA buffer API")?;
            let process = *library
                .get::<Process>(b"improcess\0")
                .context("RGA processing API")?;
            let error_text = *library
                .get::<ErrorText>(b"imStrError_t\0")
                .context("RGA error API")?;
            Ok(Self {
                _library: library,
                wrap,
                process,
                error_text,
            })
        }
    }

    pub fn convert(
        &mut self,
        input: &[u8],
        width: usize,
        height: usize,
        size: usize,
        turn: Turn,
        output: &mut Vec<u8>,
    ) -> Result<Letterbox> {
        self.convert_rect(input, width, height, (size, size), turn, output)
    }

    /// Convert a camera frame into an RGB input given as (width, height).
    pub fn convert_rect(
        &mut self,
        input: &[u8],
        width: usize,
        height: usize,
        model_size: (usize, usize),
        turn: Turn,
        output: &mut Vec<u8>,
    ) -> Result<Letterbox> {
        let (model_width, model_height) = model_size;
        ensure!(
            width > 0 && height > 0 && model_width > 0 && model_height > 0,
            "invalid frame geometry"
        );
        ensure!(width.is_multiple_of(2), "UYVY frame width is odd");
        let wanted = width
            .checked_mul(height)
            .and_then(|v| v.checked_mul(2))
            .context("frame size overflow")?;
        ensure!(input.len() == wanted, "invalid frame buffer size");
        let output_size = model_width
            .checked_mul(model_height)
            .and_then(|v| v.checked_mul(3))
            .context("output size overflow")?;
        let w: c_int = width.try_into().context("frame width is too large")?;
        let h: c_int = height.try_into().context("frame height is too large")?;
        let dst_w: c_int = model_width.try_into().context("model width is too large")?;
        let dst_h: c_int = model_height.try_into().context("model height is too large")?;
        let (fit, fitted_w, fitted_h) = letterbox_rect_layout(width, height, model_size, turn);
        output.clear();
        output.resize(output_size, 0);
        // SAFETY: the buffers stay allocated until the synchronous RGA request returns. The
        // hardware reads the source and writes only the destination.
        unsafe {
            let mut source = (self.wrap)(input.as_ptr() as *mut c_void, w, h, w, h, UYVY);
            let destination = (self.wrap)(output.as_mut_ptr().cast(), dst_w, dst_h, dst_w, dst_h, RGB);
            source.color_space = BT601_LIMITED;
            let empty: Buffer = std::mem::zeroed();
            let src_rect = Rect {
                x: 0,
                y: 0,
                width: w,
                height: h,
            };
            let dst_rect = Rect {
                x: fit.pad_x as c_int,
                y: fit.pad_y as c_int,
                width: fitted_w as c_int,
                height: fitted_h as c_int,
            };
            let rotation = match turn {
                Turn::None => 0,
                Turn::Right => 1,
                Turn::Half => 2,
                Turn::Left => 4,
            };
            let code = (self.process)(
                source,
                destination,
                empty,
                src_rect,
                dst_rect,
                Rect::default(),
                SYNC | rotation,
            );
            if code != 1 && code != 2 {
                let message = (self.error_text)(code);
                let detail = if message.is_null() {
                    "unknown RGA error".into()
                } else {
                    CStr::from_ptr(message).to_string_lossy()
                };
                bail!("RGA conversion failed ({code}): {detail}");
            }
        }
        Ok(fit)
    }
}
