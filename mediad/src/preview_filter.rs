//! Spatial filtering for writable preview buffers. A private row ring keeps the source
//! neighbourhood intact while changing a plane in place, including buffers with padding.

#[derive(Default)]
pub(crate) struct Workspace {
    rows: Vec<u8>,
    texture: Texture,
}

const TILE: usize = 16;
#[derive(Default)]
struct Texture {
    tiles: Vec<u8>,
    patterns: Vec<u8>,
    cols: usize,
    lines: usize,
}
impl Texture {
    fn prepare(&mut self, y: &[u8], stride: usize, width: usize, height: usize) {
        self.cols = width.div_ceil(TILE);
        self.lines = height.div_ceil(TILE);
        self.tiles.resize(self.cols * self.lines, 0);
        self.patterns.resize((self.lines + 1) * self.cols, 0);
        for by in 0..self.lines {
            for bx in 0..self.cols {
                // The samples span x-2..x+17 and y..y+15, independently of stride padding.
                let x = bx * TILE;
                let p = by * TILE;
                if x < 2 || x + 18 > width || p + 16 > height {
                    self.tiles[by * self.cols + bx] = 0;
                    continue;
                }
                self.tiles[by * self.cols + bx] = tile_class(y, stride, x, p);
            }
        }
        // A saturated fixture protects both sides of its edge. Keep the bright bit until
        // neighbourhood guards have been computed for all tiles.
        for by in 0..self.lines {
            for bx in 0..self.cols {
                let i = by * self.cols + bx;
                if self.tiles[i] != 1 {
                    continue;
                }
                let bright = (by.saturating_sub(1)..(by + 2).min(self.lines)).any(|j| {
                    self.tiles[j * self.cols + bx.saturating_sub(1)
                        ..j * self.cols + (bx + 2).min(self.cols)]
                        .iter()
                        .any(|v| *v == 2)
                });
                if bright {
                    self.tiles[i] = 0;
                }
            }
        }
        for j in 0..=self.lines {
            let above = j.saturating_sub(1).min(self.lines - 1) * self.cols;
            let below = j.min(self.lines - 1) * self.cols;
            for i in 0..self.cols {
                let l = i.saturating_sub(1);
                let r = (i + 1).min(self.cols - 1);
                self.patterns[j * self.cols + i] = (self.tiles[above + l] & 1)
                    | ((self.tiles[above + i] & 1) << 1)
                    | ((self.tiles[above + r] & 1) << 2)
                    | ((self.tiles[below + l] & 1) << 3)
                    | ((self.tiles[below + i] & 1) << 4)
                    | ((self.tiles[below + r] & 1) << 5);
            }
        }
    }
}
fn classify(xx: i32, yy: i32, xy: i32, sx: i32, sy: i32) -> u8 {
    let total = xx + yy;
    // Four rows of eight pair-sums of pixels; energies are in quarter-pixel units.
    if !(12 * 4 * 32..250 * 4 * 32).contains(&total) {
        return 0;
    }
    let tr = f64::from(total);
    let anis = f64::from(xx - yy).powi(2) + 4. * f64::from(xy).powi(2);
    u8::from(
        anis > 0.65 * 0.65 * tr * tr && f64::from(sx).powi(2) + f64::from(sy).powi(2) < 8. * tr,
    )
}

fn tile_class(y: &[u8], stride: usize, x: usize, p: usize) -> u8 {
    #[cfg(target_arch = "aarch64")]
    {
        unsafe { tile_neon(y.as_ptr().add(p * stride + x), stride) }
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        let mut xx = 0;
        let mut yy = 0;
        let mut xy = 0;
        let mut sx = 0;
        let mut sy = 0;
        for row in [2, 6, 10, 13] {
            let k = (p + row) * stride + x;
            for i in (0..16).step_by(2) {
                let pair = |j| i32::from(y[j]) + i32::from(y[j + 1]);
                let a = k + i;
                if y[a] >= 175 || y[a + 1] >= 175 {
                    return 2;
                }
                let gx = pair(a + 2) - pair(a - 2);
                let gy = pair(a + 2 * stride) - pair(a - 2 * stride);
                xx += gx * gx;
                yy += gy * gy;
                xy += gx * gy;
                sx += gx;
                sy += gy;
            }
        }
        classify(xx, yy, xy, sx, sy)
    }
}

#[cfg(target_arch = "aarch64")]
unsafe fn tile_neon(ptr: *const u8, stride: usize) -> u8 {
    use std::arch::aarch64::*;
    unsafe {
        let mut xx = vdupq_n_s32(0);
        let mut yy = xx;
        let mut xy = xx;
        let mut sx = vdupq_n_s16(0);
        let mut sy = sx;
        let mut light = vdupq_n_u8(0);
        for j in [2, 6, 10, 13] {
            let row = ptr.add(j * stride);
            light = vmaxq_u8(light, vld1q_u8(row));
            let pair = |v| vreinterpretq_s16_u16(vpaddlq_u8(vld1q_u8(v)));
            let gx = vsubq_s16(pair(row.add(2)), pair(row.sub(2)));
            let gy = vsubq_s16(pair(row.add(2 * stride)), pair(row.sub(2 * stride)));
            sx = vaddq_s16(sx, gx);
            sy = vaddq_s16(sy, gy);
            xx = vmlal_high_s16(vmlal_s16(xx, vget_low_s16(gx), vget_low_s16(gx)), gx, gx);
            yy = vmlal_high_s16(vmlal_s16(yy, vget_low_s16(gy), vget_low_s16(gy)), gy, gy);
            xy = vmlal_high_s16(vmlal_s16(xy, vget_low_s16(gx), vget_low_s16(gy)), gx, gy);
        }
        if vmaxvq_u8(light) >= 175 {
            return 2;
        }
        classify(
            vaddvq_s32(xx),
            vaddvq_s32(yy),
            vaddvq_s32(xy),
            vaddlvq_s16(sx),
            vaddlvq_s16(sy),
        )
    }
}
// A vector starts two pixels into a tile. Its first six pixels interpolate
// the left pair of tile centres and the other ten interpolate the right pair.
const VECTOR_WEIGHTS: [[[u8; 16]; 16]; 64] = {
    let mut result = [[[0; 16]; 16]; 64];
    let mut p = 0;
    while p < 64 {
        let mut y = 0;
        while y < 16 {
            let mut i = 0;
            while i < 16 {
                let q = if i < 6 { p } else { p >> 1 };
                let pattern = (q & 3) | ((q >> 1) & 12);
                result[p][y][i] = WEIGHTS[pattern][y][(i + 10) & 15];
                i += 1;
            }
            y += 1;
        }
        p += 1;
    }
    result
};
// Interpolate across tile centres. All 16 possible neighbourhoods fit in 2.3 KiB.
const WEIGHTS: [[[u8; TILE]; TILE]; 16] = {
    let mut table = [[[0; TILE]; TILE]; 16];
    let mut p = 0;
    while p < 16 {
        let mut y = 0;
        while y < TILE {
            let mut x = 0;
            while x < TILE {
                let fx = 2 * x + 1;
                let fy = 2 * y + 1;
                let unit = 2 * TILE;
                let sum = (p & 1) * (unit - fx) * (unit - fy)
                    + ((p >> 1) & 1) * fx * (unit - fy)
                    + ((p >> 2) & 1) * (unit - fx) * fy
                    + ((p >> 3) & 1) * fx * fy;
                table[p][y][x] = ((16 * sum + unit * unit / 2) / (unit * unit)) as u8;
                x += 1;
            }
            y += 1;
        }
        p += 1;
    }
    table
};

fn fits(data: &[u8], stride: usize, width: usize, height: usize) -> bool {
    stride >= width
        && height
            .checked_sub(1)
            .and_then(|last| last.checked_mul(stride))
            .and_then(|start| start.checked_add(width))
            .is_some_and(|end| end <= data.len())
}

pub(crate) fn filter_nv12(
    y: &mut [u8],
    y_stride: usize,
    uv: &mut [u8],
    uv_stride: usize,
    width: usize,
    height: usize,
    work: &mut Workspace,
) -> bool {
    if width == 0
        || height == 0
        || width % 2 != 0
        || height % 2 != 0
        || !fits(y, y_stride, width, height)
        || !fits(uv, uv_stride, width, height / 2)
    {
        return false;
    }
    if width > 4 && height > 4 {
        let Some(scratch_len) = width.checked_mul(4) else {
            return false;
        };
        work.rows.resize(scratch_len, 0);
        work.texture.prepare(y, y_stride, width, height);
        filter_plane::<2>(
            y,
            y_stride,
            width,
            height,
            &mut work.rows,
            &mut work.texture,
        );
        filter_plane::<1>(
            uv,
            uv_stride,
            width,
            height / 2,
            &mut work.rows,
            &mut work.texture,
        );
        if width > 12 && height / 2 > 6 {
            filter_plane::<3>(
                uv,
                uv_stride,
                width,
                height / 2,
                &mut work.rows,
                &mut work.texture,
            );
        }
    }
    true
}

// The luma and median kernels read two bytes to either side. The wide chroma
// kernel reads three UV pairs. R is the vertical radius in rows of the plane.
fn filter_plane<const R: usize>(
    data: &mut [u8],
    stride: usize,
    width: usize,
    height: usize,
    scratch: &mut [u8],
    texture: &mut Texture,
) {
    let margin = if R == 3 { 6 } else { 2 };
    let interior = width - 2 * margin;
    let slot = |row: usize| row % (R + 1) * width;
    let write_back = |data: &mut [u8], scratch: &[u8], row: usize| {
        let dst = row * stride + margin;
        let src = slot(row) + margin;
        data[dst..dst + interior].copy_from_slice(&scratch[src..src + interior]);
    };
    for row in R..height - R {
        let top = (row - R) * stride;
        let center = row * stride;
        let bottom = (row + R) * stride;
        let output = &mut scratch[slot(row)..slot(row) + width];
        let patterns = if R == 2 {
            let st = ((row + 8) >> 4) * texture.cols;
            &texture.patterns[st..st + texture.cols]
        } else {
            &[]
        };
        filter_row::<R>(
            &data[top..top + width],
            &data[center..center + width],
            &data[bottom..bottom + width],
            output,
            patterns,
            (row + 8) & 15,
        );
        if row >= 2 * R {
            write_back(data, scratch, row - R);
        }
    }
    for row in (height - 2 * R).max(R)..height - R {
        write_back(data, scratch, row);
    }
}

fn filter_row<const R: usize>(
    top: &[u8],
    mid: &[u8],
    bottom: &[u8],
    out: &mut [u8],
    patterns: &[u8],
    phase: usize,
) {
    let margin = if R == 3 { 6 } else { 2 };
    let end = out.len() - margin;
    #[cfg(target_arch = "aarch64")]
    if end - margin >= 16 {
        // SAFETY: every vector starts in margin..=end-16. All four rows have
        // identical visible widths and distinct input/output ownership.
        unsafe {
            let mut x = margin;
            while x + 16 <= end {
                let p = if R == 2 { patterns[(x - 2) >> 4] } else { 0 };
                neon::block::<R>(top, mid, bottom, out, x, p, phase);
                x += 16;
            }
            if x < end {
                if R == 2 {
                    for i in x..end {
                        let p = patterns[(i - 2) >> 4] as usize;
                        out[i] = luma(
                            top[i],
                            mid[i - 2],
                            mid[i],
                            mid[i + 2],
                            bottom[i],
                            VECTOR_WEIGHTS[p][phase][(i - 2) & 15],
                        );
                    }
                } else {
                    neon::block::<R>(top, mid, bottom, out, end - 16, 0, 0);
                }
            }
        }
        return;
    }
    for x in margin..end {
        out[x] = if R == 3 {
            wide_chroma(top, mid, bottom, x)
        } else if R == 2 {
            luma(
                top[x],
                mid[x - 2],
                mid[x],
                mid[x + 2],
                bottom[x],
                VECTOR_WEIGHTS[patterns[(x - 2) >> 4] as usize][phase][(x - 2) & 15],
            )
        } else {
            let mut nine = [
                top[x - 2],
                top[x],
                top[x + 2],
                mid[x - 2],
                mid[x],
                mid[x + 2],
                bottom[x - 2],
                bottom[x],
                bottom[x + 2],
            ];
            let (_, median, _) = nine.select_nth_unstable(4);
            ((u16::from(mid[x]) + u16::from(*median) + 1) / 2) as u8
        };
    }
}

// Evaluate Cb and Cr together so bypassing a colour boundary preserves its hue.
fn wide_chroma(top: &[u8], mid: &[u8], bottom: &[u8], x: usize) -> u8 {
    let dist = |a: &[u8], i: usize, b: &[u8], j: usize| {
        a[i].abs_diff(b[j]).max(a[i ^ 1].abs_diff(b[j ^ 1]))
    };
    if dist(mid, x - 6, mid, x + 6).max(dist(top, x, bottom, x)) >= CHROMA_EDGE {
        return mid[x];
    }
    let near = |row: &[u8], i: usize| {
        u16::from(if dist(mid, x, row, i) <= CHROMA_NEIGHBOUR {
            row[i]
        } else {
            mid[x]
        })
    };
    ((near(top, x) + near(bottom, x) + near(mid, x - 6) + near(mid, x + 6) + 2) / 4) as u8
}

const CHROMA_NEIGHBOUR: u8 = 10;
const CHROMA_EDGE: u8 = 15;
const NEIGHBOUR_LIMIT: u8 = 15;
const EDGE_THRESHOLD: u8 = 9;
const DETAIL_LIMIT: u8 = 2;

fn luma(up: u8, left: u8, value: u8, right: u8, down: u8, weight: u8) -> u8 {
    let horizontal = (u16::from(left) + u16::from(right)).div_ceil(2) as u8;
    let vertical = ((u16::from(up) + u16::from(down)) / 2) as u8;
    let edge = left
        .abs_diff(right)
        .max(up.abs_diff(down))
        .max(horizontal.abs_diff(vertical));
    let surround = (u16::from(horizontal) + u16::from(vertical)).div_ceil(2) as u8;
    let magnitude = value.abs_diff(surround).saturating_sub(1);
    let enhance = |d| {
        if value >= surround {
            value.saturating_add(d)
        } else {
            value.saturating_sub(d)
        }
    };
    let near = |other: u8| {
        u16::from(if other.abs_diff(value) <= NEIGHBOUR_LIMIT {
            other
        } else {
            value
        })
    };
    let smooth = (near(left) + near(right) + near(up) + near(down) + 2) / 4;
    let soft = ((smooth + u16::from(value)).div_ceil(2)) as u8;
    let mild = if edge >= EDGE_THRESHOLD {
        enhance((magnitude / 2).min(DETAIL_LIMIT))
    } else {
        soft
    };
    if weight == 0 {
        return mild;
    }
    let strong = if edge >= 7 {
        enhance(magnitude.min(4))
    } else {
        soft
    };
    ((u16::from(mild) * u16::from(16 - weight) + u16::from(strong) * u16::from(weight) + 8) / 16)
        as u8
}

#[cfg(target_arch = "aarch64")]
mod neon {
    use super::{CHROMA_EDGE, CHROMA_NEIGHBOUR, DETAIL_LIMIT, EDGE_THRESHOLD, NEIGHBOUR_LIMIT};
    use std::arch::aarch64::*;

    #[inline]
    unsafe fn luma16<const MODE: u8>(
        up: uint8x16_t,
        left: uint8x16_t,
        value: uint8x16_t,
        right: uint8x16_t,
        down: uint8x16_t,
        weight: uint8x16_t,
    ) -> uint8x16_t {
        unsafe {
            let horizontal = vrhaddq_u8(left, right);
            let vertical = vhaddq_u8(up, down);
            let edge = vmaxq_u8(
                vabdq_u8(horizontal, vertical),
                vmaxq_u8(vabdq_u8(left, right), vabdq_u8(up, down)),
            );
            let surround = vrhaddq_u8(horizontal, vertical);
            let detail = vminq_u8(
                vshrq_n_u8::<1>(vqsubq_u8(vabdq_u8(value, surround), vdupq_n_u8(1))),
                vdupq_n_u8(DETAIL_LIMIT),
            );
            let enhanced = vbslq_u8(
                vcgeq_u8(value, surround),
                vqaddq_u8(value, detail),
                vqsubq_u8(value, detail),
            );
            let near = |other| {
                vbslq_u8(
                    vcleq_u8(vabdq_u8(value, other), vdupq_n_u8(NEIGHBOUR_LIMIT)),
                    other,
                    value,
                )
            };
            let (l, r, u, d) = (near(left), near(right), near(up), near(down));
            let smooth = mean4(l, r, u, d);
            let soft = vrhaddq_u8(value, smooth);
            let mild = vbslq_u8(vcgeq_u8(edge, vdupq_n_u8(EDGE_THRESHOLD)), enhanced, soft);
            if MODE == 0 {
                return mild;
            }
            let amount = vminq_u8(
                vqsubq_u8(vabdq_u8(value, surround), vdupq_n_u8(1)),
                vdupq_n_u8(4),
            );
            let enhanced = vbslq_u8(
                vcgeq_u8(value, surround),
                vqaddq_u8(value, amount),
                vqsubq_u8(value, amount),
            );
            let strong = vbslq_u8(vcgeq_u8(edge, vdupq_n_u8(7)), enhanced, soft);
            if MODE == 1 {
                return strong;
            }
            let inv = vsubq_u8(vdupq_n_u8(16), weight);
            let lo = vmlal_u8(
                vmull_u8(vget_low_u8(mild), vget_low_u8(inv)),
                vget_low_u8(strong),
                vget_low_u8(weight),
            );
            let hi = vmlal_high_u8(vmull_high_u8(mild, inv), strong, weight);
            vcombine_u8(vrshrn_n_u16::<4>(lo), vrshrn_n_u16::<4>(hi))
        }
    }

    #[inline]
    unsafe fn mean4(l: uint8x16_t, r: uint8x16_t, u: uint8x16_t, d: uint8x16_t) -> uint8x16_t {
        unsafe {
            let lower = vaddq_u16(
                vaddl_u8(vget_low_u8(l), vget_low_u8(r)),
                vaddl_u8(vget_low_u8(u), vget_low_u8(d)),
            );
            let upper = vaddq_u16(vaddl_high_u8(l, r), vaddl_high_u8(u, d));
            vcombine_u8(vrshrn_n_u16::<2>(lower), vrshrn_n_u16::<2>(upper))
        }
    }

    #[inline]
    unsafe fn wide16(
        up: uint8x16_t,
        left: uint8x16_t,
        center: uint8x16_t,
        right: uint8x16_t,
        down: uint8x16_t,
    ) -> uint8x16_t {
        unsafe {
            let dist = |a, b| {
                let diff = vabdq_u8(a, b);
                vmaxq_u8(diff, vrev16q_u8(diff))
            };
            let near = |neighbour| {
                vbslq_u8(
                    vcleq_u8(dist(center, neighbour), vdupq_n_u8(CHROMA_NEIGHBOUR)),
                    neighbour,
                    center,
                )
            };
            let edge = vmaxq_u8(dist(up, down), dist(left, right));
            let smooth = mean4(near(left), near(right), near(up), near(down));
            vbslq_u8(vcgeq_u8(edge, vdupq_n_u8(CHROMA_EDGE)), center, smooth)
        }
    }

    // Order the samples by rows, then use only the comparisons needed for rank five.
    // SIMD lanes alternate between U and V, so the two channels remain independent.
    #[inline]
    unsafe fn median9(mut p: [uint8x16_t; 9]) -> uint8x16_t {
        unsafe {
            macro_rules! order {
                ($a:literal,$b:literal) => {{
                    let low = vminq_u8(p[$a], p[$b]);
                    p[$b] = vmaxq_u8(p[$a], p[$b]);
                    p[$a] = low;
                }};
            }
            order!(1, 2);
            order!(4, 5);
            order!(7, 8);
            order!(0, 1);
            order!(3, 4);
            order!(6, 7);
            order!(1, 2);
            order!(4, 5);
            order!(7, 8);
            order!(0, 3);
            order!(5, 8);
            order!(4, 7);
            order!(3, 6);
            order!(1, 4);
            order!(2, 5);
            order!(4, 7);
            order!(4, 2);
            order!(6, 4);
            order!(4, 2);
            p[4]
        }
    }

    #[inline]
    pub(super) unsafe fn block<const R: usize>(
        top: &[u8],
        mid: &[u8],
        bottom: &[u8],
        out: &mut [u8],
        x: usize,
        pattern: u8,
        phase: usize,
    ) {
        unsafe {
            let load = |row: &[u8], index: usize| vld1q_u8(row.as_ptr().add(index));
            let value = load(mid, x);
            let up = load(top, x);
            let down = load(bottom, x);
            let margin = if R == 3 { 6 } else { 2 };
            let left = load(mid, x - margin);
            let right = load(mid, x + margin);
            let result = if R == 3 {
                wide16(up, left, value, right, down)
            } else if R == 2 {
                match pattern {
                    0 => luma16::<0>(up, left, value, right, down, vdupq_n_u8(0)),
                    63 => luma16::<1>(up, left, value, right, down, vdupq_n_u8(16)),
                    _ => luma16::<2>(
                        up,
                        left,
                        value,
                        right,
                        down,
                        vld1q_u8(super::VECTOR_WEIGHTS[pattern as usize][phase].as_ptr()),
                    ),
                }
            } else {
                let median = median9([
                    load(top, x - 2),
                    up,
                    load(top, x + 2),
                    left,
                    value,
                    right,
                    load(bottom, x - 2),
                    down,
                    load(bottom, x + 2),
                ]);
                vrhaddq_u8(value, median)
            };
            vst1q_u8(out.as_mut_ptr().add(x), result);
        }
    }
}
