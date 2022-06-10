use image::{ImageBuffer, Rgb};
use rand::Rng;
use std::f64::consts::PI;
use std::time::{SystemTime};
/** 设置采样范围*/

const WIDTH: u32 = 512;
const HEIGHT: u32 = 512;
//N为采样方向(见sample函数)
const N: u32 = 64;
const MAX_DEPTH: u32 = 5;
const MAX_STEP: u32 = 64;
const MAX_DISTANCE: f64 = 5.0;
/**
设置光线步进长度
*/
const EPSILON: f64 = 1e-6;

const BIAS: f64 = 1e-4;
#[derive(Clone, Copy, Debug)]
struct Result {
    sd: f64,
    emissive: Color,
    //反射率
    reflectivity: f64,
    //折射率
    eta: f64,
    //吸收率
    absorption: Color,
}
#[derive(Clone, Copy, Debug)]
struct Color {
    r: f64,
    g: f64,
    b: f64,
}

fn color_add(a: Color, b: Color) -> Color {
    Color {
        r: a.r + b.r,
        g: a.g + b.g,
        b: a.b + b.b,
    }
}
fn color_mul(a: Color, b: Color) -> Color {
    Color {
        r: a.r * b.r,
        g: a.g * b.g,
        b: a.b * b.b,
    }
}
fn color_scale(a: Color, s: f64) -> Color {
    Color {
        r: a.r * s,
        g: a.g * s,
        b: a.b * s,
    }
}
//并集
fn union_op(a: Result, b: Result) -> Result {
    if a.sd < b.sd {
        a
    } else {
        b
    }
}
//交集
// fn intersection_op(a: Result, b: Result) -> Result {
//     let mut r = if a.sd > b.sd { b } else { a };
//     r.sd = f64::max(a.sd, b.sd);
//     r
// }
//补集
fn subtract_op(a: Result, b: Result) -> Result {
    let mut r = a;
    r.sd = f64::max(a.sd, -b.sd);
    r
}
//创建结构
fn build_rsult(a: f64) -> Result {
    Result {
        sd: a,
        emissive: Color { r: 0.0, g:0.0, b: 0.0 },
        reflectivity: 0.2,
        eta: 1.5,
        absorption: Color { r: 4.0, g:4.0, b: 4.0 },
    }
}
fn scene(x: f64, y: f64) -> Result {
    let align_line = build_rsult(box_sdf(x, y, 0.4, 0.25, 2.0 * PI / 4.0, 0.05, 0.25));
    let vertical_line = build_rsult(box_sdf(x, y, 0.46, 0.6, 2.0 * PI / 20.0, 0.05, 0.35));
    let left_top_corner = build_rsult(box_sdf(x, y, 0.1, 0.3, 2.0 * PI / 20.0, 0.05, 0.36));
    let bottom_corner = build_rsult(box_sdf(x, y, 0.4, 0.9, 2.0 * PI / 4.0, 0.05, 0.25));
    let triangle_right_top = build_rsult(triangle_sdf(x, y, 0.55, 0.13, 0.5, 0.3, 0.9, 0.3));

    let x = (x - 0.5).abs() + 0.5;
    let y = (y - 0.5).abs() + 0.5;
    let light = Result {
        sd: circle_sdf(x, y, -0.05, -0.05, 0.05),
        emissive: Color { r: 6.0, g:6.0, b: 6.0 },
        reflectivity: 0.0,
        eta: 0.0,
        absorption: Color { r: 0.0, g:0.0, b: 0.0 },
    };

    let rs = union_op(
        subtract_op(
            subtract_op(
                union_op(union_op(vertical_line, triangle_right_top), align_line),
                left_top_corner,
            ),
            bottom_corner,
        ),
        light,
    );
    rs
}
//矩形sdf
fn box_sdf(x: f64, y: f64, cx: f64, cy: f64, theta: f64, sx: f64, sy: f64) -> f64 {
    let costheta = theta.cos();
    let sintheta = theta.sin();
    let dx = ((x - cx) * costheta + (y - cy) * sintheta).abs() - sx;
    let dy = ((y - cy) * costheta - (x - cx) * sintheta).abs() - sy;
    let ax = if dx > 0.0 { dx } else { 0.0 };
    let ay = if dy > 0.0 { dy } else { 0.0 };

    let rs = f64::min(f64::max(dx, dy), 0.0) as f64 + (ax * ax + ay * ay).sqrt();
    rs
}

//三角形sdf
fn triangle_sdf(x: f64, y: f64, ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> f64 {
    let d = f64::min(
        f64::min(
            segment_sdf(x, y, ax, ay, bx, by),
            segment_sdf(x, y, bx, by, cx, cy),
        ),
        segment_sdf(x, y, cx, cy, ax, ay),
    );
    let rs = if (bx - ax) * (y - ay) > (by - ay) * (x - ax)
        && (cx - bx) * (y - by) > (cy - by) * (x - bx)
        && (ax - cx) * (y - cy) > (ay - cy) * (x - cx)
    {
        -d
    } else {
        d
    };
    rs
}
//平面sdf
// fn plane_sdf(x: f64, y: f64, px: f64, py: f64, nx: f64, ny: f64) -> f64 {
//     (x - px) * nx + (y - py) * ny
// }
//线段sdf
fn segment_sdf(x: f64, y: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let vx = x - ax;
    let vy = y - ay;
    let ux = bx - ax;
    let uy = by - ay;
    let t = f64::max(
        f64::min((vx * ux + vy * uy) / (ux * ux + uy * uy), 1.0),
        0.0,
    );
    let dx = vx - ux * t;
    let dy = vy - uy * t;
    (dx * dx + dy * dy).sqrt()
}
//胶囊体sdf
// fn capsule_sdf(x: f64, y: f64, ax: f64, ay: f64, bx: f64, by: f64, r: f64) -> f64 {
//     segment_sdf(x, y, ax, ay, bx, by) - r
// }

fn circle_sdf(x: f64, y: f64, cx: f64, cy: f64, r: f64) -> f64 {
    let ux: f64 = x - cx;
    let uy: f64 = y - cy;
    (ux * ux + uy * uy).sqrt() - r
}
//折射求解
fn refract(ix: f64, iy: f64, nx: f64, ny: f64, eta: f64) -> (bool, f64, f64) {
    let idotn = ix * nx + iy * ny;
    let k = 1.0 - eta * eta * (1.0 - idotn * idotn);
    if k < 0.0 {
        return (false, 0.0, 0.0);
    } else {
        let a = eta * idotn + k.sqrt();
        let rx = eta * ix - a * nx;
        let ry = eta * iy - a * ny;
        return (true, rx, ry);
    }
}
//斯涅尔定律
fn fresnel(cosi: f64, cost: f64, etai: f64, etat: f64) -> f64 {
    let rs = (etat * cosi - etai * cost) / (etat * cosi + etai * cost);
    let rp = (etai * cosi - etat * cost) / (etai * cosi + etat * cost);
    let t = (rs * rs + rp * rp) * 0.5;
    t
}
//比尔-朗伯定律
fn beer_lambert(a: Color, d: f64) -> Color  {
    Color { 
        r:(-a.r * d).exp(),
        g:(-a.g*d ).exp(),
        b:(-a.b * d ).exp(),
    }
}
//反射求解
fn reflect(ix: f64, iy: f64, nx: f64, ny: f64) -> (f64, f64) {
    let dot2 = (ix * nx + iy * ny) * 2.0;
    (ix - dot2 * nx, iy - dot2 * ny)
}
//梯度函数
fn gradient(x: f64, y: f64) -> (f64, f64) {
    (
        (scene(x + EPSILON, y).sd - scene(x - EPSILON, y).sd) * (0.5 / EPSILON),
        (scene(x, y + EPSILON).sd - scene(x, y - EPSILON).sd) * (0.5 / EPSILON),
    )
}
fn trace(ox: f64, oy: f64, dx: f64, dy: f64, depth: u32) -> Color {
    let mut t: f64 = 0.0;
    //判断是否到达表面
    let sign = if scene(ox, oy).sd > 0.0 { 1.0 } else { -1.0 };
    for _i in 0..MAX_STEP {
        let x = ox + dx * t;
        let y = oy + dy * t;
        let r = scene(x, y);
        if r.sd * sign < EPSILON {
            let mut sum = r.emissive;
            if depth < MAX_DEPTH && (r.reflectivity > 0.0 || r.eta > 0.0) {
                let mut refl = r.reflectivity;
                let (mut nx, mut ny) = gradient(x, y);
                nx *= sign;
                ny *= sign;
                if r.eta > 0.0 {
                    let refl_eta = if sign < 0.0 { r.eta } else { 1.0 / r.eta };
                    let (flag, rx, ry) = refract(dx, dy, nx, ny, refl_eta);
                    if flag {
                        let cosi = -(dx * nx + dy * ny);
                        let cost = -(rx * nx + ry * ny);
                        refl = if sign < 0.0 {
                            fresnel(cosi, cost, r.eta, 1.0)
                        } else {
                            fresnel(cosi, cost, 1.0, r.eta)
                        };
                        sum = color_add(sum, color_scale(trace(x - nx * BIAS, y - ny * BIAS, rx, ry, depth + 1), 1.0 - refl));
                    } else {
                        refl = 1.0;
                    }
                }
                if refl > 0.0 {
                    let (rx, ry) = reflect(dx, dy, nx, ny);
                    sum = color_add(sum, color_scale(trace(x + nx * BIAS, y + ny * BIAS, rx, ry, depth + 1), refl));
                }
            }
            //采样点小于阈值，返回自发光强度
            return color_mul(sum, beer_lambert(r.absorption, t));
        }
        t += r.sd;
        if t > MAX_DISTANCE {
            break;
        }
    }
    Color { r: 0.0, g:0.0, b: 0.0 }
}
//运用蒙地卡罗积分实现64个方向均匀采样
fn sample(x: f64, y: f64) -> Color {
    let mut sum: Color =  Color { r: 0.0, g:0.0, b: 0.0 };
    for i in 0..N {
        //随机方向
        let tmp = 2.0 * PI * (i as f64 + rand::thread_rng().gen_range(0.0..1.0)) / N as f64;
        sum = color_add(sum, trace(x, y, tmp.cos(), tmp.sin(), 0));
    }
    color_scale(sum, 1.0 / N as f64)
}

fn main() {
    let mut img = ImageBuffer::from_pixel(WIDTH, HEIGHT, Rgb([0u8, 0u8, 0u8]));
    for x in 0..WIDTH {
        for y in 0..HEIGHT {
            let xx = x as f64 / WIDTH as f64;
            let yy = y as f64 / HEIGHT as f64;
            let color = sample(xx, yy);
            if x > WIDTH {
                continue;
            }
            if y > HEIGHT {
                continue;
            }
            img.put_pixel(x, y, Rgb([f64::min(color.r* 255.0, 255.0) as u8, f64::min(color.g* 255.0, 255.0) as u8, f64::min(color.b* 255.0, 255.0) as u8]));
        }
    }
    let after = SystemTime::now();
    println!("{:?}", after);
    img.save("out.png").unwrap();
}
