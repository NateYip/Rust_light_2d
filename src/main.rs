use image::{ImageBuffer, Rgb};
use rand::Rng;
use std::f64::consts::PI;

/** 设置采样范围*/

const WIDTH: u32 = 1024;
const HEIGHT: u32 = 1024;
//N为采样方向(见sample函数)
const N: u32 = 256;
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
    emissive: f64,
    //反射率
    reflectivity: f64,
    //折射率
    eta: f64,
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
fn intersection_op(a: Result, b: Result) -> Result {
    let mut r = if a.sd > b.sd { b } else { a };
    r.sd = f64::max(a.sd, b.sd);
    r
}
//补集
fn subtract_op(a: Result, b: Result) -> Result {
    let mut r = a;
    r.sd = f64::max(a.sd, -b.sd);
    r
}

fn scene(x: f64, y: f64) -> Result {
    
    let align_line = Result {
        sd: box_sdf(x,y,0.4,0.25, 2.0* PI/4.0 ,0.05,0.25), 
        emissive:0.0,
        reflectivity:0.2,
        eta:1.5
    };
    let vertical_line = Result {
        sd: box_sdf(x,y,0.46,0.6, 2.0* PI/20.0 ,0.05,0.35), 
        emissive:0.0,
        reflectivity:0.2,
        eta:1.5
    };
    let left_top_corner = Result {
        sd: box_sdf(x,y,0.1,0.3, 2.0* PI/20.0 ,0.05,0.36), 
        emissive:0.0,
        reflectivity:0.2,
        eta:1.5
    };
    let bottom_corner = Result {
        sd: box_sdf(x,y,0.4,0.9, 2.0* PI/4.0 ,0.05,0.25), 
        emissive:0.0,
        reflectivity:0.2,
        eta:1.5
    };
    let triangle_right_top = Result {
        sd : triangle_sdf(x, y, 0.55,0.13,0.5,0.3,0.9,0.3),
        emissive:0.0,
        reflectivity:0.2,
        eta:1.5
    };    
    let lx =(x - 0.5).abs() + 0.5;
    let ly = (y - 0.5).abs() + 0.5;
    let light = Result {
        sd : circle_sdf(lx,ly,1.05 ,1.05,0.05),
        emissive:6.0,
        reflectivity:0.0,
        eta:0.0,
    };
    let rs =union_op( 
            subtract_op(
                subtract_op(
                    union_op( union_op(vertical_line , triangle_right_top) 
                    , align_line)
                , left_top_corner)
            ,bottom_corner)
            ,light);
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
fn plane_sdf(x: f64, y: f64, px: f64, py: f64, nx: f64, ny: f64) -> f64 {
    (x - px) * nx + (y - py) * ny
}
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
fn capsule_sdf(x: f64, y: f64, ax: f64, ay: f64, bx: f64, by: f64, r: f64) -> f64 {
    segment_sdf(x, y, ax, ay, bx, by) - r
}

fn circle_sdf(x: f64, y: f64, cx: f64, cy: f64, r: f64) -> f64 {
    let ux: f64 = x - cx;
    let uy: f64 = y - cy;
    (ux * ux + uy * uy).sqrt() - r
}
//折射求解
fn refract(ix: f64, iy: f64, nx: f64, ny: f64, eta: f64, rx: &mut f64, ry: &mut f64) -> f64 {
    let idotn = ix * nx + iy * ny;
    let k = 1.0 - eta * eta * (1.0 - idotn * idotn);
    if k < 0.0 {
        return 0.0;
    } else {
        let a = eta * idotn + k.sqrt();
        *rx = eta * ix - a * nx;
        *ry = eta * iy - a * ny;
        return 1.0;
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
fn trace(ox: f64, oy: f64, dx: f64, dy: f64, depth: u32) -> f64 {
    let mut tmp: f64 = 0.0;
    //判断是否到达表面
    let sign = if scene(ox, oy).sd > 0.0 { 1.0 } else { -1.0 };
    for _i in 0..MAX_STEP {
        let x = ox + dx * tmp;
        let y = oy + dy * tmp;
        let r = scene(x, y);
        if r.sd * sign < EPSILON {
            let mut sum = r.emissive;
            if depth < MAX_DEPTH && (r.reflectivity > 0.0 || r.eta > 0.0) {
                let mut refl = r.reflectivity;
                let mut rx = 0.0;
                let mut ry = 0.0;
                let (mut nx, mut ny) = gradient(x, y);
                nx *= sign;
                ny *= sign;
                if r.eta > 0.0 {
                    let refl_eta = if sign < 0.0 { r.eta } else { 1.0 };
                    if refract(dx, dy, nx, ny, refl_eta, &mut rx, &mut ry) == 1.0 {
                        sum +=
                            (1.0 - refl) * trace(x - nx * BIAS, y - ny * BIAS, rx, ry, depth + 1);
                    } else {
                        refl = 1.0;
                    }
                }
                if refl > 0.0 {
                    (rx, ry) = reflect(dx, dy, nx, ny);
                    sum += refl * trace(x + nx * BIAS, y + ny * BIAS, rx, ry, depth + 1);
                }
            }
            //采样点小于阈值，返回自发光强度
            return sum;
        }
        tmp += r.sd;
        if tmp > MAX_DISTANCE {
            break;
        }
    }
    0.0
}
//运用蒙地卡罗积分实现64个方向均匀采样
fn sample(x: f64, y: f64) -> f64 {
    let mut sum: f64 = 0.0;
    for i in 0..N {
        //随机方向
        let tmp = 2.0 * PI * (i as f64 + rand::thread_rng().gen_range(0.0..1.0)) / N as f64;
        sum += trace(x, y, tmp.cos(), tmp.sin(), 0);
    }
    sum / (N as f64)
}

fn main() {
    let mut img = ImageBuffer::from_pixel(WIDTH, HEIGHT, Rgb([0u8, 0u8, 0u8]));
    for x in 0..WIDTH {
        for y in 0..HEIGHT {
            let xx = x as f64 / WIDTH as f64;
            let yy = y as f64 / HEIGHT as f64;
            let color = sample(xx, yy);
            let color = f64::min(color * 255.0, 255.0) as u8;
            if x > WIDTH {
                continue;
            }
            if y > HEIGHT {
                continue;
            }
            img.put_pixel(x, y, Rgb([color, color, color]));
        }
    }
    img.save("out.png").unwrap();
}
