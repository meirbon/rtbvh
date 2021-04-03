use glam::*;
use l3d::prelude::*;
use rayon::prelude::*;
use rtbvh::builders::spatial_sah::SpatialTriangle;
use rtbvh::*;
use std::ops::BitAnd;
use std::path::PathBuf;
use std::time::{Duration, Instant};

#[derive(Debug, Copy, Clone)]
struct Triangle {
    vertex0: Vec4,
    vertex1: Vec4,
    vertex2: Vec4,
}

impl Triangle {
    fn intersect4(&self, packet: &mut RayPacket4, t_min: &[f32; 4]) -> Option<[bool; 4]> {
        let zero = Vec4::ZERO;
        let one = Vec4::ONE;

        let org_x = Vec4::from(packet.origin_x);
        let org_y = Vec4::from(packet.origin_y);
        let org_z = Vec4::from(packet.origin_z);

        let dir_x = Vec4::from(packet.direction_x);
        let dir_y = Vec4::from(packet.direction_y);
        let dir_z = Vec4::from(packet.direction_z);

        let p0_x = self.vertex0.xxxx();
        let p0_y = self.vertex0.yyyy();
        let p0_z = self.vertex0.zzzz();

        let p1_x = self.vertex1.xxxx();
        let p1_y = self.vertex1.yyyy();
        let p1_z = self.vertex1.zzzz();

        let p2_x = self.vertex2.xxxx();
        let p2_y = self.vertex2.yyyy();
        let p2_z = self.vertex2.zzzz();

        let edge1_x = p1_x - p0_x;
        let edge1_y = p1_y - p0_y;
        let edge1_z = p1_z - p0_z;

        let edge2_x = p2_x - p0_x;
        let edge2_y = p2_y - p0_y;
        let edge2_z = p2_z - p0_z;

        let h_x = (dir_y * edge2_z) - (dir_z * edge2_y);
        let h_y = (dir_z * edge2_x) - (dir_x * edge2_z);
        let h_z = (dir_x * edge2_y) - (dir_y * edge2_x);

        let a = (edge1_x * h_x) + (edge1_y * h_y) + (edge1_z * h_z);
        let epsilon = Vec4::from([1e-6; 4]);
        let mask = a.cmple(-epsilon) | a.cmpge(epsilon);
        if mask.bitmask() == 0 {
            return None;
        }

        let f = one / a;
        let s_x = org_x - p0_x;
        let s_y = org_y - p0_y;
        let s_z = org_z - p0_z;

        let u = f * ((s_x * h_x) + (s_y * h_y) + (s_z * h_z));
        let mask = mask.bitand(u.cmpge(zero) & u.cmple(one));
        if mask.bitmask() == 0 {
            return None;
        }

        let q_x = s_y * edge1_z - s_z * edge1_y;
        let q_y = s_z * edge1_x - s_x * edge1_z;
        let q_z = s_x * edge1_y - s_y * edge1_x;

        let v = f * ((dir_x * q_x) + (dir_y * q_y) + (dir_z * q_z));
        let mask = mask.bitand(v.cmpge(zero) & (u + v).cmple(one));
        if mask.bitmask() == 0 {
            return None;
        }

        let t_min = Vec4::from(*t_min);

        let t_value = f * ((edge2_x * q_x) + (edge2_y * q_y) + (edge2_z * q_z));
        let mask = mask.bitand(t_value.cmpge(t_min) & t_value.cmplt(packet.t.into()));
        let bitmask = mask.bitmask();
        if bitmask == 0 {
            return None;
        }

        packet.t = Vec4::select(mask, t_value, packet.t.into()).into();

        Some([
            bitmask & 1 != 0,
            bitmask & 2 != 0,
            bitmask & 4 != 0,
            bitmask & 8 != 0,
        ])
    }
}

impl Primitive for Triangle {
    fn center(&self) -> [f32; 3] {
        ((self.vertex0.xyz() + self.vertex1.xyz() + self.vertex2.xyz()) * (1.0 / 3.0)).into()
    }

    fn aabb(&self) -> Aabb {
        let mut aabb = Aabb::empty();
        aabb.grow(self.vertex0.xyz());
        aabb.grow(self.vertex1.xyz());
        aabb.grow(self.vertex2.xyz());
        aabb
    }
}

impl SpatialTriangle for Triangle {
    fn vertex0(&self) -> [f32; 3] {
        self.vertex0.xyz().into()
    }

    fn vertex1(&self) -> [f32; 3] {
        self.vertex1.xyz().into()
    }

    fn vertex2(&self) -> [f32; 3] {
        self.vertex2.xyz().into()
    }
}

pub struct Timer {
    moment: Instant,
}

impl Timer {
    pub fn new() -> Timer {
        Timer {
            moment: Instant::now(),
        }
    }

    pub fn reset(&mut self) {
        self.moment = Instant::now();
    }

    pub fn elapsed(&self) -> Duration {
        self.moment.elapsed()
    }

    pub fn elapsed_in_millis(&self) -> f32 {
        let elapsed = self.elapsed();
        let secs = elapsed.as_secs() as u32;
        let millis = elapsed.subsec_micros();
        (secs * 1_000) as f32 + (millis as f32 / 1000.0)
    }
}

impl Default for Timer {
    fn default() -> Self {
        Self::new()
    }
}

fn main() {
    let loader = l3d::LoadInstance::new().with_default();
    let result = loader.load(LoadOptions {
        path: PathBuf::from("objects/teapot.obj"),
        ..Default::default()
    });

    let mesh = match result {
        LoadResult::Mesh(m) => m,
        LoadResult::Scene(_) => panic!(),
        LoadResult::None(_) => panic!(),
    };

    let primitives = mesh
        .vertices
        .chunks_exact(3)
        .map(|c| Triangle {
            vertex0: Vec4::from(c[0]),
            vertex1: Vec4::from(c[1]),
            vertex2: Vec4::from(c[2]),
        })
        .collect::<Vec<Triangle>>();
    let aabbs = primitives.iter().map(|t| t.aabb()).collect::<Vec<Aabb>>();

    let bvh = Builder {
        aabbs: aabbs.as_slice(),
        primitives: primitives.as_slice(),
    }
    .construct_spatial_sah();

    // Put ray in middle of teapot
    let origin = Vec3::new(0.0, 1.5, 0.0);
    let direction = Vec3::Z;

    const EPSILON: f32 = 1e-6;

    let timer = Timer::new();
    (0..1_000_000).into_iter().for_each(|_| {
        let _hit_id = bvh.traverse(
            origin.as_ref(),
            direction.as_ref(),
            1e-4,
            1e34,
            |id, t_min, t_max| -> Option<(f32, usize)> {
                let triangle = unsafe { primitives.get_unchecked(id) };
                let edge1 = triangle.vertex1.truncate() - triangle.vertex0.truncate();
                let edge2 = triangle.vertex2.truncate() - triangle.vertex0.truncate();
                let h_val = direction.cross(edge2);
                let a_val = edge1.dot(h_val);
                if a_val > -EPSILON && a_val < EPSILON {
                    return None;
                }
                let f_val = 1.0 / a_val;
                let s_val = origin - triangle.vertex0.xyz();
                let u_val = f_val * s_val.dot(h_val);
                if !(0.0..=1.0).contains(&u_val) {
                    return None;
                }
                let q_val = s_val.cross(edge1);
                let v_val = f_val * direction.dot(q_val);
                if v_val < 0. || (u_val + v_val) > 1.0 {
                    return None;
                }

                let t: f32 = f_val * edge2.dot(q_val);
                if t > t_min && t < t_max {
                    return Some((t, id));
                }
                None
            },
        );
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "Single-threaded rays: 1.000.000 rays in {} ms, {} million rays per second",
        elapsed,
        1000.0 / elapsed
    );

    let timer = Timer::new();
    (0..1_000_000).into_par_iter().for_each(|_| {
        let _hit_id = bvh.traverse(
            origin.as_ref(),
            direction.as_ref(),
            1e-4,
            1e34,
            |id, t_min, t_max| -> Option<(f32, usize)> {
                let triangle = unsafe { primitives.get_unchecked(id) };
                let edge1 = triangle.vertex1.xyz() - triangle.vertex0.xyz();
                let edge2 = triangle.vertex2.xyz() - triangle.vertex0.xyz();
                let h_val = direction.cross(edge2);
                let a_val = edge1.dot(h_val);
                if a_val > -EPSILON && a_val < EPSILON {
                    return None;
                }
                let f_val = 1.0 / a_val;
                let s_val = origin - triangle.vertex0.xyz();
                let u_val = f_val * s_val.dot(h_val);
                if !(0.0..=1.0).contains(&u_val) {
                    return None;
                }
                let q_val = s_val.cross(edge1);
                let v_val = f_val * direction.dot(q_val);
                if v_val < 0. || (u_val + v_val) > 1.0 {
                    return None;
                }

                let t: f32 = f_val * edge2.dot(q_val);
                if t > t_min && t < t_max {
                    return Some((t, id));
                }
                None
            },
        );
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "{} threads rays: 1.000.000 rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        elapsed,
        1000.0 / elapsed
    );

    let origin_x = [0.0; 4];
    let origin_y = [1.5; 4];
    let origin_z = [0.0; 4];
    let direction_x = [0.0; 4];
    let direction_y = [0.0; 4];
    let direction_z = [1.0; 4];

    let timer = Timer::new();
    (0..250_000).into_iter().for_each(|_| {
        let mut packet = RayPacket4 {
            origin_x,
            origin_y,
            origin_z,
            direction_x,
            direction_y,
            direction_z,
            pixel_ids: [0; 4],
            t: [1e34; 4],
        };

        bvh.traverse4(&mut packet, |id, packet| {
            let t = unsafe { primitives.get_unchecked(id) };
            let _result = t.intersect4(packet, &[1e-5; 4]);
        });
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "Single-threaded packets: 1.000.000 rays in {} ms, {} million rays per second",
        elapsed,
        1000.0 / elapsed
    );

    let timer = Timer::new();
    (0..250_000).into_par_iter().for_each(|_| {
        let mut packet = RayPacket4 {
            origin_x,
            origin_y,
            origin_z,
            direction_x,
            direction_y,
            direction_z,
            pixel_ids: [0; 4],
            t: [1e34; 4],
        };

        bvh.traverse4(&mut packet, |id, packet| {
            let t = unsafe { primitives.get_unchecked(id) };
            let _result = t.intersect4(packet, &[1e-5; 4]);
        });
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "{} threads packets: 1.000.000 rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        elapsed,
        1000.0 / elapsed
    );
}
