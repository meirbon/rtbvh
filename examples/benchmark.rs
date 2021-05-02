use glam::*;
use l3d::prelude::*;
use rayon::prelude::*;
use rtbvh::{spatial_sah::SpatialTriangle, Primitive};
use std::path::PathBuf;
use std::time::{Duration, Instant};

#[repr(align(64))]
#[derive(Debug, Copy, Clone)]
struct Triangle {
    vertex0: Vec4,
    vertex1: Vec4,
    vertex2: Vec4,
    id: usize,
}

const PRIMS_PER_LEAF: usize = 1;
const RAYS: usize = 10_000_000;

impl bvh::aabb::Bounded for Triangle {
    fn aabb(&self) -> bvh::aabb::AABB {
        bvh::aabb::AABB::empty()
            .grow(&(*self.vertex0.xyz().as_ref()).into())
            .grow(&(*self.vertex1.xyz().as_ref()).into())
            .grow(&(*self.vertex2.xyz().as_ref()).into())
    }
}

impl bvh::bounding_hierarchy::BHShape for Triangle {
    fn set_bh_node_index(&mut self, id: usize) {
        self.id = id;
    }

    fn bh_node_index(&self) -> usize {
        self.id
    }
}

impl rtbvh::Primitive for Triangle {
    fn center(&self) -> Vec3 {
        (self.vertex0.xyz() + self.vertex1.xyz() + self.vertex2.xyz()) * (1.0 / 3.0)
    }

    fn aabb(&self) -> rtbvh::Aabb {
        let mut aabb = rtbvh::Aabb::empty();
        aabb.grow(self.vertex0.xyz());
        aabb.grow(self.vertex1.xyz());
        aabb.grow(self.vertex2.xyz());
        aabb
    }
}

impl SpatialTriangle for Triangle {
    fn vertex0(&self) -> Vec3 {
        self.vertex0.xyz()
    }

    fn vertex1(&self) -> Vec3 {
        self.vertex1.xyz()
    }

    fn vertex2(&self) -> Vec3 {
        self.vertex2.xyz()
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

    let mut primitives = mesh
        .vertices
        .chunks_exact(3)
        .enumerate()
        .map(|(id, c)| Triangle {
            vertex0: Vec4::from(c[0]),
            vertex1: Vec4::from(c[1]),
            vertex2: Vec4::from(c[2]),
            id,
        })
        .collect::<Vec<Triangle>>();
    let aabbs = primitives
        .iter()
        .map(|t| t.aabb())
        .collect::<Vec<rtbvh::Aabb>>();

    // Put ray in middle of teapot
    let origin = Vec3::new(0.0, 1.5, 0.0);
    let direction = Vec3::Z;

    let timer = Timer::default();
    let bvh = rtbvh::Builder {
        aabbs: aabbs.as_slice(),
        primitives: primitives.as_slice(),
        primitives_per_leaf: PRIMS_PER_LEAF,
    }
    .construct_spatial_sah();

    println!(
        "Bvh construction with spatial sah type of {} primitives took {} ms",
        primitives.len(),
        timer.elapsed_in_millis()
    );

    let timer = Timer::new();
    (0..RAYS).into_iter().for_each(|_| {
        let mut ray = rtbvh::Ray::new(origin, direction);
        for (triangle, ray) in bvh.traverse_iter(&mut ray, &primitives) {
            triangle.intersect(ray);
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "Single-threaded rays: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let timer = Timer::new();
    (0..RAYS).into_par_iter().for_each(|_| {
        let mut ray = rtbvh::Ray::new(origin, direction);
        for (triangle, ray) in bvh.traverse_iter(&mut ray, &primitives) {
            triangle.intersect(ray);
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "{} threads rays: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let origin_x = Vec4::from([0.0; 4]);
    let origin_y = Vec4::from([1.5; 4]);
    let origin_z = Vec4::from([0.0; 4]);
    let direction_x = Vec4::from([0.0; 4]);
    let direction_y = Vec4::from([0.0; 4]);
    let direction_z = Vec4::from([1.0; 4]);

    let timer = Timer::new();
    (0..(RAYS / 3)).into_iter().for_each(|_| {
        let mut packet = rtbvh::RayPacket4 {
            origin_x,
            origin_y,
            origin_z,
            direction_x,
            direction_y,
            direction_z,
            inv_direction_x: Vec4::ONE / direction_x,
            inv_direction_y: Vec4::ONE / direction_y,
            inv_direction_z: Vec4::ONE / direction_z,
            pixel_ids: UVec4::ZERO,
            t: [1e34; 4].into(),
        };

        for (triangle, packet) in bvh.traverse_iter_packet(&mut packet, &primitives) {
            triangle.intersect4(packet, Vec4::splat(1e-4));
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "Single-threaded packets: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let timer = Timer::new();
    (0..(RAYS / 4)).into_par_iter().for_each(|_| {
        let mut packet = rtbvh::RayPacket4 {
            origin_x,
            origin_y,
            origin_z,
            direction_x,
            direction_y,
            direction_z,
            inv_direction_x: Vec4::ONE / direction_x,
            inv_direction_y: Vec4::ONE / direction_y,
            inv_direction_z: Vec4::ONE / direction_z,
            pixel_ids: [0; 4].into(),
            t: [1e34; 4].into(),
        };

        for (triangle, packet) in bvh.traverse_iter_packet(&mut packet, &primitives) {
            triangle.intersect4(packet, Vec4::splat(1e-4));
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "{} threads packets: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    println!();
    let timer = Timer::default();
    let mbvh = rtbvh::Mbvh::from(bvh);
    println!("Mbvh construction took {} ms", timer.elapsed_in_millis());

    let timer = Timer::new();
    (0..RAYS).into_iter().for_each(|_| {
        let mut ray = rtbvh::Ray::new(origin, direction);
        for (triangle, ray) in mbvh.traverse_iter(&mut ray, &primitives) {
            triangle.intersect(ray);
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "Single-threaded rays: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let timer = Timer::new();
    (0..RAYS).into_par_iter().for_each(|_| {
        let mut ray = rtbvh::Ray::new(origin, direction);
        for (triangle, ray) in mbvh.traverse_iter(&mut ray, &primitives) {
            triangle.intersect(ray);
        }
    });

    let elapsed = timer.elapsed_in_millis();
    println!(
        "{} threads rays: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let timer = Timer::new();
    (0..(RAYS / 4)).into_iter().for_each(|_| {
        let mut packet = rtbvh::RayPacket4 {
            origin_x,
            origin_y,
            origin_z,
            direction_x,
            direction_y,
            direction_z,
            inv_direction_x: Vec4::ONE / direction_x,
            inv_direction_y: Vec4::ONE / direction_y,
            inv_direction_z: Vec4::ONE / direction_z,
            pixel_ids: UVec4::ZERO,
            t: [1e34; 4].into(),
        };

        for (triangle, packet) in mbvh.traverse_iter_packet(&mut packet, &primitives) {
            triangle.intersect4(packet, Vec4::splat(1e-4));
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "Single-threaded packets: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let timer = Timer::new();
    (0..(RAYS / 4)).into_par_iter().for_each(|_| {
        let mut packet = rtbvh::RayPacket4 {
            origin_x,
            origin_y,
            origin_z,
            direction_x,
            direction_y,
            direction_z,
            inv_direction_x: Vec4::ONE / direction_x,
            inv_direction_y: Vec4::ONE / direction_y,
            inv_direction_z: Vec4::ONE / direction_z,
            pixel_ids: [0; 4].into(),
            t: [1e34; 4].into(),
        };

        for (triangle, packet) in mbvh.traverse_iter_packet(&mut packet, &primitives) {
            triangle.intersect4(packet, Vec4::splat(1e-4));
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "{} threads packets: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );
    println!();

    // binned sah bvh
    let timer = Timer::default();
    let bvh = rtbvh::Builder {
        aabbs: aabbs.as_slice(),
        primitives: primitives.as_slice(),
        primitives_per_leaf: PRIMS_PER_LEAF,
    }
    .construct_binned_sah();

    println!(
        "Bvh construction with spatial sah type of {} primitives took {} ms",
        primitives.len(),
        timer.elapsed_in_millis()
    );

    let timer = Timer::new();
    (0..RAYS).into_iter().for_each(|_| {
        let mut ray = rtbvh::Ray::new(origin, direction);
        for (triangle, ray) in bvh.traverse_iter(&mut ray, &primitives) {
            triangle.intersect(ray);
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "Single-threaded rays: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let timer = Timer::new();
    (0..RAYS).into_par_iter().for_each(|_| {
        let mut ray = rtbvh::Ray::new(origin, direction);
        for (triangle, ray) in bvh.traverse_iter(&mut ray, &primitives) {
            triangle.intersect(ray);
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "{} threads rays: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let origin_x = Vec4::from([0.0; 4]);
    let origin_y = Vec4::from([1.5; 4]);
    let origin_z = Vec4::from([0.0; 4]);
    let direction_x = Vec4::from([0.0; 4]);
    let direction_y = Vec4::from([0.0; 4]);
    let direction_z = Vec4::from([1.0; 4]);

    let timer = Timer::new();
    (0..(RAYS / 3)).into_iter().for_each(|_| {
        let mut packet = rtbvh::RayPacket4 {
            origin_x,
            origin_y,
            origin_z,
            direction_x,
            direction_y,
            direction_z,
            inv_direction_x: Vec4::ONE / direction_x,
            inv_direction_y: Vec4::ONE / direction_y,
            inv_direction_z: Vec4::ONE / direction_z,
            pixel_ids: UVec4::ZERO,
            t: [1e34; 4].into(),
        };

        for (triangle, packet) in bvh.traverse_iter_packet(&mut packet, &primitives) {
            triangle.intersect4(packet, Vec4::splat(1e-4));
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "Single-threaded packets: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let timer = Timer::new();
    (0..(RAYS / 4)).into_par_iter().for_each(|_| {
        let mut packet = rtbvh::RayPacket4 {
            origin_x,
            origin_y,
            origin_z,
            direction_x,
            direction_y,
            direction_z,
            inv_direction_x: Vec4::ONE / direction_x,
            inv_direction_y: Vec4::ONE / direction_y,
            inv_direction_z: Vec4::ONE / direction_z,
            pixel_ids: [0; 4].into(),
            t: [1e34; 4].into(),
        };

        for (triangle, packet) in bvh.traverse_iter_packet(&mut packet, &primitives) {
            triangle.intersect4(packet, Vec4::splat(1e-4));
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "{} threads packets: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    // =========== binned sah
    println!();
    let timer = Timer::default();
    let mbvh = rtbvh::Mbvh::from(bvh);
    println!("Mbvh construction took {} ms", timer.elapsed_in_millis());

    let timer = Timer::new();
    (0..RAYS).into_iter().for_each(|_| {
        let mut ray = rtbvh::Ray::new(origin, direction);
        for (triangle, ray) in mbvh.traverse_iter(&mut ray, &primitives) {
            let _result = triangle.intersect(ray);
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "Single-threaded rays: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let timer = Timer::new();
    (0..RAYS).into_par_iter().for_each(|_| {
        let mut ray = rtbvh::Ray::new(origin, direction);
        for (triangle, ray) in mbvh.traverse_iter(&mut ray, &primitives) {
            triangle.intersect(ray);
        }
    });

    let elapsed = timer.elapsed_in_millis();
    println!(
        "{} threads rays: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let timer = Timer::new();
    (0..(RAYS / 4)).into_iter().for_each(|_| {
        let mut packet = rtbvh::RayPacket4 {
            origin_x,
            origin_y,
            origin_z,
            direction_x,
            direction_y,
            direction_z,
            inv_direction_x: Vec4::ONE / direction_x,
            inv_direction_y: Vec4::ONE / direction_y,
            inv_direction_z: Vec4::ONE / direction_z,
            pixel_ids: UVec4::ZERO,
            t: [1e34; 4].into(),
        };

        for (triangle, packet) in mbvh.traverse_iter_packet(&mut packet, &primitives) {
            triangle.intersect4(packet, Vec4::splat(1e-4));
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "Single-threaded packets: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let timer = Timer::new();
    (0..(RAYS / 4)).into_par_iter().for_each(|_| {
        let mut packet = rtbvh::RayPacket4 {
            origin_x,
            origin_y,
            origin_z,
            direction_x,
            direction_y,
            direction_z,
            inv_direction_x: Vec4::ONE / direction_x,
            inv_direction_y: Vec4::ONE / direction_y,
            inv_direction_z: Vec4::ONE / direction_z,
            pixel_ids: [0; 4].into(),
            t: [1e34; 4].into(),
        };

        for (triangle, packet) in mbvh.traverse_iter_packet(&mut packet, &primitives) {
            triangle.intersect4(packet, Vec4::splat(1e-4));
        }
    });
    let elapsed = timer.elapsed_in_millis();
    println!(
        "{} threads packets: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    // 0.5.0 bvh crate
    println!();
    let timer = Timer::default();
    let bvh = bvh::bvh::BVH::build(&mut primitives);
    println!(
        "bvh-0.5 of {} primitives construction took {} ms",
        primitives.len(),
        timer.elapsed_in_millis()
    );

    let ray = bvh::ray::Ray::new((*origin.as_ref()).into(), (*direction.as_ref()).into());

    let timer = Timer::new();
    (0..RAYS).into_iter().for_each(|_| {
        let t_min = 1e-4;
        let mut t_max = 1e34;
        let origin = vec3(ray.origin.x, ray.origin.y, ray.origin.z);
        let direction = vec3(ray.direction.x, ray.direction.y, ray.direction.z);
        for triangle in bvh.traverse_iterator(&ray, &primitives) {
            let v0 = triangle.vertex0();
            let v1 = triangle.vertex1();
            let v2 = triangle.vertex2();

            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let h_val = direction.cross(edge2);
            let a_val = edge1.dot(h_val);
            if a_val > -1e-5 && a_val < 1e-5 {
                continue;
            }
            let f_val = 1.0 / a_val;
            let s_val = origin - v0.xyz();
            let u_val = f_val * s_val.dot(h_val);
            if !(0.0..=1.0).contains(&u_val) {
                continue;
            }
            let q_val = s_val.cross(edge1);
            let v_val = f_val * direction.dot(q_val);
            if v_val < 0. || (u_val + v_val) > 1.0 {
                continue;
            }

            let t: f32 = f_val * edge2.dot(q_val);
            if t > t_min && t < t_max {
                t_max = t;
            }
        }
    });

    let elapsed = timer.elapsed_in_millis();
    println!(
        "Single-threaded rays: {} rays  in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let timer = Timer::new();
    (0..RAYS).into_par_iter().for_each(|_| {
        let t_min = 1e-4;
        let mut t_max = 1e34;
        let origin = vec3(ray.origin.x, ray.origin.y, ray.origin.z);
        let direction = vec3(ray.direction.x, ray.direction.y, ray.direction.z);
        for triangle in bvh.traverse_iterator(&ray, &primitives) {
            let v0 = triangle.vertex0();
            let v1 = triangle.vertex1();
            let v2 = triangle.vertex2();

            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let h_val = direction.cross(edge2);
            let a_val = edge1.dot(h_val);
            if a_val > -1e-5 && a_val < 1e-5 {
                continue;
            }
            let f_val = 1.0 / a_val;
            let s_val = origin - v0.xyz();
            let u_val = f_val * s_val.dot(h_val);
            if !(0.0..=1.0).contains(&u_val) {
                continue;
            }
            let q_val = s_val.cross(edge1);
            let v_val = f_val * direction.dot(q_val);
            if v_val < 0. || (u_val + v_val) > 1.0 {
                continue;
            }

            let t: f32 = f_val * edge2.dot(q_val);
            if t > t_min && t < t_max {
                t_max = t;
            }
        }
    });

    let elapsed = timer.elapsed_in_millis();
    println!(
        "{} threads rays: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );
}
