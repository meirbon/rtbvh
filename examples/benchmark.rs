use glam::*;
use l3d::prelude::*;
use rayon::prelude::*;
use rtbvh::*;
use std::path::PathBuf;

mod shared;
use shared::*;

const PRIMS_PER_LEAF: usize = 1;
const WIDTH: usize = 1000;
const HEIGHT: usize = 1000;
const FRAMES: usize = 100;
const RAYS: usize = FRAMES * WIDTH * HEIGHT;

fn intersect_rays<'a, 'b, B: IntoRayIterator<'a, 'b, Triangle> + Sync>(
    bvh: &'a B,
    triangles: &'a [Triangle],
    parallel: bool,
    mut rays: Vec<Ray>,
) -> f32 {
    let rays = unsafe { std::mem::transmute::<&mut [Ray], &'b mut [Ray]>(rays.as_mut_slice()) };

    let timer = Timer::default();
    let elapsed = if parallel {
        rays.par_chunks_mut(WIDTH).for_each(|chunk| {
            for ray in chunk {
                for (triangle, ray) in bvh.iter(ray, triangles) {
                    let _result = triangle.intersect(ray);
                }
            }
        });
        timer.elapsed_in_millis()
    } else {
        for ray in rays.iter_mut() {
            for (triangle, ray) in bvh.iter(ray, triangles) {
                let _result = triangle.intersect(ray);
            }
        }
        timer.elapsed_in_millis()
    };

    elapsed
}

fn intersect_packets<'a, 'b, B: IntoPacketIterator<'a, 'b, Triangle> + Sync>(
    bvh: &'a B,
    triangles: &'a [Triangle],
    parallel: bool,
    mut rays: Vec<RayPacket4>,
) -> f32 {
    let rays = unsafe {
        std::mem::transmute::<&mut [RayPacket4], &'b mut [RayPacket4]>(rays.as_mut_slice())
    };

    let timer = Timer::default();
    let elapsed = if parallel {
        rays.par_chunks_mut(WIDTH).for_each(|chunk| {
            for ray in chunk {
                for (triangle, ray) in bvh.iter(ray, triangles) {
                    let _result = triangle.intersect4(ray, Vec4::splat(1e-4));
                }
            }
        });
        timer.elapsed_in_millis()
    } else {
        for ray in rays.iter_mut() {
            for (triangle, ray) in bvh.iter(ray, triangles) {
                let _result = triangle.intersect4(ray, Vec4::splat(1e-4));
            }
        }
        timer.elapsed_in_millis()
    };

    elapsed
}

fn main() {
    let fov = 90_f32.to_radians();
    let screen_size = (fov * 0.5 / (180.0 / std::f32::consts::PI)).tan();
    let up = Vec3::Y;
    let right = Vec3::Z.cross(up);
    let pos = vec3(0.0, 1.5, -100.0);
    let center = pos + Vec3::Z;

    let aspect_ratio = WIDTH as f32 / HEIGHT as f32;
    let p1 = center - screen_size * right * aspect_ratio + screen_size * up;
    let p2 = center + screen_size * right * aspect_ratio + screen_size * up;
    let p3 = center - screen_size * right * aspect_ratio - screen_size * up;

    let right = p2 - p1;
    let up = p3 - p1;
    let camera = CameraView3D {
        pos,
        right,
        up,
        p1,
        direction: Vec3::Z,
        inv_width: 1.0 / WIDTH as f32,
        inv_height: 1.0 / HEIGHT as f32,
        aspect_ratio,
        fov,
    };

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
        .map(|c| Triangle::new(Vec4::from(c[0]), Vec4::from(c[1]), Vec4::from(c[2])))
        .collect::<Vec<_>>();
    let aabbs = primitives
        .iter()
        .map(|t| t.aabb())
        .collect::<Vec<rtbvh::Aabb>>();

    let mut rays = Vec::with_capacity(WIDTH * HEIGHT * FRAMES);
    let mut packets = Vec::with_capacity(FRAMES * WIDTH * HEIGHT / 4);
    for _ in 0..FRAMES {
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                rays.push(camera.generate_ray(x as u32, y as u32));
            }
        }

        for y in 0..HEIGHT {
            for x in (0..WIDTH).step_by(4) {
                let x = x as u32;
                let y = y as u32;
                packets.push(camera.generate_ray4([x, x + 1, x + 2, x + 3], [y; 4], [0; 4]));
            }
        }
    }

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

    let elapsed = intersect_rays(&bvh, &primitives, false, rays.clone());
    println!(
        "Single-threaded rays: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let elapsed = intersect_rays(&bvh, &primitives, true, rays.clone());
    println!(
        "{} threads rays: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let elapsed = intersect_packets(&bvh, &primitives, false, packets.clone());

    println!(
        "Single-threaded packets: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let elapsed = intersect_packets(&bvh, &primitives, true, packets.clone());

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

    let elapsed = intersect_rays(&mbvh, &primitives, false, rays.clone());
    println!(
        "Single-threaded rays: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let elapsed = intersect_rays(&mbvh, &primitives, true, rays.clone());
    println!(
        "{} threads rays: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let elapsed = intersect_packets(&mbvh, &primitives, false, packets.clone());
    println!(
        "Single-threaded packets: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let elapsed = intersect_packets(&mbvh, &primitives, true, packets.clone());
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
        "Bvh construction with binned sah type of {} primitives took {} ms",
        primitives.len(),
        timer.elapsed_in_millis()
    );

    let elapsed = intersect_rays(&bvh, &primitives, false, rays.clone());
    println!(
        "Single-threaded rays: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let elapsed = intersect_rays(&bvh, &primitives, true, rays.clone());
    println!(
        "{} threads rays: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let elapsed = intersect_packets(&bvh, &primitives, false, packets.clone());
    println!(
        "Single-threaded packets: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let elapsed = intersect_packets(&bvh, &primitives, true, packets.clone());
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

    let elapsed = intersect_rays(&mbvh, &primitives, false, rays.clone());
    println!(
        "Single-threaded rays: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let elapsed = intersect_rays(&mbvh, &primitives, true, rays.clone());
    println!(
        "{} threads rays: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let elapsed = intersect_packets(&mbvh, &primitives, false, packets.clone());
    println!(
        "Single-threaded packets: {} rays in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let elapsed = intersect_packets(&mbvh, &primitives, true, packets);
    println!(
        "{} threads packets: {} rays in {} ms, {} million rays per second",
        rayon::current_num_threads(),
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );
    println!();

    // 0.5.0 bvh crate
    println!();
    let timer = Timer::default();
    let bvh = bvh::bvh::BVH::build(&mut primitives);
    println!(
        "bvh-0.5 of {} primitives construction took {} ms",
        primitives.len(),
        timer.elapsed_in_millis()
    );

    let timer = Timer::new();
    rays.chunks_mut(WIDTH).for_each(|chunk| {
        for ray in chunk {
            let r = bvh::ray::Ray::new(
                (*ray.origin.as_ref()).into(),
                (*ray.direction.as_ref()).into(),
            );
    
            for triangle in bvh.traverse_iterator(&r, &primitives) {
                let _result = triangle.intersect(ray);
            }
        }
    });
    let elapsed = timer.elapsed_in_millis();
    rays.iter_mut().for_each(|r| r.reset());
    println!(
        "Single-threaded rays: {} rays  in {} ms, {} million rays per second",
        RAYS,
        elapsed,
        RAYS as f32 / 1000.0 / elapsed
    );

    let timer = Timer::new();
    rays.par_chunks_mut(WIDTH).for_each(|chunk| {
        for ray in chunk {
            let r = bvh::ray::Ray::new(
                (*ray.origin.as_ref()).into(),
                (*ray.direction.as_ref()).into(),
            );

            for triangle in bvh.traverse_iterator(&r, &primitives) {
                let _result = triangle.intersect(ray);
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
