use glam::*;
use l3d::prelude::*;
use pixels::{Pixels, SurfaceTexture};
use rayon::prelude::*;
use rtbvh::*;
use std::collections::HashMap;
use std::fmt::Formatter;
use std::num::NonZeroUsize;
use std::path::PathBuf;
use winit::dpi::LogicalSize;
use winit::event::{ElementState, Event, VirtualKeyCode, WindowEvent};
use winit::event_loop::ControlFlow;
use shared::*;

#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
enum BvhToUse {
    Bvh = 0,
    Mbvh = 1,
    AltBvh = 2,
    BvhPacket = 3,
    MbvhPacket = 4,
}

impl std::fmt::Display for BvhToUse {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                BvhToUse::Bvh => "Bvh",
                BvhToUse::Mbvh => "Mbvh",
                BvhToUse::AltBvh => "bvh-0.5",
                BvhToUse::BvhPacket => "Bvh-packet",
                BvhToUse::MbvhPacket => "Mbvh-packet",
            }
        )
    }
}


const WIDTH: usize = 1280;
const HEIGHT: usize = 720;

fn main() {
    let loader = LoadInstance::new().with_default();
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
        .collect::<Vec<Triangle>>();
    let aabbs = primitives
        .iter()
        .map(|t| t.aabb())
        .collect::<Vec<rtbvh::Aabb>>();

    let bvh = rtbvh::Builder {
        aabbs: Some(aabbs.as_slice()),
        primitives: primitives.as_slice(),
        primitives_per_leaf: NonZeroUsize::new(3),
    }
    .construct_spatial_sah()
    .unwrap();

    let mbvh = Mbvh::from(bvh.clone());

    let alt_bvh = bvh::bvh::BVH::build(&mut primitives);

    let event_loop = winit::event_loop::EventLoop::new();
    let window = winit::window::WindowBuilder::new()
        .with_inner_size(LogicalSize::new(WIDTH as u32, HEIGHT as u32))
        .with_resizable(false)
        .with_title("rtbvh")
        .build(&event_loop)
        .unwrap();

    let mut pixels = {
        let window_size = window.inner_size();
        let surface_texture = SurfaceTexture::new(window_size.width, window_size.height, &window);
        Pixels::new(WIDTH as u32, HEIGHT as u32, surface_texture).unwrap()
    };

    let mut keys: HashMap<VirtualKeyCode, bool> = HashMap::new();

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
    let mut camera = CameraView3D {
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

    let mut timer = Timer::default();
    let mut rt_timer = Timer::default();
    let mut averager = Averager::default();

    let mut bvh_to_use = BvhToUse::Bvh;
    event_loop.run(move |event, _, cf| {
        *cf = ControlFlow::Poll;

        match event {
            Event::WindowEvent { event, .. } => match event {
                WindowEvent::CloseRequested => {
                    *cf = ControlFlow::Exit;
                }
                WindowEvent::Destroyed => {
                    *cf = ControlFlow::Exit;
                }
                WindowEvent::KeyboardInput { input, .. } => {
                    if let Some(k) = input.virtual_keycode {
                        if input.state == ElementState::Pressed {
                            keys.insert(k, true);
                        } else {
                            keys.insert(k, false);
                        }
                    }
                }
                _ => {}
            },
            Event::MainEventsCleared => {
                let elapsed = timer.elapsed_in_millis() / 100.0;
                timer.reset();

                let title = format!("Bvh: {}, {:.3} MRays/s", bvh_to_use, averager.get_average());
                window.set_title(title.as_str());

                if keys.get(&VirtualKeyCode::Escape).copied().unwrap_or(false) {
                    *cf = ControlFlow::Exit;
                    return;
                }

                if keys.get(&VirtualKeyCode::Key1).copied().unwrap_or(false) {
                    bvh_to_use = BvhToUse::Bvh;
                }

                if keys.get(&VirtualKeyCode::Key2).copied().unwrap_or(false) {
                    bvh_to_use = BvhToUse::Mbvh;
                }

                if keys.get(&VirtualKeyCode::Key3).copied().unwrap_or(false) {
                    bvh_to_use = BvhToUse::AltBvh;
                }

                if keys.get(&VirtualKeyCode::Key4).copied().unwrap_or(false) {
                    bvh_to_use = BvhToUse::BvhPacket;
                }

                if keys.get(&VirtualKeyCode::Key5).copied().unwrap_or(false) {
                    bvh_to_use = BvhToUse::MbvhPacket;
                }

                let mut offset = Vec3::ZERO;
                let mut view_offset = Vec3::ZERO;
                if keys.get(&VirtualKeyCode::W).copied().unwrap_or(false) {
                    offset += Vec3::Z;
                }

                if keys.get(&VirtualKeyCode::S).copied().unwrap_or(false) {
                    offset -= Vec3::Z;
                }

                if keys.get(&VirtualKeyCode::A).copied().unwrap_or(false) {
                    offset += Vec3::X;
                }

                if keys.get(&VirtualKeyCode::D).copied().unwrap_or(false) {
                    offset -= Vec3::X;
                }

                if keys.get(&VirtualKeyCode::E).copied().unwrap_or(false) {
                    offset += Vec3::Y;
                }

                if keys.get(&VirtualKeyCode::Q).copied().unwrap_or(false) {
                    offset -= Vec3::Y;
                }

                if keys.get(&VirtualKeyCode::Up).copied().unwrap_or(false) {
                    view_offset += Vec3::Y;
                }

                if keys.get(&VirtualKeyCode::Down).copied().unwrap_or(false) {
                    view_offset -= Vec3::Y;
                }

                if keys.get(&VirtualKeyCode::Left).copied().unwrap_or(false) {
                    view_offset += Vec3::X;
                }

                if keys.get(&VirtualKeyCode::Right).copied().unwrap_or(false) {
                    view_offset -= Vec3::X;
                }

                if keys.get(&VirtualKeyCode::LShift).copied().unwrap_or(false) {
                    offset *= 10.0;
                    view_offset *= 2.0;
                }

                camera.translate(offset * elapsed);
                camera.translate_target(view_offset * elapsed / 10.0);

                window.request_redraw();

                let frame = pixels.get_frame();
                rt_timer.reset();

                if bvh_to_use == BvhToUse::BvhPacket || bvh_to_use == BvhToUse::MbvhPacket {
                    frame
                        .par_chunks_exact_mut(16)
                        .enumerate()
                        .for_each(|(mut i, pixels)| {
                            i *= 4;
                            let x = i as u32 % WIDTH as u32;
                            let y = i as u32 / WIDTH as u32;
                            let mut packet =
                                camera.generate_ray4([x, x + 1, x + 2, x + 3], [y; 4], [0; 4]);

                            let t_min = Vec4::splat(1e-4);
                            let mut triangles = [None; 4];
                            match bvh_to_use {
                                BvhToUse::BvhPacket => {
                                    for (triangle, packet) in
                                        bvh.traverse_iter_packet(&mut packet, &primitives)
                                    {
                                        if let Some(result) = triangle.intersect4(packet, t_min) {
                                            for i in 0..4 {
                                                if result[i] {
                                                    triangles[i] = Some(triangle);
                                                }
                                            }
                                        }
                                    }
                                }
                                BvhToUse::MbvhPacket => {
                                    for (triangle, packet) in
                                        mbvh.traverse_iter_packet(&mut packet, &primitives)
                                    {
                                        if let Some(result) = triangle.intersect4(packet, t_min) {
                                            for i in 0..4 {
                                                if result[i] {
                                                    triangles[i] = Some(triangle);
                                                }
                                            }
                                        }
                                    }
                                }
                                _ => {}
                            }

                            for (i, triangle) in triangles.iter().enumerate() {
                                let offset = i * 4;
                                if let Some(triangle) = triangle {
                                    let normal = triangle.normal.xyz().clamp(Vec3::ZERO, Vec3::ONE)
                                        * Vec3::splat(255.0);

                                    pixels[offset] = normal.x as u8;
                                    pixels[offset + 1] = normal.y as u8;
                                    pixels[offset + 2] = normal.z as u8;
                                    pixels[offset + 3] = 255;
                                } else {
                                    for i in 0..4 {
                                        pixels[offset + i] = 0;
                                    }
                                }
                            }
                        });
                } else {
                    frame
                        .par_chunks_exact_mut(4)
                        .enumerate()
                        .for_each(|(i, pixel)| {
                            let x = i as u32 % WIDTH as u32;
                            let y = i as u32 / WIDTH as u32;

                            let mut ray = camera.generate_ray(x, y);
                            let mut t = None;
                            let r = bvh::ray::Ray::new(
                                (*ray.origin.as_ref()).into(),
                                (*ray.direction.as_ref()).into(),
                            );

                            match bvh_to_use {
                                BvhToUse::Bvh => {
                                    for (triangle, r) in bvh.traverse_iter(&mut ray, &primitives) {
                                        if triangle.intersect(r) {
                                            t = Some(triangle);
                                        }
                                    }
                                }
                                BvhToUse::Mbvh => {
                                    for (triangle, r) in mbvh.traverse_iter(&mut ray, &primitives) {
                                        if triangle.intersect(r) {
                                            t = Some(triangle);
                                        }
                                    }
                                }
                                BvhToUse::AltBvh => {
                                    for triangle in alt_bvh.traverse_iterator(&r, &primitives) {
                                        if triangle.intersect(&mut ray) {
                                            t = Some(triangle);
                                        }
                                    }
                                }
                                _ => {}
                            }

                            if let Some(triangle) = t {
                                let normal = triangle.normal.xyz().clamp(Vec3::ZERO, Vec3::ONE)
                                    * Vec3::splat(255.0);

                                pixel[0] = normal.x as u8;
                                pixel[1] = normal.y as u8;
                                pixel[2] = normal.z as u8;
                                pixel[3] = 255;
                            } else {
                                pixel.iter_mut().for_each(|p| *p = 0);
                            }
                        });
                }

                averager.add_sample(
                    WIDTH as f32 * HEIGHT as f32
                        / 1_000_000.0
                        / (rt_timer.elapsed_in_millis() / 1000.0),
                );

                if pixels.render().is_err() {
                    *cf = ControlFlow::Exit;
                }
            }
            Event::RedrawRequested(_) => {}
            _ => {}
        }
    })
}

#[derive(Debug, Clone)]
pub struct Averager<T: num::Float + num::FromPrimitive> {
    values: Vec<T>,
    capacity: usize,
    index: usize,
    has_looped: bool,
}

impl<T: num::Float + num::FromPrimitive> Default for Averager<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: num::Float + num::FromPrimitive> Averager<T> {
    pub fn new() -> Averager<T> {
        Self {
            values: vec![T::from_f32(0.0).unwrap(); 100],
            capacity: 100,
            index: 0,
            has_looped: false,
        }
    }

    pub fn with_capacity(capacity: usize) -> Averager<T> {
        Self {
            values: vec![T::from_f32(0.0).unwrap(); capacity],
            capacity,
            index: 0,
            has_looped: false,
        }
    }

    pub fn add_sample(&mut self, sample: T) {
        if self.has_looped {
            for i in 0..(self.capacity - 1) {
                self.values[i] = self.values[i + 1];
            }
            self.values[self.capacity - 1] = sample;
            return;
        }

        if self.index >= (self.capacity - 1) {
            self.has_looped = true;
        }

        self.values[self.index] = sample;
        self.index += 1;
    }

    pub fn get_average(&mut self) -> T {
        let range = if self.has_looped {
            self.capacity
        } else {
            self.index
        };
        let mut avg = T::from(0.0).unwrap();
        for i in 0..range {
            avg = avg + self.values[i];
        }
        avg * (T::from_f32(1.0).unwrap() / T::from_usize(range).unwrap())
    }

    pub fn data(&self) -> &[T] {
        &self.values[0..self.index.min(self.capacity)]
    }
}
