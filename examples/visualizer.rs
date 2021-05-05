use glam::*;
use l3d::prelude::*;
use pixels::{Pixels, SurfaceTexture};
use rayon::prelude::*;
use rtbvh::spatial_sah::SpatialTriangle;
use rtbvh::*;
use std::collections::HashMap;
use std::fmt::Formatter;
use std::num::NonZeroUsize;
use std::path::PathBuf;
use std::time::{Duration, Instant};
use winit::dpi::LogicalSize;
use winit::event::{ElementState, Event, VirtualKeyCode, WindowEvent};
use winit::event_loop::ControlFlow;

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

#[repr(align(16))]
#[derive(Debug, Copy, Clone)]
struct Triangle {
    vertex0: Vec4,
    vertex1: Vec4,
    vertex2: Vec4,
    normal: Vec3,
    id: usize,
}

impl Triangle {
    pub fn new(p0: Vec4, p1: Vec4, p2: Vec4) -> Self {
        Self {
            vertex0: p0,
            vertex1: p1,
            vertex2: p2,
            normal: (p1.xyz() - p0.xyz()).cross(p2.xyz() - p0.xyz()).normalize(),
            id: 0,
        }
    }
}

impl Primitive for Triangle {
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

#[derive(Default, Debug, Copy, Clone)]
pub struct CameraView3D {
    pub pos: Vec3,
    pub right: Vec3,
    pub up: Vec3,
    pub p1: Vec3,
    pub direction: Vec3,
    pub inv_width: f32,
    pub inv_height: f32,
    pub aspect_ratio: f32,
    pub fov: f32,
}

impl CameraView3D {
    pub fn translate_target(&mut self, offset: Vec3) {
        self.direction = (self.direction - offset.x * self.right - offset.y * self.up).normalize();

        let screen_size = (self.fov * 0.5 / (180.0 / std::f32::consts::PI)).tan();
        let up = Vec3::Y;
        let right = self.direction.cross(up);

        let center = self.pos + self.direction;

        self.p1 = center - screen_size * right * self.aspect_ratio + screen_size * up;
        let p2 = center + screen_size * right * self.aspect_ratio + screen_size * up;
        let p3 = center - screen_size * right * self.aspect_ratio - screen_size * up;

        self.right = p2 - self.p1;
        self.up = p3 - self.p1;
    }

    pub fn translate(&mut self, offset: Vec3) {
        let screen_size = (self.fov * 0.5 / (180.0 / std::f32::consts::PI)).tan();
        let up = Vec3::Y;
        let right = self.direction.cross(up);

        self.pos += -offset.x * right + offset.y * up + offset.z * self.direction;

        let center = self.pos + self.direction;

        self.p1 = center - screen_size * right * self.aspect_ratio + screen_size * up;
        let p2 = center + screen_size * right * self.aspect_ratio + screen_size * up;
        let p3 = center - screen_size * right * self.aspect_ratio - screen_size * up;

        self.right = p2 - self.p1;
        self.up = p3 - self.p1;
    }
}

#[allow(dead_code)]
impl CameraView3D {
    pub fn generate_ray(&self, x: u32, y: u32) -> Ray {
        let u = x as f32 * self.inv_width;
        let v = y as f32 * self.inv_height;
        let point_on_pixel = self.p1 + u * self.right + v * self.up;
        let direction = (point_on_pixel - self.pos).normalize();

        Ray::new(self.pos, direction)
    }

    pub fn generate_ray4(&self, x: [u32; 4], y: [u32; 4], ids: [u32; 4]) -> RayPacket4 {
        let x = [x[0] as f32, x[1] as f32, x[2] as f32, x[3] as f32];
        let y = [y[0] as f32, y[1] as f32, y[2] as f32, y[3] as f32];

        let x = Vec4::from(x);
        let y = Vec4::from(y);

        let u = x * self.inv_width;
        let v = y * self.inv_height;

        let p_x = Vec4::splat(self.p1[0]) + u * self.right[0] + v * self.up[0];
        let p_y = Vec4::splat(self.p1[1]) + u * self.right[1] + v * self.up[1];
        let p_z = Vec4::splat(self.p1[2]) + u * self.right[2] + v * self.up[2];

        let direction_x = p_x - Vec4::splat(self.pos[0]);
        let direction_y = p_y - Vec4::splat(self.pos[1]);
        let direction_z = p_z - Vec4::splat(self.pos[2]);

        let length_squared = direction_x * direction_x;
        let length_squared = length_squared + direction_y * direction_y;
        let length_squared = length_squared + direction_z * direction_z;

        let length = vec4(
            length_squared.x.sqrt(),
            length_squared.y.sqrt(),
            length_squared.z.sqrt(),
            length_squared.w.sqrt(),
        );

        let inv_length = Vec4::ONE / length;

        let direction_x = direction_x * inv_length;
        let direction_y = direction_y * inv_length;
        let direction_z = direction_z * inv_length;

        let origin_x = Vec4::splat(self.pos[0]);
        let origin_y = Vec4::splat(self.pos[1]);
        let origin_z = Vec4::splat(self.pos[2]);

        RayPacket4 {
            origin_x,
            origin_y,
            origin_z,
            direction_x,
            direction_y,
            direction_z,
            inv_direction_x: 1.0 / direction_x,
            inv_direction_y: 1.0 / direction_y,
            inv_direction_z: 1.0 / direction_z,
            t: [1e34_f32; 4].into(),
            pixel_ids: ids.into(),
        }
    }

    fn calculate_matrix(&self) -> (Vec3, Vec3, Vec3) {
        let y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
        let z: Vec3 = self.direction.normalize();
        let x: Vec3 = z.cross(y).normalize();
        let y: Vec3 = x.cross(z).normalize();
        (x, y, z)
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
    .construct_binned_sah()
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
