use glam::*;
use rtbvh::*;
use std::time::{Duration, Instant};

#[repr(align(16))]
#[derive(Debug, Copy, Clone)]
pub struct Triangle {
    pub vertex0: Vec4,
    pub vertex1: Vec4,
    pub vertex2: Vec4,
    pub normal: Vec3,
    pub id: usize,
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

    pub fn generate_ray4(&self, x: [u32; 4], y: [u32; 4]) -> RayPacket4 {
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
