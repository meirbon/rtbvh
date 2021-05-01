use glam::*;

#[derive(Copy, Clone, PartialEq)]
pub struct Ray {
    pub origin: Vec3,
    pub t_min: f32,
    pub direction: Vec3,
    pub t: f32,
    pub(crate) inv_direction: Vec3,
    pub(crate) signs: [u8; 4],
}

impl From<(Vec3, Vec3)> for Ray {
    fn from(vectors: (Vec3, Vec3)) -> Self {
        let signs = [
            (vectors.1.x < 0.0) as u8,
            (vectors.1.y < 0.0) as u8,
            (vectors.1.z < 0.0) as u8,
            0,
        ];

        Ray {
            origin: vectors.0,
            direction: vectors.1,
            inv_direction: Vec3::ONE / vectors.1,
            t_min: 1e-4,
            t: 1e34,
            signs,
        }
    }
}

impl From<Ray> for (Vec3, Vec3) {
    fn from(r: Ray) -> Self {
        (r.origin, r.direction)
    }
}

#[repr(align(16))]
#[derive(Clone)]
pub struct RayPacket4 {
    pub origin_x: Vec4,
    pub origin_y: Vec4,
    pub origin_z: Vec4,

    pub direction_x: Vec4,
    pub direction_y: Vec4,
    pub direction_z: Vec4,

    pub inv_direction_x: Vec4,
    pub inv_direction_y: Vec4,
    pub inv_direction_z: Vec4,

    pub t: Vec4,
    pub pixel_ids: UVec4,
}

impl RayPacket4 {
    pub fn new(origins: [Vec4; 4], directions: [Vec4; 4], pixel_ids: [u32; 4]) -> RayPacket4 {
        Self {
            origin_x: vec4(origins[0].x, origins[1].x, origins[2].x, origins[3].x),
            origin_y: vec4(origins[0].y, origins[1].y, origins[2].y, origins[3].y),
            origin_z: vec4(origins[0].z, origins[1].z, origins[2].z, origins[3].z),
            direction_x: vec4(
                directions[0].x,
                directions[1].x,
                directions[2].x,
                directions[3].x,
            ),
            direction_y: vec4(
                directions[0].y,
                directions[1].y,
                directions[2].y,
                directions[3].y,
            ),
            direction_z: vec4(
                directions[0].z,
                directions[1].z,
                directions[2].z,
                directions[3].z,
            ),
            inv_direction_x: vec4(
                1.0 / directions[0].x,
                1.0 / directions[1].x,
                1.0 / directions[2].x,
                1.0 / directions[3].x,
            ),
            inv_direction_y: vec4(
                1.0 / directions[0].y,
                1.0 / directions[1].y,
                1.0 / directions[2].y,
                1.0 / directions[3].y,
            ),
            inv_direction_z: vec4(
                1.0 / directions[0].z,
                1.0 / directions[1].z,
                1.0 / directions[2].z,
                1.0 / directions[3].z,
            ),
            t: Vec4::splat(1e34),
            pixel_ids: pixel_ids.into(),
        }
    }

    pub fn origin_xyz(&self) -> (Vec4, Vec4, Vec4) {
        (self.origin_x, self.origin_y, self.origin_z)
    }

    pub fn direction_xyz(&self) -> (Vec4, Vec4, Vec4) {
        (self.direction_x, self.direction_y, self.direction_z)
    }

    #[inline]
    pub fn ray(&self, index: usize) -> Ray {
        debug_assert!(index <= 4);

        let origin = vec3(
            self.origin_x[index],
            self.origin_y[index],
            self.origin_z[index],
        );

        let direction = vec3(
            self.direction_x[index],
            self.direction_y[index],
            self.direction_z[index],
        );

        let signs = [
            (direction.x < 0.0) as u8,
            (direction.y < 0.0) as u8,
            (direction.z < 0.0) as u8,
            0,
        ];

        Ray {
            origin,
            t_min: 1e-4,
            direction,
            t: self.t[index],
            inv_direction: Vec3::ONE / direction,
            signs,
        }
    }

    #[inline]
    pub fn t(&self) -> Vec4 {
        self.t
    }
}

#[derive(Copy, Clone)]
pub struct ShadowPacket4 {
    pub origin_x: Vec4,
    pub origin_y: Vec4,
    pub origin_z: Vec4,
    pub direction_x: Vec4,
    pub direction_y: Vec4,
    pub direction_z: Vec4,
    pub t_max: Vec4,
}

#[allow(dead_code)]
impl Ray {
    pub fn new(origin: Vec3, direction: Vec3) -> Ray {
        let signs = [
            (direction.x < 0.0) as u8,
            (direction.y < 0.0) as u8,
            (direction.z < 0.0) as u8,
            0,
        ];

        Ray {
            origin,
            direction,
            t_min: 1e-4,
            t: 1e34,
            inv_direction: Vec3::ONE / direction,
            signs,
        }
    }

    #[inline]
    pub fn reflect(&self, hit_point: Vec3, n: Vec3, epsilon: f32) -> Ray {
        let tmp: Vec3 = n * n.dot(self.direction) * 2.0;
        let direction = self.direction - tmp;

        let signs = [
            (direction.x < 0.0) as u8,
            (direction.y < 0.0) as u8,
            (direction.z < 0.0) as u8,
            0,
        ];

        Ray {
            origin: hit_point + direction * epsilon,
            direction,
            inv_direction: Vec3::ONE / direction,
            t_min: 1e-4,
            t: 1e34,
            signs,
        }
    }

    #[inline]
    pub fn get_point_at(&self, t: f32) -> Vec3 {
        self.origin + t * self.direction
    }

    #[inline(always)]
    pub(crate) fn sign_x(&self) -> usize {
        self.signs[0] as usize
    }

    #[inline(always)]
    pub(crate) fn sign_y(&self) -> usize {
        self.signs[1] as usize
    }

    #[inline(always)]
    pub(crate) fn sign_z(&self) -> usize {
        self.signs[2] as usize
    }
}
