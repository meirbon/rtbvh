use glam::*;

#[derive(Copy, Clone)]
pub struct Ray {
    pub origin: [f32; 3],
    pub direction: [f32; 3],
}

impl Ray {
    pub fn get_vectors<T: From<[f32; 3]>>(&self) -> (T, T) {
        (self.origin.into(), self.direction.into())
    }
}

impl<T: Into<[f32; 3]>> From<(T, T)> for Ray {
    fn from(vectors: (T, T)) -> Self {
        Ray {
            origin: vectors.0.into(),
            direction: vectors.1.into(),
        }
    }
}

impl<T: From<[f32; 3]>> From<Ray> for (T, T) {
    fn from(r: Ray) -> Self {
        (r.origin.into(), r.direction.into())
    }
}

#[derive(Clone)]
pub struct RayPacket4 {
    pub origin_x: [f32; 4],
    pub origin_y: [f32; 4],
    pub origin_z: [f32; 4],

    pub direction_x: [f32; 4],
    pub direction_y: [f32; 4],
    pub direction_z: [f32; 4],

    pub t: [f32; 4],
    pub pixel_ids: [u32; 4],
}

impl Default for RayPacket4 {
    fn default() -> Self {
        Self {
            origin_x: [0.0; 4],
            origin_y: [0.0; 4],
            origin_z: [0.0; 4],
            direction_x: [0.0; 4],
            direction_y: [0.0; 4],
            direction_z: [0.0; 4],
            t: [0.0; 4],
            pixel_ids: [0; 4],
        }
    }
}

impl RayPacket4 {
    pub fn new() -> RayPacket4 {
        Self::default()
    }

    pub fn origin_xyz<T: From<[f32; 4]>>(&self) -> (T, T, T) {
        (
            T::from(self.origin_x),
            T::from(self.origin_y),
            T::from(self.origin_z),
        )
    }

    pub fn direction_xyz<T: From<[f32; 4]>>(&self) -> (T, T, T) {
        (
            T::from(self.direction_x),
            T::from(self.direction_y),
            T::from(self.direction_z),
        )
    }

    pub fn ray(&self, index: usize) -> Ray {
        debug_assert!(index <= 4);

        let origin = [
            self.origin_x[index],
            self.origin_y[index],
            self.origin_z[index],
        ];

        let direction = [
            self.direction_x[index],
            self.direction_y[index],
            self.direction_z[index],
        ];

        Ray { origin, direction }
    }

    pub fn t<T: From<[f32; 4]>>(&self) -> T {
        T::from(self.t)
    }
}

#[derive(Copy, Clone)]
pub struct ShadowPacket4 {
    pub origin_x: [f32; 4],
    pub origin_y: [f32; 4],
    pub origin_z: [f32; 4],
    pub direction_x: [f32; 4],
    pub direction_y: [f32; 4],
    pub direction_z: [f32; 4],
    pub t_max: [f32; 4],
}

#[allow(dead_code)]
impl Ray {
    pub fn new(origin: [f32; 3], direction: [f32; 3]) -> Ray {
        Ray { origin, direction }
    }

    pub fn reflect(&self, p: &[f32; 3], n: &[f32; 3], epsilon: f32) -> Ray {
        let p = Vec3A::from(*p);
        let n = Vec3A::from(*n);

        let direction = Vec3A::from(self.direction);

        let tmp: Vec3A = n * n.dot(direction) * 2.0;
        let direction = direction - tmp;

        Ray {
            origin: (p + direction * epsilon).into(),
            direction: direction.into(),
        }
    }

    pub fn get_point_at(&self, t: f32) -> [f32; 3] {
        let mut point = self.origin;
        for (p, d) in point.iter_mut().zip(self.direction.iter().copied()) {
            *p += d * t;
        }
        point
    }
}
