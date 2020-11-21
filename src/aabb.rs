use glam::*;
use serde::{Deserialize, Serialize};
use std::default::Default;
use std::fmt::{Display, Formatter};

use crate::RayPacket4;

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[repr(C)]
pub struct AABB {
    pub min: [f32; 3],
    pub max: [f32; 3],
}

pub trait Bounds {
    fn bounds(&self) -> AABB;
}

impl Default for AABB {
    fn default() -> Self {
        AABB {
            min: [0.0; 3],
            max: [0.0; 3],
        }
    }
}

impl Display for AABB {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let min = Vec3A::from(self.min);
        let max = Vec3A::from(self.max);

        write!(
            f,
            "(min: ({}, {}, {}), max: ({}, {}, {}))",
            min.x,
            min.y,
            min.z,
            max.x,
            max.y,
            max.z,
        )
    }
}

#[allow(dead_code)]
impl AABB {
    pub fn new() -> AABB {
        AABB {
            min: [1e34; 3],
            max: [-1e34; 3],
        }
    }

    pub fn full() -> AABB {
        AABB {
            min: [-1e34; 3],
            max: [1e34; 3],
        }
    }

    pub fn empty() -> AABB {
        AABB {
            min: [1e34; 3],
            max: [-1e34; 3],
        }
    }

    pub fn intersect(&self, origin: Vec3A, dir_inverse: Vec3A, t: f32) -> Option<(f32, f32)> {
        let (min, max) = self.points::<Vec3A>();

        let t1 = (min - origin) * dir_inverse;
        let t2 = (max - origin) * dir_inverse;

        let t_min: Vec3A = t1.min(t2);
        let t_max: Vec3A = t1.max(t2);

        let t_min = t_min.max_element();
        let t_max = t_max.min_element();

        if t_max > 0.0 && t_max > t_min && t_min < t {
            return Some((t_min, t_max));
        }

        None
    }

    pub fn intersect4(
        &self,
        packet: &RayPacket4,
        inv_dir_x: Vec4,
        inv_dir_y: Vec4,
        inv_dir_z: Vec4,
    ) -> Option<[f32; 4]> {
        let (org_x, org_y, org_z) = packet.origin_xyz::<Vec4>();

        let t1_x = (Vec4::splat(self.min[0]) - org_x) * inv_dir_x;
        let t1_y = (Vec4::splat(self.min[1]) - org_y) * inv_dir_y;
        let t1_z = (Vec4::splat(self.min[2]) - org_z) * inv_dir_z;

        let t2_x = (Vec4::splat(self.max[0]) - org_x) * inv_dir_x;
        let t2_y = (Vec4::splat(self.max[1]) - org_y) * inv_dir_y;
        let t2_z = (Vec4::splat(self.max[2]) - org_z) * inv_dir_z;

        let t_min_x = t1_x.min(t2_x);
        let t_min_y = t1_y.min(t2_y);
        let t_min_z = t1_z.min(t2_z);

        let t_max_x = t1_x.max(t2_x);
        let t_max_y = t1_y.max(t2_y);
        let t_max_z = t1_z.max(t2_z);

        let t_min = t_min_x.max(t_min_y.max(t_min_z));
        let t_max = t_max_x.min(t_max_y.min(t_max_z));

        let mask =
            t_max.cmpgt(Vec4::zero()) & t_max.cmpgt(t_min) & t_min.cmplt(Vec4::from(packet.t));
        if mask.any() {
            Some(t_min.into())
        } else {
            None
        }
    }

    pub fn contains<T: Into<[f32; 3]>>(&self, pos: T) -> bool {
        let pos: Vec3A = Vec3A::from(pos.into());
        let (min, max) = self.points::<Vec3A>();
        (pos.cmpgt(min) & pos.cmplt(max)).all()
    }

    pub fn grow<T: Into<[f32; 3]>>(&mut self, pos: T) {
        let pos: Vec3A = Vec3A::from(pos.into());
        let (min, max) = self.points::<Vec3A>();
        let min = min.min(pos);
        let max = max.max(pos);

        for i in 0..3 {
            self.min[i] = min[i];
            self.max[i] = max[i];
        }
    }

    pub fn grow_bb(&mut self, aabb: &AABB) {
        let (min, max) = self.points::<Vec3A>();
        let (a_min, a_max) = aabb.points::<Vec3A>();

        let min = min.min(a_min);
        let max = max.max(a_max);

        self.min = min.into();
        self.max = max.into();
    }

    pub fn shrink(&mut self, aabb: &AABB) {
        let (min, max) = self.points::<Vec3A>();
        let (a_min, a_max) = aabb.points::<Vec3A>();

        let min = min.max(a_min);
        let max = max.min(a_max);

        self.min = min.into();
        self.max = max.into();
    }

    pub fn offset_by(&mut self, delta: f32) {
        let delta = Vec3A::splat(delta);
        let (min, max) = self.points::<Vec3A>();

        let min = min - delta;
        let max = max + delta;

        self.min = min.into();
        self.max = max.into();
    }

    pub fn union_of(&self, bb: &AABB) -> AABB {
        let (min, max) = self.points::<Vec3A>();
        let (b_min, b_max) = bb.points::<Vec3A>();

        let new_min = Vec3A::from(min).min(Vec3A::from(b_min));
        let new_max = Vec3A::from(max).max(Vec3A::from(b_max));

        AABB {
            min: new_min.into(),
            max: new_max.into(),
        }
    }

    pub fn intersection(&self, bb: &AABB) -> AABB {
        let (min, max) = self.points::<Vec3A>();
        let (b_min, b_max) = bb.points::<Vec3A>();

        let new_min = Vec3A::from(min).max(Vec3A::from(b_min));
        let new_max = Vec3A::from(max).min(Vec3A::from(b_max));

        AABB {
            min: new_min.into(),
            max: new_max.into(),
        }
    }

    pub fn volume(&self) -> f32 {
        let length = Vec3A::from(self.max) - Vec3A::from(self.min);
        return length.x * length.y * length.z;
    }

    pub fn center<T: From<[f32; 3]>>(&self) -> T {
        T::from(((Vec3A::from(self.min) + Vec3A::from(self.max)) * 0.5).into())
    }

    pub fn area(&self) -> f32 {
        let e = Vec3A::from(self.max) - Vec3A::from(self.min);
        let value: f32 = e.x * e.y + e.x * e.z + e.y * e.z;

        0.0_f32.max(value)
    }

    pub fn half_area(&self) -> f32 {
        let d = self.diagonal::<Vec3A>();
        (d[0] + d[1]) * d[2] + d[0] * d[1]
    }

    pub fn lengths<T: From<[f32; 3]>>(&self) -> T {
        let (min, max) = self.points::<Vec3A>();
        T::from((max - min).into())
    }

    pub fn longest_axis(&self) -> usize {
        let mut a: usize = 0;
        if self.extend(1) > self.extend(0) {
            a = 1;
        }
        if self.extend(2) > self.extend(a) {
            a = 2
        }
        a
    }

    pub fn all_corners(&self) -> [Vec3A; 8] {
        let lengths: Vec3A = self.lengths();

        let x_l = Vec3A::new(lengths.x, 0.0, 0.0);
        let y_l = Vec3A::new(0.0, lengths.y, 0.0);
        let z_l = Vec3A::new(0.0, 0.0, lengths.z);

        let (min, max) = self.points();

        [
            min,
            max,
            min + x_l,
            min + y_l,
            min + z_l,
            min + x_l + y_l,
            min + x_l + z_l,
            min + y_l + z_l,
        ]
    }

    pub fn extend(&self, axis: usize) -> f32 {
        self.max[axis] - self.min[axis]
    }

    pub fn from_points(points: &[Vec3A]) -> AABB {
        let mut aabb = AABB::empty();
        for point in points {
            aabb.grow(*point);
        }
        aabb.offset_by(1e-5);
        aabb
    }

    pub fn transformed<T: Into<[f32; 16]>>(&self, transform: T) -> AABB {
        let transform: Mat4 = Mat4::from_cols_array(&transform.into());
        let mut corners: [Vec3A; 8] = self.all_corners();
        for i in 0..8 {
            let corner: Vec4 = transform * corners[i].extend(1.0);
            corners[i] = Vec3A::new(corner.x, corner.y, corner.z);
        }
        AABB::from_points(&corners)
    }

    pub fn points<T: From<[f32; 3]>>(&self) -> (T, T) {
        (self.min.into(), self.max.into())
    }

    pub fn transform<T: Into<[f32; 16]>>(&mut self, transform: T) {
        let transformed = self.transformed(transform);
        *self = AABB {
            min: transformed.min,
            max: transformed.max,
            ..(*self).clone()
        }
    }

    pub fn diagonal<T: From<[f32; 3]>>(&self) -> T {
        let (min, max) = self.points::<Vec3A>();
        T::from((max - min).into())
    }

    pub fn longest_extent(&self) -> f32 {
        self.diagonal::<Vec3A>()[self.longest_axis()]
    }

    pub fn union_of_list(aabbs: &[AABB]) -> AABB {
        let mut aabb = AABB::new();
        for bb in aabbs {
            aabb.grow_bb(bb);
        }
        aabb
    }
}

impl<T: Into<[f32; 3]>> Into<AABB> for (T, T) {
    fn into(self) -> AABB {
        AABB {
            min: self.0.into(),
            max: self.1.into(),
        }
    }
}

impl<T: From<[f32; 3]>> Into<(T, T)> for AABB {
    fn into(self) -> (T, T) {
        (T::from(self.min), T::from(self.max))
    }
}

#[macro_export]
/// Creates an AABB from a list of vertices
macro_rules! aabb {
    ($($x:expr),*) => {{
        let mut bb = AABB::empty();
        $(
            bb.grow($x);
        )*

        bb.offset_by(1e-4);
        bb
    }};
}
