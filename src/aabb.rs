use crate::{Ray, RayPacket4};
use glam::*;
use std::{default::Default, fmt::Debug};
use std::{
    fmt::{Display, Formatter},
    ops::Index,
};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(align(32))]
#[derive(Debug, Copy, Clone)]
pub struct Aabb<Extra: Debug + Copy = i32> {
    pub min: Vec3,
    pub extra1: Extra,
    pub max: Vec3,
    pub extra2: Extra,
}

pub trait Bounds<E: Debug + Copy = i32> {
    fn bounds(&self) -> Aabb<E>;
}

impl<E: Debug + Copy> Default for Aabb<E>
where
    E: Default,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<E: Debug + Copy> Display for Aabb<E>
where
    E: Display,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "(min: {}, extra1: {}, max: {}, extra2: {}",
            self.min, self.extra1, self.max, self.extra2
        )
    }
}

impl<E: Debug + Copy + Default> Aabb<E> {
    #[inline]
    pub fn new() -> Self {
        Aabb {
            min: Vec3::splat(1e34),
            extra1: Default::default(),
            max: Vec3::splat(-1e34),
            extra2: Default::default(),
        }
    }

    #[inline]
    pub fn full() -> Self {
        Aabb {
            min: Vec3::splat(-1e34),
            max: Vec3::splat(1e34),
            ..Default::default()
        }
    }

    #[inline]
    pub fn empty() -> Self {
        Self::new()
    }

    #[inline]
    pub fn union_of(&self, bb: &Self) -> Self {
        let (min, max) = self.points();
        let (b_min, b_max) = bb.points();

        let new_min = min.min(b_min);
        let new_max = max.max(b_max);

        Self {
            min: new_min,
            max: new_max,
            ..Default::default()
        }
    }

    #[inline]
    pub fn intersection(&self, bb: &Self) -> Self {
        let (min, max) = self.points();
        let (b_min, b_max) = bb.points();

        let new_min = min.max(b_min);
        let new_max = max.min(b_max);

        Self {
            min: new_min,
            max: new_max,
            ..Default::default()
        }
    }

    #[inline]
    pub fn from_points(points: &[Vec3]) -> Self {
        let mut aabb = Self::empty();
        for point in points {
            aabb.grow(*point);
        }
        aabb.offset_by(1e-5);
        aabb
    }

    #[inline]
    pub fn transformed(&self, transform: Mat4) -> Self {
        let mut corners = self.all_corners();
        for c in corners.iter_mut() {
            let corner: Vec4 = transform * c.extend(1.0);
            *c = Vec3::new(corner.x, corner.y, corner.z);
        }

        Self::from_points(&corners)
    }

    #[inline]
    pub fn union_of_list(aabbs: &[Self]) -> Self {
        let mut aabb = Self::new();
        for bb in aabbs {
            aabb.grow_bb(bb);
        }
        aabb.with_offset(0.0001)
    }

    #[inline]
    pub fn transform(&mut self, transform: Mat4) {
        *self = self.transformed(transform);
    }
}

#[allow(dead_code)]
impl<E: Debug + Copy> Aabb<E> {
    pub fn is_valid(&self) -> bool {
        Vec3A::from(self.min).cmple(self.max.into()).all()
    }

    #[inline(always)]
    pub fn intersect(&self, ray: &Ray) -> Option<f32> {
        let mut ray_min = (self[ray.sign_x()].x - ray.origin.x) * ray.inv_direction.x;
        let mut ray_max = (self[1 - ray.sign_x()].x - ray.origin.x) * ray.inv_direction.x;

        let y_min = (self[ray.sign_y()].y - ray.origin.y) * ray.inv_direction.y;
        let y_max = (self[1 - ray.sign_y()].y - ray.origin.y) * ray.inv_direction.y;

        if (ray_min > y_max) || (y_min > ray_max) {
            return None;
        }

        if y_min > ray_min {
            ray_min = y_min;
        }

        if y_max < ray_max {
            ray_max = y_max;
        }

        let z_min = (self[ray.sign_z()].z - ray.origin.z) * ray.inv_direction.z;
        let z_max = (self[1 - ray.sign_z()].z - ray.origin.z) * ray.inv_direction.z;

        if (ray_min > z_max) || (z_min > ray_max) {
            return None;
        }

        if z_max < ray_max {
            ray_max = z_max;
        }

        if ray_max > ray.t_min {
            Some(ray_max)
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn intersects(&self, ray: &Ray) -> bool {
        let mut ray_min = (self[ray.sign_x()].x - ray.origin.x) * ray.inv_direction.x;
        let mut ray_max = (self[1 - ray.sign_x()].x - ray.origin.x) * ray.inv_direction.x;

        let y_min = (self[ray.sign_y()].y - ray.origin.y) * ray.inv_direction.y;
        let y_max = (self[1 - ray.sign_y()].y - ray.origin.y) * ray.inv_direction.y;

        if (ray_min > y_max) || (y_min > ray_max) {
            return false;
        }

        if y_min > ray_min {
            ray_min = y_min;
        }

        if y_max < ray_max {
            ray_max = y_max;
        }

        let z_min = (self[ray.sign_z()].z - ray.origin.z) * ray.inv_direction.z;
        let z_max = (self[1 - ray.sign_z()].z - ray.origin.z) * ray.inv_direction.z;

        if (ray_min > z_max) || (z_min > ray_max) {
            return false;
        }

        if z_max < ray_max {
            ray_max = z_max;
        }

        ray_max > 0.0
    }

    #[inline]
    pub fn intersect4(&self, packet: &RayPacket4) -> Option<[f32; 4]> {
        let t1_x = (self.min.xxxx() - packet.origin_x) * packet.inv_direction_x;
        let t1_y = (self.min.yyyy() - packet.origin_y) * packet.inv_direction_y;
        let t1_z = (self.min.zzzz() - packet.origin_z) * packet.inv_direction_z;

        let t2_x = (self.max.xxxx() - packet.origin_x) * packet.inv_direction_x;
        let t2_y = (self.max.yyyy() - packet.origin_y) * packet.inv_direction_y;
        let t2_z = (self.max.zzzz() - packet.origin_z) * packet.inv_direction_z;

        let t_min_x = t1_x.min(t2_x);
        let t_min_y = t1_y.min(t2_y);
        let t_min_z = t1_z.min(t2_z);

        let t_max_x = t1_x.max(t2_x);
        let t_max_y = t1_y.max(t2_y);
        let t_max_z = t1_z.max(t2_z);

        let t_min = t_min_x.max(t_min_y.max(t_min_z));
        let t_max = t_max_x.min(t_max_y.min(t_max_z));

        let mask = t_max.cmpgt(Vec4::ZERO) & t_max.cmpgt(t_min) & t_min.cmplt(packet.t);
        if mask.any() {
            Some(t_min.into())
        } else {
            None
        }
    }

    pub fn contains(&self, pos: Vec3) -> bool {
        let (min, max) = self.points();
        (pos.cmpgt(min) & pos.cmplt(max)).all()
    }

    #[inline]
    pub fn grow(&mut self, pos: Vec3) {
        let (min, max) = self.points();
        let min = min.min(pos);
        let max = max.max(pos);

        for i in 0..3 {
            self.min[i] = min[i];
            self.max[i] = max[i];
        }
    }

    #[inline]
    pub fn grow_bb(&mut self, aabb: &Self) {
        let (min, max) = self.points();
        let (a_min, a_max) = aabb.points();

        let min = min.min(a_min);
        let max = max.max(a_max);

        self.min = min;
        self.max = max;
    }

    #[inline]
    pub fn grow_bbs(&mut self, aabbs: &[Self]) {
        let (mut min, mut max) = self.points();
        for bb in aabbs {
            let (a_min, a_max) = bb.points();
            min = min.min(a_min);
            max = max.max(a_max);
        }

        self.min = min;
        self.max = max;
    }

    #[inline]
    pub fn shrink(&mut self, aabb: &Self) {
        let (min, max) = self.points();
        let (a_min, a_max) = aabb.points();

        let min = min.max(a_min);
        let max = max.min(a_max);

        self.min = min;
        self.max = max;
    }

    #[inline]
    pub fn with_offset(mut self, delta: f32) -> Self {
        let delta = Vec3::splat(delta);

        let min = self.min - delta;
        let max = self.max + delta;

        self.min = min;
        self.max = max;
        self
    }

    #[inline]
    pub fn offset_by(&mut self, delta: f32) {
        let delta = Vec3::splat(delta);

        let min = self.min - delta;
        let max = self.max + delta;

        self.min = min;
        self.max = max;
    }

    #[inline]
    pub fn volume(&self) -> f32 {
        let length = Vec3A::from(self.max) - Vec3A::from(self.min);
        length.x * length.y * length.z
    }

    #[inline]
    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    #[inline]
    pub fn area(&self) -> f32 {
        let e = Vec3A::from(self.max) - Vec3A::from(self.min);
        let value: f32 = e.x * e.y + e.x * e.z + e.y * e.z;

        0.0_f32.max(value)
    }

    #[inline]
    pub fn half_area(&self) -> f32 {
        let d = self.diagonal();
        (d[0] + d[1]) * d[2] + d[0] * d[1]
    }

    #[inline]
    pub fn lengths(&self) -> Vec3 {
        self.max - self.min
    }

    #[inline]
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

    #[inline]
    pub fn all_corners(&self) -> [Vec3; 8] {
        let lengths = self.lengths();

        let x_l = Vec3::new(lengths.x, 0.0, 0.0);
        let y_l = Vec3::new(0.0, lengths.y, 0.0);
        let z_l = Vec3::new(0.0, 0.0, lengths.z);

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

    #[inline]
    pub fn extend(&self, axis: usize) -> f32 {
        self.max[axis] - self.min[axis]
    }

    #[inline]
    pub fn points(&self) -> (Vec3, Vec3) {
        (self.min, self.max)
    }

    #[inline]
    pub fn diagonal(&self) -> Vec3 {
        self.max - self.min
    }

    #[inline]
    pub fn longest_extent(&self) -> f32 {
        self.diagonal()[self.longest_axis()]
    }
}

impl<E: Debug + Copy + Send + Sync + Default, const COUNT: usize> From<[Vec3; COUNT]> for Aabb<E> {
    fn from(points: [Vec3; COUNT]) -> Self {
        Self::from_points(&points)
    }
}

impl<E: Debug + Copy + Send + Sync + Default> From<(Vec3, Vec3)> for Aabb<E> {
    fn from((min, max): (Vec3, Vec3)) -> Self {
        Self {
            min,
            max,
            ..Default::default()
        }
    }
}

impl<E: Debug + Copy + Send + Sync + Default> From<Aabb<E>> for (Vec3, Vec3) {
    fn from(bb: Aabb<E>) -> Self {
        (bb.min, bb.max)
    }
}

impl<E: Debug + Copy> Bounds<E> for Aabb<E> {
    fn bounds(&self) -> Aabb<E> {
        *self
    }
}

impl<E: Debug + Copy + Send + Sync> crate::Primitive<E> for Aabb<E> {
    fn center(&self) -> Vec3 {
        self.center()
    }

    fn aabb(&self) -> Self {
        *self
    }
}

impl<E: Debug + Copy> Index<usize> for &Aabb<E> {
    type Output = Vec3;

    fn index(&self, index: usize) -> &Vec3 {
        if index == 0 {
            &self.min
        } else {
            &self.max
        }
    }
}

impl<E: Debug + Copy> Index<usize> for Aabb<E> {
    type Output = Vec3;

    fn index(&self, index: usize) -> &Vec3 {
        if index == 0 {
            &self.min
        } else {
            &self.max
        }
    }
}

#[macro_export]
/// Creates an AABB from a list of vertices
macro_rules! aabb {
    ($($x:expr),*) => {{
        let mut bb = Aabb::empty();
        $(
            bb.grow($x);
        )*

        bb.offset_by(1e-4);
        bb
    }};
}
