use glam::*;
use serde::{Deserialize, Serialize};
use std::fmt::{Display, Formatter};

use crate::{Aabb, Ray, RayPacket4};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[repr(C)]
pub struct BvhNode {
    pub bounds: Aabb<i32>,
}

impl Default for BvhNode {
    fn default() -> Self {
        Self {
            bounds: Aabb::default(),
        }
    }
}

impl Display for BvhNode {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.bounds)
    }
}

pub struct NewNodeInfo {
    pub left: usize,
    pub left_box: Aabb,
    pub left_count: i32,
    pub left_left_first: i32,
    pub right_box: Aabb,
    pub right_count: i32,
    pub right_left_first: i32,
}

pub struct NodeUpdatePayLoad {
    pub index: usize,
    pub bounds: Aabb,
    pub left_first: i32,
    pub count: i32,
}

#[allow(dead_code)]
impl BvhNode {
    const BINS: usize = 7;
    const MAX_PRIMITIVES: i32 = 5;
    const MAX_DEPTH: u32 = 32;

    pub fn new() -> BvhNode {
        BvhNode {
            bounds: Aabb::new(),
        }
    }

    #[inline(always)]
    pub fn intersect(&self, ray: &Ray) -> Option<f32> {
        self.bounds.intersect(ray)
    }

    #[inline(always)]
    pub fn intersect4(&self, packet: &RayPacket4) -> Option<[f32; 4]> {
        self.bounds.intersect4(packet)
    }

    #[inline(always)]
    pub fn set_left_first(&mut self, left_first: Option<u32>) {
        self.bounds.extra2 = match left_first {
            Some(l) => l as i32,
            None => -1,
        }
    }

    #[inline(always)]
    pub fn with_left_first(mut self, left_first: Option<u32>) -> Self {
        self.set_left_first(left_first);
        self
    }

    #[inline(always)]
    pub fn get_left_first(&self) -> Option<u32> {
        if self.bounds.extra2 >= 0 {
            Some(self.bounds.extra2 as u32)
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn get_left_first_unchecked(&self) -> i32 {
        self.bounds.extra2
    }

    #[inline(always)]
    pub fn set_count(&mut self, count: Option<u32>) {
        self.bounds.extra1 = match count {
            Some(c) => c as i32,
            None => -1,
        };
    }

    #[inline(always)]
    pub fn with_count(mut self, count: Option<u32>) -> Self {
        self.set_count(count);
        self
    }

    #[inline(always)]
    pub fn get_count(&self) -> Option<u32> {
        if self.bounds.extra1 >= 0 {
            Some(self.bounds.extra1 as u32)
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn get_count_unchecked(&self) -> i32 {
        self.bounds.extra1
    }

    #[inline(always)]
    pub fn has_children(&self) -> bool {
        self.bounds.extra1 < 0 && self.bounds.extra2 >= 0
    }

    #[inline(always)]
    pub fn is_leaf(&self) -> bool {
        self.bounds.extra1 >= 0
    }

    #[inline(always)]
    pub fn is_valid(&self) -> bool {
        self.bounds.extra2 >= 0
    }

    #[inline]
    pub(crate) fn sort_nodes(
        left: Option<f32>,
        right: Option<f32>,
        hit_stack: &mut [i32],
        mut stack_ptr: i32,
        left_first: i32,
    ) -> i32 {
        if let (Some(t_near_left), Some(t_near_right)) = (left, right) {
            stack_ptr += 1;
            if t_near_left < t_near_right {
                hit_stack[stack_ptr as usize] = left_first;
                stack_ptr += 1;
                hit_stack[stack_ptr as usize] = left_first + 1;
            } else {
                hit_stack[stack_ptr as usize] = left_first + 1;
                stack_ptr += 1;
                hit_stack[stack_ptr as usize] = left_first;
            }
        } else if left.is_some() {
            stack_ptr += 1;
            hit_stack[stack_ptr as usize] = left_first;
        } else if right.is_some() {
            stack_ptr += 1;
            hit_stack[stack_ptr as usize] = left_first + 1;
        }

        stack_ptr
    }

    #[inline]
    pub(crate) fn sort_nodes4(
        left: Option<[f32; 4]>,
        right: Option<[f32; 4]>,
        hit_stack: &mut [i32],
        mut stack_ptr: i32,
        left_first: i32,
    ) -> i32 {
        let left_first = left_first as i32;
        if let (Some(left), Some(right)) = (left, right) {
            let t_near_left = Vec4::from(left);
            let t_near_right = Vec4::from(right);

            stack_ptr += 1;
            if t_near_left.cmplt(t_near_right).bitmask() > 0 {
                hit_stack[stack_ptr as usize] = left_first;
                stack_ptr += 1;
                hit_stack[stack_ptr as usize] = left_first + 1;
            } else {
                hit_stack[stack_ptr as usize] = left_first + 1;
                stack_ptr += 1;
                hit_stack[stack_ptr as usize] = left_first;
            }
        } else if left.is_some() {
            stack_ptr += 1;
            hit_stack[stack_ptr as usize] = left_first;
        } else if right.is_some() {
            stack_ptr += 1;
            hit_stack[stack_ptr as usize] = left_first + 1;
        }

        stack_ptr
    }
}
