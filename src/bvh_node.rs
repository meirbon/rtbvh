use glam::*;
use serde::{Deserialize, Serialize};
use std::fmt::{Display, Formatter};

use crate::{Aabb, RayPacket4};

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
            // left_first: -1,
            // count: -1,
        }
    }

    #[inline(always)]
    pub fn intersect(&self, origin: Vec3A, dir_inverse: Vec3A, t: f32) -> Option<(f32, f32)> {
        if self.is_valid() {
            self.bounds.intersect(origin, dir_inverse, t)
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn intersect4(
        &self,
        packet: &RayPacket4,
        inv_dir_x: Vec4,
        inv_dir_y: Vec4,
        inv_dir_z: Vec4,
    ) -> Option<[f32; 4]> {
        if self.is_valid() {
            self.bounds
                .intersect4(packet, inv_dir_x, inv_dir_y, inv_dir_z)
        } else {
            None
        }
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

    pub fn depth_test<I>(
        tree: &[BvhNode],
        prim_indices: &[u32],
        origin: Vec3A,
        dir: Vec3A,
        t_min: f32,
        t_max: f32,
        depth_test: I,
    ) -> (f32, u32)
    where
        I: Fn(usize, f32, f32) -> Option<(f32, u32)>,
    {
        let mut t = t_max;
        let dir_inverse = Vec3A::new(1.0, 1.0, 1.0) / dir;

        if tree[0].intersect(origin, dir_inverse, t).is_none() {
            return (t_max, 0);
        }

        let mut depth: i32 = 0;
        let mut hit_stack = [0; 64];
        let mut stack_ptr: i32 = 0;

        while stack_ptr >= 0 {
            #[cfg(all(
                any(target_arch = "x86_64", target_arch = "x86"),
                not(feature = "wasm_support")
            ))]
            unsafe {
                core::arch::x86_64::_mm_prefetch(
                    tree.as_ptr().add(hit_stack[stack_ptr as usize] as usize) as *const i8,
                    core::arch::x86_64::_MM_HINT_T0,
                )
            };

            depth += 1;
            let node = &tree[hit_stack[stack_ptr as usize] as usize];
            stack_ptr -= 1;

            if node.bounds.extra1 > -1 {
                // Leaf node
                for i in 0..node.bounds.extra1 {
                    let prim_id = prim_indices[(node.bounds.extra2 + i) as usize];
                    if let Some((new_t, d)) = depth_test(prim_id as usize, t_min, t) {
                        t = new_t;
                        depth += d as i32;
                    }
                }
            } else if let Some(left_first) = node.get_left_first() {
                let hit_left = tree[left_first as usize].intersect(origin, dir_inverse, t);
                let hit_right = tree[(left_first + 1) as usize].intersect(origin, dir_inverse, t);
                let new_stack_ptr = Self::sort_nodes(
                    hit_left,
                    hit_right,
                    hit_stack.as_mut(),
                    stack_ptr,
                    left_first,
                );
                stack_ptr = new_stack_ptr;
            }
        }

        (t, depth as u32)
    }

    pub fn traverse<I, R>(
        tree: &[BvhNode],
        prim_indices: &[u32],
        origin: Vec3A,
        dir: Vec3A,
        t_min: f32,
        t_max: f32,
        mut intersection_test: I,
    ) -> Option<R>
    where
        I: FnMut(usize, f32, f32) -> Option<(f32, R)>,
        R: Copy,
    {
        let mut hit_stack = [0; 64];
        let mut stack_ptr: i32 = 0;
        let mut t = t_max;
        let mut hit_record = None;

        let dir_inverse = Vec3A::ONE / dir;
        while stack_ptr >= 0 {
            #[cfg(all(
                any(target_arch = "x86_64", target_arch = "x86"),
                not(feature = "wasm_support")
            ))]
            unsafe {
                core::arch::x86_64::_mm_prefetch(
                    tree.as_ptr().add(hit_stack[stack_ptr as usize] as usize) as *const i8,
                    core::arch::x86_64::_MM_HINT_T0,
                )
            };

            let node = &tree[hit_stack[stack_ptr as usize] as usize];
            stack_ptr -= 1;

            if node.is_leaf() {
                // Leaf node
                for i in 0..node.bounds.extra1 {
                    let prim_id = prim_indices[(node.bounds.extra2 + i) as usize];
                    if let Some((new_t, new_hit)) = intersection_test(prim_id as usize, t_min, t) {
                        t = new_t;
                        hit_record = Some(new_hit);
                    }
                }
            } else if let Some(left_first) = node.get_left_first() {
                let hit_left = tree[left_first as usize].intersect(origin, dir_inverse, t);
                let hit_right = tree[(left_first + 1) as usize].intersect(origin, dir_inverse, t);
                stack_ptr = Self::sort_nodes(
                    hit_left,
                    hit_right,
                    hit_stack.as_mut(),
                    stack_ptr,
                    left_first,
                );
            }
        }

        hit_record
    }

    pub fn traverse_t<I>(
        tree: &[BvhNode],
        prim_indices: &[u32],
        origin: Vec3A,
        dir: Vec3A,
        t_min: f32,
        t_max: f32,
        mut intersection_test: I,
    ) -> Option<f32>
    where
        I: FnMut(usize, f32, f32) -> Option<f32>,
    {
        let mut hit_stack = [0; 64];
        let mut stack_ptr: i32 = 0;
        let mut t = t_max;

        let dir_inverse = Vec3A::new(1.0, 1.0, 1.0) / dir;
        hit_stack[stack_ptr as usize] = 0;
        while stack_ptr >= 0 {
            #[cfg(all(
                any(target_arch = "x86_64", target_arch = "x86"),
                not(feature = "wasm_support")
            ))]
            unsafe {
                core::arch::x86_64::_mm_prefetch(
                    tree.as_ptr().add(hit_stack[stack_ptr as usize] as usize) as *const i8,
                    core::arch::x86_64::_MM_HINT_T0,
                )
            };

            let node = &tree[hit_stack[stack_ptr as usize] as usize];
            stack_ptr -= 1;

            if node.bounds.extra1 > -1 {
                // Leaf node
                for i in 0..node.bounds.extra1 {
                    let prim_id = prim_indices[(node.bounds.extra2 + i) as usize];
                    if let Some(new_t) = intersection_test(prim_id as usize, t_min, t) {
                        t = new_t;
                    }
                }
            } else if let Some(left_first) = node.get_left_first() {
                let hit_left = tree[left_first as usize].intersect(origin, dir_inverse, t);
                let hit_right = tree[(left_first + 1) as usize].intersect(origin, dir_inverse, t);
                stack_ptr = Self::sort_nodes(
                    hit_left,
                    hit_right,
                    hit_stack.as_mut(),
                    stack_ptr,
                    left_first,
                );
            }
        }

        if t < t_max {
            Some(t)
        } else {
            None
        }
    }

    pub fn occludes<I>(
        tree: &[BvhNode],
        prim_indices: &[u32],
        origin: Vec3A,
        dir: Vec3A,
        t_min: f32,
        t_max: f32,
        mut intersection_test: I,
    ) -> bool
    where
        I: FnMut(usize, f32, f32) -> bool,
    {
        let mut hit_stack = [0; 64];
        let mut stack_ptr: i32 = 0;

        let dir_inverse = Vec3A::new(1.0, 1.0, 1.0) / dir;
        hit_stack[stack_ptr as usize] = 0;
        while stack_ptr >= 0 {
            #[cfg(all(
                any(target_arch = "x86_64", target_arch = "x86"),
                not(feature = "wasm_support")
            ))]
            unsafe {
                core::arch::x86_64::_mm_prefetch(
                    tree.as_ptr().add(hit_stack[stack_ptr as usize] as usize) as *const i8,
                    core::arch::x86_64::_MM_HINT_T0,
                )
            };

            let node = &tree[hit_stack[stack_ptr as usize] as usize];
            stack_ptr -= 1;

            if node.bounds.extra1 > -1 {
                // Leaf node
                for i in 0..node.bounds.extra1 {
                    let prim_id = prim_indices[(node.bounds.extra2 + i) as usize];
                    if intersection_test(prim_id as usize, t_min, t_max) {
                        return true;
                    }
                }
            } else if let Some(left_first) = node.get_left_first() {
                let hit_left = tree[left_first as usize].intersect(origin, dir_inverse, t_max);
                let hit_right =
                    tree[(left_first + 1) as usize]
                        .bounds
                        .intersect(origin, dir_inverse, t_max);
                stack_ptr = Self::sort_nodes(
                    hit_left,
                    hit_right,
                    hit_stack.as_mut(),
                    stack_ptr,
                    left_first,
                );
            }
        }

        false
    }

    #[inline]
    fn sort_nodes(
        left: Option<(f32, f32)>,
        right: Option<(f32, f32)>,
        hit_stack: &mut [i32],
        mut stack_ptr: i32,
        left_first: u32,
    ) -> i32 {
        if let (Some((t_near_left, _)), Some((t_near_right, _))) = (left, right) {
            if t_near_left < t_near_right {
                stack_ptr += 1;
                hit_stack[stack_ptr as usize] = left_first as i32;
                stack_ptr += 1;
                hit_stack[stack_ptr as usize] = left_first as i32 + 1;
            } else {
                stack_ptr += 1;
                hit_stack[stack_ptr as usize] = left_first as i32 + 1;
                stack_ptr += 1;
                hit_stack[stack_ptr as usize] = left_first as i32;
            }
        } else if left.is_some() {
            stack_ptr += 1;
            hit_stack[stack_ptr as usize] = left_first as i32;
        } else if right.is_some() {
            stack_ptr += 1;
            hit_stack[stack_ptr as usize] = left_first as i32 + 1;
        }

        stack_ptr
    }

    #[inline]
    fn sort_nodes4(
        left: Option<[f32; 4]>,
        right: Option<[f32; 4]>,
        hit_stack: &mut [i32],
        mut stack_ptr: i32,
        left_first: u32,
    ) -> i32 {
        let left_first = left_first as i32;
        if let (Some(left), Some(right)) = (left, right) {
            let t_near_left = Vec4::from(left);
            let t_near_right = Vec4::from(right);

            if t_near_left.cmplt(t_near_right).bitmask() > 0 {
                stack_ptr += 1;
                hit_stack[stack_ptr as usize] = left_first;
                stack_ptr += 1;
                hit_stack[stack_ptr as usize] = left_first + 1;
            } else {
                stack_ptr += 1;
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

    pub fn traverse4<I: FnMut(usize, &mut RayPacket4)>(
        tree: &[BvhNode],
        prim_indices: &[u32],
        packet: &mut RayPacket4,
        mut intersection_test: I,
    ) {
        let mut hit_stack = [0; 64];
        let mut stack_ptr: i32 = 0;

        let one = Vec4::ONE;
        let inv_dir_x = one / Vec4::from(packet.direction_x);
        let inv_dir_y = one / Vec4::from(packet.direction_y);
        let inv_dir_z = one / Vec4::from(packet.direction_z);

        while stack_ptr >= 0 {
            #[cfg(all(
                any(target_arch = "x86_64", target_arch = "x86"),
                not(feature = "wasm_support")
            ))]
            unsafe {
                core::arch::x86_64::_mm_prefetch(
                    tree.as_ptr().add(hit_stack[stack_ptr as usize] as usize) as *const i8,
                    core::arch::x86_64::_MM_HINT_T0,
                )
            };

            let node = &tree[hit_stack[stack_ptr as usize] as usize];
            stack_ptr -= 1;

            if node.bounds.extra1 > -1 {
                // Leaf node
                for i in 0..node.bounds.extra1 {
                    let prim_id = prim_indices[(node.bounds.extra2 + i) as usize] as usize;
                    intersection_test(prim_id, packet);
                }
            } else if let Some(left_first) = node.get_left_first() {
                let hit_left =
                    tree[left_first as usize].intersect4(packet, inv_dir_x, inv_dir_y, inv_dir_z);
                let hit_right = tree[(left_first + 1) as usize]
                    .intersect4(packet, inv_dir_x, inv_dir_y, inv_dir_z);

                stack_ptr = Self::sort_nodes4(
                    hit_left,
                    hit_right,
                    hit_stack.as_mut(),
                    stack_ptr,
                    left_first,
                );
            }
        }
    }
}
