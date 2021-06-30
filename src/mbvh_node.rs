use crate::{bvh_node::*, Ray};
use crate::{Aabb, RayPacket4};

use glam::*;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(align(8))]
pub(crate) struct MbvhHit {
    pub(crate) ids: [u8; 4],
    pub(crate) result: [bool; 4],
}

impl Default for MbvhHit {
    fn default() -> Self {
        Self {
            ids: [0; 4],
            result: [false; 4],
        }
    }
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Clone)]
#[repr(C)]
pub struct MbvhNode {
    min_x: [f32; 4],
    max_x: [f32; 4],
    min_y: [f32; 4],
    max_y: [f32; 4],
    min_z: [f32; 4],
    max_z: [f32; 4],
    pub children: [i32; 4],
    pub counts: [i32; 4],
}

impl std::default::Default for MbvhNode {
    fn default() -> Self {
        MbvhNode {
            min_x: [0.0; 4],
            max_x: [0.0; 4],
            min_y: [0.0; 4],
            max_y: [0.0; 4],
            min_z: [0.0; 4],
            max_z: [0.0; 4],
            children: [0; 4],
            counts: [0; 4],
        }
    }
}

impl MbvhNode {
    pub fn new() -> MbvhNode {
        let min_x = [1e34; 4];
        let min_y = [1e34; 4];
        let min_z = [1e34; 4];

        let max_x = [-1e34; 4];
        let max_y = [-1e34; 4];
        let max_z = [-1e34; 4];

        let children = [-1; 4];
        let counts = [-1; 4];

        MbvhNode {
            min_x,
            max_x,
            min_y,
            max_y,
            min_z,
            max_z,
            children,
            counts,
        }
    }

    pub fn points(&self) -> (Vec4, Vec4, Vec4, Vec4, Vec4, Vec4) {
        #[cfg(all(
            any(target_arch = "x86_64", target_arch = "x86"),
            not(feature = "wasm_support")
        ))]
        {
            (
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.min_x.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.max_x.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.min_y.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.max_y.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.min_z.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.max_z.as_ptr()) }),
            )
        }
        #[cfg(any(
            not(any(target_arch = "x86_64", target_arch = "x86")),
            feature = "wasm_support"
        ))]
        {
            (
                Vec4::from(self.min_x),
                Vec4::from(self.max_x),
                Vec4::from(self.min_y),
                Vec4::from(self.max_y),
                Vec4::from(self.min_z),
                Vec4::from(self.max_z),
            )
        }
    }

    pub fn min_points(&self) -> (Vec4, Vec4, Vec4) {
        #[cfg(all(
            any(target_arch = "x86_64", target_arch = "x86"),
            not(feature = "wasm_support")
        ))]
        {
            (
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.min_x.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.min_y.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.min_z.as_ptr()) }),
            )
        }
        #[cfg(any(
            not(any(target_arch = "x86_64", target_arch = "x86")),
            feature = "wasm_support"
        ))]
        {
            (
                Vec4::from(self.min_x),
                Vec4::from(self.min_y),
                Vec4::from(self.min_z),
            )
        }
    }

    pub fn max_points(&self) -> (Vec4, Vec4, Vec4) {
        #[cfg(all(
            any(target_arch = "x86_64", target_arch = "x86"),
            not(feature = "wasm_support")
        ))]
        {
            (
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.max_x.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.max_y.as_ptr()) }),
                Vec4::from(unsafe { core::arch::x86_64::_mm_load_ps(self.max_z.as_ptr()) }),
            )
        }
        #[cfg(any(
            not(any(target_arch = "x86_64", target_arch = "x86")),
            feature = "wasm_support"
        ))]
        {
            (
                Vec4::from(self.max_x),
                Vec4::from(self.max_y),
                Vec4::from(self.max_z),
            )
        }
    }

    pub fn set_bounds(&mut self, node_id: usize, min: &[f32; 3], max: &[f32; 3]) {
        debug_assert!(node_id < 4);
        self.min_x[node_id] = min[0];
        self.min_y[node_id] = min[1];
        self.min_z[node_id] = min[2];

        self.max_x[node_id] = max[0];
        self.max_y[node_id] = max[1];
        self.max_z[node_id] = max[2];
    }

    pub fn set_bounds_bb(&mut self, node_id: usize, bounds: &Aabb) {
        self.set_bounds(node_id, bounds.min.as_ref(), bounds.max.as_ref());
    }

    #[inline(always)]
    pub(crate) fn intersect(&self, ray: &Ray) -> MbvhHit {
        let origin_x: Vec4 = ray.origin.xxxx();
        let origin_y: Vec4 = ray.origin.yyyy();
        let origin_z: Vec4 = ray.origin.zzzz();

        let inv_dir_x: Vec4 = ray.inv_direction.xxxx();
        let inv_dir_y: Vec4 = ray.inv_direction.yyyy();
        let inv_dir_z: Vec4 = ray.inv_direction.zzzz();

        let (min_x, max_x, min_y, max_y, min_z, max_z) = self.points();

        let tx0: Vec4 = (min_x - origin_x) * inv_dir_x;
        let tx1: Vec4 = (max_x - origin_x) * inv_dir_x;
        let ty0: Vec4 = (min_y - origin_y) * inv_dir_y;
        let ty1: Vec4 = (max_y - origin_y) * inv_dir_y;
        let tz0: Vec4 = (min_z - origin_z) * inv_dir_z;
        let tz1: Vec4 = (max_z - origin_z) * inv_dir_z;

        let tx_min: Vec4 = tx0.min(tx1);
        let tx_max: Vec4 = tx0.max(tx1);
        let ty_min: Vec4 = ty0.min(ty1);
        let ty_max: Vec4 = ty0.max(ty1);
        let tz_min: Vec4 = tz0.min(tz1);
        let tz_max: Vec4 = tz0.max(tz1);

        let mut t_min = tx_min.max(ty_min.max(tz_min));
        let t_max = tx_max.min(ty_max.min(tz_max));

        let result = t_max.cmpge(t_min) & t_min.cmplt(Vec4::splat(ray.t));
        let result = result.bitmask();

        let result = [
            (result & 1) != 0,
            (result & 2) != 0,
            (result & 4) != 0,
            (result & 8) != 0,
        ];

        let mut ids = [0, 1, 2, 3];

        let t_minf = t_min.as_mut();

        if t_minf[0] > t_minf[1] {
            t_minf.swap(0, 1);
            ids.swap(0, 1);
        }
        if t_minf[2] > t_minf[3] {
            t_minf.swap(2, 3);
            ids.swap(2, 3);
        }
        if t_minf[0] > t_minf[2] {
            t_minf.swap(0, 2);
            ids.swap(0, 2);
        }
        if t_minf[1] > t_minf[3] {
            t_minf.swap(1, 3);
            ids.swap(1, 3);
        }
        if t_minf[2] > t_minf[3] {
            ids.swap(2, 3);
        }

        MbvhHit { ids, result }
    }

    #[inline(always)]
    pub(crate) fn intersect4(&self, packet: &RayPacket4) -> MbvhHit {
        let (min_x, max_x, min_y, max_y, min_z, max_z) = self.points();

        let mut result = BVec4A::new(false, false, false, false);
        let mut final_t_min = Vec4::splat(1e20);

        for i in 0..4 {
            let org_comp = Vec4::splat(packet.origin_x[i]);
            let dir_comp = Vec4::splat(packet.inv_direction_x[i]);

            let t1 = (min_x - org_comp) * dir_comp;
            let t2 = (max_x - org_comp) * dir_comp;

            let t_min = t1.min(t2);
            let t_max = t1.max(t2);

            let org_comp = Vec4::splat(packet.origin_y[i]);
            let dir_comp = Vec4::splat(packet.inv_direction_y[i]);

            let t1 = (min_y - org_comp) * dir_comp;
            let t2 = (max_y - org_comp) * dir_comp;

            let t_min = t_min.max(t1.min(t2));
            let t_max = t_max.min(t1.max(t2));

            let org_comp = Vec4::splat(packet.origin_z[i]);
            let dir_comp = Vec4::splat(packet.inv_direction_z[i]);

            let t1 = (min_z - org_comp) * dir_comp;
            let t2 = (max_z - org_comp) * dir_comp;

            let t_min: Vec4 = t_min.max(t1.min(t2));
            let t_max: Vec4 = t_max.min(t1.max(t2));

            let greater_than_min = t_max.cmpgt(t_min);
            let less_than_t = t_min.cmplt(Vec4::splat(packet.t[i]));
            result |= greater_than_min & less_than_t;

            final_t_min = final_t_min.min(t_min);
        }

        let result = result.bitmask();

        let result = [
            (result & 1) != 0,
            (result & 2) != 0,
            (result & 4) != 0,
            (result & 8) != 0,
        ];

        let ids = [0, 1, 2, 3];
        MbvhHit { ids, result }
    }

    #[inline(always)]
    pub fn merge_nodes(
        m_index: usize,
        cur_node: usize,
        bvh_pool: &[BvhNode],
        mbvh_pool: &mut [MbvhNode],
        pool_ptr: &mut usize,
    ) {
        let cur_node = &bvh_pool[cur_node];
        if cur_node.is_leaf() {
            panic!("Leaf nodes should not be attempted to be split!");
        } else if m_index >= mbvh_pool.len() {
            panic!("Index {} is out of bounds!", m_index);
        }

        let num_children = mbvh_pool[m_index].merge_node(cur_node, bvh_pool);

        for idx in 0..num_children {
            if mbvh_pool[m_index].children[idx] < 0 {
                mbvh_pool[m_index].set_bounds(idx, &[1e34; 3], &[-1e34; 3]);
                mbvh_pool[m_index].children[idx] = 0;
                mbvh_pool[m_index].counts[idx] = 0;
                continue;
            }

            if mbvh_pool[m_index].counts[idx] < 0 {
                let cur_node = mbvh_pool[m_index].children[idx] as usize;
                if !bvh_pool[cur_node].is_leaf() {
                    let new_idx = *pool_ptr;
                    *pool_ptr += 1;
                    mbvh_pool[m_index].children[idx] = new_idx as i32;
                    Self::merge_nodes(new_idx, cur_node, bvh_pool, mbvh_pool, pool_ptr);
                } else {
                    mbvh_pool[m_index].counts[idx] = 0;
                    mbvh_pool[m_index].children[idx] = 0;
                }
            }
        }
    }

    fn merge_node(&mut self, node: &BvhNode, pool: &[BvhNode]) -> usize {
        self.children = [-1; 4];
        self.counts = [-1; 4];
        let mut num_children = 0;

        let left_first = node.get_left_first();
        if left_first.is_none() {
            return num_children;
        }

        let left_first = left_first.unwrap();
        let left_node = &pool[left_first as usize];
        let right_node = &pool[(left_first + 1) as usize];

        if left_node.is_leaf() {
            let idx = num_children;
            if let Some(left_first) = left_node.get_left_first() {
                num_children += 1;
                self.children[idx] = left_first as i32;
                self.counts[idx] = left_node.get_count_unchecked();
                self.set_bounds_bb(idx, &left_node.bounds);
            }
        } else {
            // Node has children

            if let Some(left) = left_node.get_left_first() {
                let right = left + 1;

                let left_node = &pool[left as usize];
                let right_node = &pool[right as usize];

                if let Some(left_first) = left_node.get_left_first() {
                    let idx1 = num_children;
                    num_children += 1;
                    self.set_bounds_bb(idx1, &left_node.bounds);
                    if left_node.is_leaf() {
                        self.children[idx1] = left_first as i32;
                        self.counts[idx1] = left_node.get_count_unchecked();
                    } else {
                        self.children[idx1] = left as i32;
                        self.counts[idx1] = -1;
                    }
                }

                if let Some(left_first) = right_node.get_left_first() {
                    let idx2 = num_children;
                    num_children += 1;
                    self.set_bounds_bb(idx2, &right_node.bounds);
                    if right_node.is_leaf() {
                        self.children[idx2] = left_first as i32;
                        self.counts[idx2] = right_node.get_count_unchecked();
                    } else {
                        self.children[idx2] = right as i32;
                        self.counts[idx2] = -1;
                    }
                }
            }
        }

        if let Some(left_first) = right_node.get_left_first() {
            if right_node.is_leaf() {
                // Node only has a single child
                let idx = num_children;
                num_children += 1;
                self.set_bounds_bb(idx, &right_node.bounds);

                self.children[idx] = left_first as i32;
                self.counts[idx] = right_node.get_count_unchecked();
            } else if let Some(left) = right_node.get_left_first() {
                let right = left + 1;
                let left_node = &pool[left as usize];
                let right_node = &pool[right as usize];

                if let Some(left_first) = left_node.get_left_first() {
                    let idx1 = num_children;
                    num_children += 1;
                    self.set_bounds_bb(idx1, &left_node.bounds);
                    if left_node.is_leaf() {
                        self.children[idx1] = left_first as i32;
                        self.counts[idx1] = left_node.get_count_unchecked();
                    } else {
                        self.children[idx1] = left as i32;
                        self.counts[idx1] = -1;
                    }
                }

                if let Some(left_first) = right_node.get_left_first() {
                    let idx2 = num_children;
                    num_children += 1;
                    self.set_bounds_bb(idx2, &right_node.bounds);
                    if right_node.is_leaf() {
                        self.children[idx2] = left_first as i32;
                        self.counts[idx2] = right_node.get_count_unchecked();
                    } else {
                        self.children[idx2] = right as i32;
                        self.counts[idx2] = -1;
                    }
                }
            }
        }

        // In case this quad node isn't filled & not all nodes are leaf nodes, merge 1 more node
        let mut merged = true;
        while num_children < 4 {
            if !merged {
                break;
            }
            merged = false;

            for i in 0..3 {
                if self.counts[i] >= 0 {
                    continue;
                }

                if self.children[i] < 0 {
                    continue;
                }

                let left = self.children[i];
                let right = left + 1;
                let left_sub_node = &pool[left as usize];
                let right_sub_node = &pool[right as usize];

                if let Some(left_first) = left_sub_node.get_left_first() {
                    // Overwrite current node
                    self.set_bounds_bb(i, &left_sub_node.bounds);
                    if left_sub_node.is_leaf() {
                        self.children[i] = left_first as i32;
                        self.counts[i] = left_sub_node.get_count_unchecked();
                    } else {
                        self.children[i] = left;
                        self.counts[i] = -1;
                    }

                    // Add its right node
                    self.set_bounds_bb(num_children, &right_sub_node.bounds);
                    if right_sub_node.is_leaf() {
                        self.children[num_children] = right_sub_node.get_left_first_unchecked();
                        self.counts[num_children] = right_sub_node.get_count_unchecked();
                    } else {
                        self.children[num_children] = right;
                        self.counts[num_children] = -1;
                    }

                    num_children += 1;
                    merged = true;
                } else if let Some(left_first) = right_sub_node.get_left_first() {
                    if right_sub_node.is_leaf() {
                        self.children[i] = left_first as i32;
                        self.counts[i] = right_sub_node.get_count_unchecked();
                    } else {
                        self.children[i] = right;
                        self.counts[i] = -1;
                    }
                    merged = true;
                }

                break;
            }
        }

        num_children
    }
}
