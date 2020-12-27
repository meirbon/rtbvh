use crate::builders::spatial_sah::SpatialTriangle;
use crate::builders::*;
use crate::bvh_node::*;
use crate::mbvh_node::*;
use crate::{aabb::Bounds, builders::BuildAlgorithm};
use crate::{RayPacket4, AABB};
use glam::*;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;

pub trait Primitive: Debug + Copy + Send + Sync {
    fn center(&self) -> [f32; 3];
    fn aabb(&self) -> AABB;
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum BuildType {
    None,
    LocallyOrderedClustered,
    BinnedSAH,
    Spatial,
}

pub struct Builder<'a, T: Primitive> {
    pub aabbs: &'a [AABB],
    pub primitives: &'a [T],
}

impl<'a, T: Primitive> Builder<'a, T> {
    pub fn construct_spatial_sah(self) -> BVH
    where
        T: SpatialTriangle,
    {
        spatial_sah::SpatialSahBuilder::new(self.aabbs, self.primitives).build()
    }

    pub fn construct_binned_sah(self) -> BVH {
        binned_sah::BinnedSahBuilder::new(self.aabbs, self.primitives).build()
    }

    pub fn construct_locally_ordered_clustered(self) -> BVH {
        locb::LocallyOrderedClusteringBuilder::new(self.aabbs, self.primitives).build()
    }
}

// A BVH structure with nodes and primitive indices
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BVH {
    pub(crate) nodes: Vec<BVHNode>,
    pub(crate) prim_indices: Vec<u32>,
    pub(crate) build_type: BuildType,
}

impl Default for BVH {
    fn default() -> Self {
        Self {
            nodes: Default::default(),
            prim_indices: Default::default(),
            build_type: BuildType::None,
        }
    }
}

impl BVH {
    pub fn nodes(&self) -> &[BVHNode] {
        self.nodes.as_slice()
    }

    pub fn indices(&self) -> &[u32] {
        self.prim_indices.as_slice()
    }

    pub fn build_type(&self) -> BuildType {
        self.build_type
    }

    pub fn prim_count(&self) -> usize {
        self.prim_indices.len()
    }

    pub fn refit(&mut self, new_aabbs: &[AABB]) {
        for i in (0..self.nodes.len()).rev() {
            // Create new aabb
            let mut aabb = AABB::new();

            // Check if node is valid
            if let Some(left_first) = self.nodes[i].get_left_first() {
                let count = self.nodes[i].get_count_unchecked();
                if self.nodes[i].is_leaf() {
                    // Grow using primitives
                    for i in 0..count {
                        let prim_id = self.prim_indices[(left_first + i as u32) as usize] as usize;
                        aabb.grow_bb(&new_aabbs[prim_id]);
                    }
                } else {
                    // Grow using child nodes
                    // Left node
                    aabb.grow_bb(&self.nodes[left_first as usize].bounds);
                    // Right node
                    aabb.grow_bb(&self.nodes[(left_first + 1) as usize].bounds);
                }
            }

            // Overwrite AABB
            self.nodes[i].bounds = aabb;
        }
    }

    /// Intersect bvh and retrieve the final t value.
    /// intersection_test takes the following arguments (prim_id, t_min, t_max)
    /// and should return a hit record defined by the user
    #[inline(always)]
    pub fn traverse<I, R>(
        &self,
        origin: &[f32; 3],
        direction: &[f32; 3],
        t_min: f32,
        t_max: f32,
        intersection_test: I,
    ) -> Option<R>
    where
        I: FnMut(usize, f32, f32) -> Option<(f32, R)>,
        R: Copy,
    {
        BVHNode::traverse(
            self.nodes.as_slice(),
            self.prim_indices.as_slice(),
            Vec3A::from(*origin),
            Vec3A::from(*direction),
            t_min,
            t_max,
            intersection_test,
        )
    }

    /// Intersect bvh and retrieve the final t value.
    /// intersection_test takes the following arguments (prim_id, t_min, t_max)
    #[inline(always)]
    pub fn traverse_t<I>(
        &self,
        origin: &[f32; 3],
        direction: &[f32; 3],
        t_min: f32,
        t_max: f32,
        intersection_test: I,
    ) -> Option<f32>
    where
        I: FnMut(usize, f32, f32) -> Option<f32>,
    {
        BVHNode::traverse_t(
            self.nodes.as_slice(),
            self.prim_indices.as_slice(),
            Vec3A::from(*origin),
            Vec3A::from(*direction),
            t_min,
            t_max,
            intersection_test,
        )
    }

    /// Traverses the bvh and checks whether the ray is occluded
    /// intersection_test takes the following arguments (prim_id, t_min, t_max)
    /// and should return whether the ray hit the object
    #[inline(always)]
    pub fn occludes<I>(
        &self,
        origin: &[f32; 3],
        direction: &[f32; 3],
        t_min: f32,
        t_max: f32,
        intersection_test: I,
    ) -> bool
    where
        I: FnMut(usize, f32, f32) -> bool,
    {
        BVHNode::occludes(
            self.nodes.as_slice(),
            self.prim_indices.as_slice(),
            Vec3A::from(*origin),
            Vec3A::from(*direction),
            t_min,
            t_max,
            intersection_test,
        )
    }

    /// intersection_test takes the following arguments (prim_id, t_min, t_max)
    /// and should optionally return a t value and depth value.
    #[inline(always)]
    pub fn depth_test<I>(
        &self,
        origin: &[f32; 3],
        direction: &[f32; 3],
        t_min: f32,
        t_max: f32,
        intersection_test: I,
    ) -> (f32, u32)
    where
        I: Fn(usize, f32, f32) -> Option<(f32, u32)>,
    {
        BVHNode::depth_test(
            self.nodes.as_slice(),
            self.prim_indices.as_slice(),
            Vec3A::from(*origin),
            Vec3A::from(*direction),
            t_min,
            t_max,
            intersection_test,
        )
    }

    /// intersection_test takes the following arguments (prim_id, ray_packet)
    #[inline(always)]
    pub fn traverse4<I>(&self, packet: &mut RayPacket4, intersection_test: I)
    where
        I: FnMut(usize, &mut RayPacket4),
    {
        BVHNode::traverse4(
            self.nodes.as_slice(),
            self.prim_indices.as_slice(),
            packet,
            intersection_test,
        );
    }

    fn traverse_check(&self, cur_node: &BVHNode, checks: &mut [u8]) {
        if let Some(left_first) = cur_node.get_left_first() {
            if cur_node.is_leaf() {
                for i in 0..cur_node.count {
                    let prim_id = self.prim_indices[(left_first as i32 + i) as usize] as usize;
                    debug_assert!(
                        prim_id < checks.len(),
                        "prim_id: {}, max_prim_id: {}",
                        prim_id,
                        checks.len()
                    );
                    checks[prim_id] = 1;
                }
            } else {
                let left_node = &self.nodes[left_first as usize];
                let right_node = &self.nodes[left_first as usize + 1];

                self.traverse_check(left_node, checks);
                self.traverse_check(right_node, checks);
            }
        }
    }

    /// Validates the current bvh for correctness in terms of primitive ids and
    /// tree structure
    pub fn validate(&self, prim_count: usize) {
        let mut found_indices = vec![0 as u8; prim_count];
        self.traverse_check(&self.nodes[0], found_indices.as_mut());

        for i in 0..found_indices.len() {
            if found_indices[i] == 0 {
                println!("prim ({}) not referenced", i);
            }
        }
    }

    pub fn as_raw_indices(self) -> Vec<u32> {
        self.prim_indices
    }

    pub fn as_raw_nodes(self) -> Vec<BVHNode> {
        self.nodes
    }

    pub fn as_raw(self) -> (Vec<BVHNode>, Vec<u32>) {
        (self.nodes, self.prim_indices)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MBVH {
    pub(crate) nodes: Vec<BVHNode>,
    pub(crate) m_nodes: Vec<MBVHNode>,
    pub(crate) prim_indices: Vec<u32>,
}

impl Default for MBVH {
    fn default() -> Self {
        Self {
            nodes: Default::default(),
            m_nodes: Default::default(),
            prim_indices: Default::default(),
        }
    }
}

impl MBVH {
    pub fn nodes(&self) -> &[BVHNode] {
        self.nodes.as_slice()
    }

    pub fn quad_nodes(&self) -> &[MBVHNode] {
        self.m_nodes.as_slice()
    }

    pub fn indices(&self) -> &[u32] {
        self.prim_indices.as_slice()
    }

    pub fn prim_count(&self) -> usize {
        self.prim_indices.len()
    }

    pub fn construct_from_raw(nodes: &[BVHNode], prim_indices: &[u32]) -> Self {
        debug_assert!(nodes.len() >= 1);
        let mut m_nodes = vec![MBVHNode::new(); nodes.len()];
        let mut pool_ptr = 1;

        if nodes.len() <= 4 {
            for i in 0..nodes.len() {
                let cur_node = &nodes[i];
                m_nodes[0].set_bounds_bb(i, &cur_node.bounds);
                m_nodes[0].children[i] = cur_node.left_first;
                m_nodes[0].counts[i] = cur_node.count;
            }

            return MBVH {
                nodes: nodes.to_vec(),
                m_nodes,
                prim_indices: prim_indices.to_vec(),
            };
        }

        MBVHNode::merge_nodes(0, 0, nodes, m_nodes.as_mut_slice(), &mut pool_ptr);

        MBVH {
            nodes: nodes.to_vec(),
            m_nodes,
            prim_indices: prim_indices.to_vec(),
        }
    }

    pub fn construct(bvh: &BVH) -> Self {
        debug_assert!(bvh.nodes.len() >= 1);
        let mut m_nodes = vec![MBVHNode::new(); bvh.nodes.len()];
        let mut pool_ptr = 1;

        if bvh.nodes.len() <= 4 {
            for i in 0..bvh.nodes.len() {
                let cur_node = &bvh.nodes[i];
                m_nodes[0].set_bounds_bb(i, &cur_node.bounds);
                m_nodes[0].children[i] = cur_node.left_first;
                m_nodes[0].counts[i] = cur_node.count;
            }

            return MBVH {
                nodes: bvh.nodes.clone(),
                m_nodes,
                prim_indices: bvh.prim_indices.clone(),
            };
        }

        MBVHNode::merge_nodes(
            0,
            0,
            bvh.nodes.as_slice(),
            m_nodes.as_mut_slice(),
            &mut pool_ptr,
        );

        MBVH {
            nodes: bvh.nodes.clone(),
            m_nodes,
            prim_indices: bvh.prim_indices.clone(),
        }
    }

    /// Intersect bvh and retrieve the final t value.
    /// intersection_test takes the following arguments (prim_id, t_min, t_max)
    /// and should return a hit record defined by the user
    #[inline(always)]
    pub fn traverse<I, R>(
        &self,
        origin: &[f32; 3],
        direction: &[f32; 3],
        t_min: f32,
        t_max: f32,
        intersection_test: I,
    ) -> Option<R>
    where
        I: FnMut(usize, f32, f32) -> Option<(f32, R)>,
        R: Copy,
    {
        MBVHNode::traverse(
            self.m_nodes.as_slice(),
            self.prim_indices.as_slice(),
            Vec3A::from(*origin),
            Vec3A::from(*direction),
            t_min,
            t_max,
            intersection_test,
        )
    }

    /// Intersect bvh and retrieve the final t value.
    /// intersection_test takes the following arguments (prim_id, t_min, t_max)
    #[inline(always)]
    pub fn traverse_t<I>(
        &self,
        origin: &[f32; 3],
        direction: &[f32; 3],
        t_min: f32,
        t_max: f32,
        intersection_test: I,
    ) -> Option<f32>
    where
        I: FnMut(usize, f32, f32) -> Option<f32>,
    {
        MBVHNode::traverse_t(
            self.m_nodes.as_slice(),
            self.prim_indices.as_slice(),
            Vec3A::from(*origin),
            Vec3A::from(*direction),
            t_min,
            t_max,
            intersection_test,
        )
    }

    /// Traverses the bvh and checks whether the ray is occluded
    /// intersection_test takes the following arguments (prim_id, t_min, t_max)
    /// and should return whether the ray hit the object
    #[inline(always)]
    pub fn occludes<I>(
        &self,
        origin: &[f32; 3],
        direction: &[f32; 3],
        t_min: f32,
        t_max: f32,
        intersection_test: I,
    ) -> bool
    where
        I: FnMut(usize, f32, f32) -> bool,
    {
        MBVHNode::occludes(
            self.m_nodes.as_slice(),
            self.prim_indices.as_slice(),
            Vec3A::from(*origin),
            Vec3A::from(*direction),
            t_min,
            t_max,
            intersection_test,
        )
    }

    /// intersection_test takes the following arguments (prim_id, t_min, t_max)
    /// and should optionally return a t value and depth value.
    #[inline(always)]
    pub fn depth_test<I>(
        &self,
        origin: &[f32; 3],
        direction: &[f32; 3],
        t_min: f32,
        t_max: f32,
        depth_test: I,
    ) -> (f32, u32)
    where
        I: Fn(usize, f32, f32) -> Option<(f32, u32)>,
    {
        MBVHNode::depth_test(
            self.m_nodes.as_slice(),
            self.prim_indices.as_slice(),
            Vec3A::from(*origin),
            Vec3A::from(*direction),
            t_min,
            t_max,
            depth_test,
        )
    }

    /// intersection_test takes the following arguments (prim_id, ray_packet)
    #[inline(always)]
    pub fn traverse4<I>(&self, packet: &mut RayPacket4, intersection_test: I)
    where
        I: FnMut(usize, &mut RayPacket4),
    {
        MBVHNode::traverse4(
            self.m_nodes.as_slice(),
            self.prim_indices.as_slice(),
            packet,
            intersection_test,
        );
    }

    pub fn as_raw_indices(self) -> Vec<u32> {
        self.prim_indices
    }

    pub fn as_raw_nodes(self) -> Vec<MBVHNode> {
        self.m_nodes
    }

    pub fn as_raw(self) -> (Vec<MBVHNode>, Vec<u32>) {
        (self.m_nodes, self.prim_indices)
    }
}

impl From<BVH> for MBVH {
    fn from(bvh: BVH) -> Self {
        MBVH::construct(&bvh)
    }
}

impl Bounds for BVH {
    fn bounds(&self) -> AABB {
        self.nodes[0].bounds.clone()
    }
}

impl Bounds for MBVH {
    fn bounds(&self) -> AABB {
        self.nodes[0].bounds.clone()
    }
}
