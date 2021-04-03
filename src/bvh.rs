use crate::builders::spatial_sah::SpatialTriangle;
use crate::builders::*;
use crate::bvh_node::*;
use crate::mbvh_node::*;
use crate::{aabb::Bounds, builders::BuildAlgorithm};
use crate::{Aabb, RayPacket4};
use glam::*;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;

pub trait Primitive<AabbType: Debug + Copy + Send + Sync = i32>: Debug + Copy + Send + Sync {
    fn center(&self) -> [f32; 3];
    fn aabb(&self) -> Aabb<AabbType>;
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum BuildType {
    None,
    LocallyOrderedClustered,
    BinnedSAH,
    Spatial,
}

pub struct Builder<'a, T: Primitive<i32>> {
    pub aabbs: &'a [Aabb<i32>],
    pub primitives: &'a [T],
}

impl<'a, T: Primitive<i32>> Builder<'a, T> {
    pub fn construct_spatial_sah(self) -> Bvh
    where
        T: SpatialTriangle,
    {
        spatial_sah::SpatialSahBuilder::new(self.aabbs, self.primitives).build()
    }

    pub fn construct_binned_sah(self) -> Bvh {
        binned_sah::BinnedSahBuilder::new(self.aabbs, self.primitives).build()
    }

    pub fn construct_locally_ordered_clustered(self) -> Bvh {
        locb::LocallyOrderedClusteringBuilder::new(self.aabbs, self.primitives).build()
    }
}

// A BVH structure with nodes and primitive indices
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Bvh {
    pub(crate) nodes: Vec<BvhNode>,
    pub(crate) prim_indices: Vec<u32>,
    pub(crate) build_type: BuildType,
}

impl Default for Bvh {
    fn default() -> Self {
        Self {
            nodes: Default::default(),
            prim_indices: Default::default(),
            build_type: BuildType::None,
        }
    }
}

impl Bvh {
    pub fn nodes(&self) -> &[BvhNode] {
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

    pub fn refit(&mut self, new_aabbs: &[Aabb]) {
        for i in (0..self.nodes.len()).rev() {
            // Create new aabb
            let mut aabb = Aabb::new();

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

                // Offset by a small epsilon to make sure the bounds encapsulate all of the geometry
                aabb.offset_by(0.0001);
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
        BvhNode::traverse(
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
        BvhNode::traverse_t(
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
        BvhNode::occludes(
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
        BvhNode::depth_test(
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
        BvhNode::traverse4(
            self.nodes.as_slice(),
            self.prim_indices.as_slice(),
            packet,
            intersection_test,
        );
    }

    fn traverse_check(&self, cur_node: &BvhNode, checks: &mut [u8]) {
        if let Some(left_first) = cur_node.get_left_first() {
            if cur_node.is_leaf() {
                for i in 0..cur_node.get_count_unchecked() {
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
    pub fn validate(&self, prim_count: usize) -> bool {
        let mut found_indices = vec![0_u8; prim_count];
        self.traverse_check(&self.nodes[0], found_indices.as_mut());

        let mut valid = true;
        for (i, index) in found_indices.iter().copied().enumerate() {
            if index == 0 {
                eprintln!("prim ({}) not referenced", i);
                valid = false;
            }
        }
        valid
    }

    pub fn into_raw_indices(self) -> Vec<u32> {
        self.prim_indices
    }

    pub fn into_raw_nodes(self) -> Vec<BvhNode> {
        self.nodes
    }

    pub fn into_raw(self) -> (Vec<BvhNode>, Vec<u32>) {
        (self.nodes, self.prim_indices)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Mbvh {
    pub(crate) nodes: Vec<BvhNode>,
    pub(crate) m_nodes: Vec<MbvhNode>,
    pub(crate) prim_indices: Vec<u32>,
}

impl Default for Mbvh {
    fn default() -> Self {
        Self {
            nodes: Default::default(),
            m_nodes: Default::default(),
            prim_indices: Default::default(),
        }
    }
}

impl Mbvh {
    pub fn nodes(&self) -> &[BvhNode] {
        self.nodes.as_slice()
    }

    pub fn quad_nodes(&self) -> &[MbvhNode] {
        self.m_nodes.as_slice()
    }

    pub fn indices(&self) -> &[u32] {
        self.prim_indices.as_slice()
    }

    pub fn prim_count(&self) -> usize {
        self.prim_indices.len()
    }

    pub fn construct_from_raw(nodes: &[BvhNode], prim_indices: &[u32]) -> Self {
        debug_assert!(!nodes.is_empty());
        let mut m_nodes = vec![MbvhNode::new(); nodes.len()];
        let mut pool_ptr = 1;

        if nodes.len() <= 4 {
            for (i, node) in nodes.iter().enumerate() {
                m_nodes[0].set_bounds_bb(i, &node.bounds);
                m_nodes[0].children[i] = node.get_left_first_unchecked();
                m_nodes[0].counts[i] = node.get_count_unchecked();
            }

            return Mbvh {
                nodes: nodes.to_vec(),
                m_nodes,
                prim_indices: prim_indices.to_vec(),
            };
        }

        MbvhNode::merge_nodes(0, 0, nodes, m_nodes.as_mut_slice(), &mut pool_ptr);

        Mbvh {
            nodes: nodes.to_vec(),
            m_nodes,
            prim_indices: prim_indices.to_vec(),
        }
    }

    pub fn construct(bvh: &Bvh) -> Self {
        debug_assert!(!bvh.nodes.is_empty());
        let mut m_nodes = vec![MbvhNode::new(); bvh.nodes.len()];
        let mut pool_ptr = 1;

        if bvh.nodes.len() <= 4 {
            for (i, node) in bvh.nodes.iter().enumerate() {
                m_nodes[0].set_bounds_bb(i, &node.bounds);
                m_nodes[0].children[i] = node.get_left_first_unchecked();
                m_nodes[0].counts[i] = node.get_count_unchecked();
            }

            return Mbvh {
                nodes: bvh.nodes.clone(),
                m_nodes,
                prim_indices: bvh.prim_indices.clone(),
            };
        }

        MbvhNode::merge_nodes(
            0,
            0,
            bvh.nodes.as_slice(),
            m_nodes.as_mut_slice(),
            &mut pool_ptr,
        );

        Mbvh {
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
        MbvhNode::traverse(
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
        MbvhNode::traverse_t(
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
        MbvhNode::occludes(
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
        MbvhNode::depth_test(
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
        MbvhNode::traverse4(
            self.m_nodes.as_slice(),
            self.prim_indices.as_slice(),
            packet,
            intersection_test,
        );
    }

    pub fn into_raw_indices(self) -> Vec<u32> {
        self.prim_indices
    }

    pub fn into_raw_nodes(self) -> Vec<MbvhNode> {
        self.m_nodes
    }

    pub fn into_raw(self) -> (Vec<MbvhNode>, Vec<u32>) {
        (self.m_nodes, self.prim_indices)
    }
}

impl From<Bvh> for Mbvh {
    fn from(bvh: Bvh) -> Self {
        Mbvh::construct(&bvh)
    }
}

impl Bounds<i32> for Bvh {
    fn bounds(&self) -> Aabb {
        self.nodes[0].bounds
    }
}

impl Bounds<i32> for Mbvh {
    fn bounds(&self) -> Aabb {
        self.nodes[0].bounds
    }
}
