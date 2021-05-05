use crate::*;
use glam::*;
use rayon::prelude::*;
use std::fmt::{Debug, Formatter};
use std::num::NonZeroUsize;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub trait Primitive<AabbType: Debug + Copy + Send + Sync = i32>: Debug + Send + Sync {
    fn center(&self) -> Vec3;

    fn aabb(&self) -> Aabb<AabbType>;
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum BuildType {
    None,
    LocallyOrderedClustered,
    BinnedSAH,
    Spatial,
}

#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum BuildError {
    NoPrimitives,
    InequalAabbsAndPrimitives(usize, usize),
}

impl std::fmt::Display for BuildError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                BuildError::NoPrimitives => "No primitives supplied".to_owned(),
                BuildError::InequalAabbsAndPrimitives(bbs, prims) => {
                    format!("#Aabbs({}) != #Primitives({})", *bbs, *prims)
                }
            }
        )
    }
}

impl std::error::Error for BuildError {}

#[derive(Debug, Copy, Clone)]
pub struct Builder<'a, T: Primitive<i32>> {
    /// Reference to aabbs
    pub aabbs: Option<&'a [Aabb<i32>]>,
    /// Reference to primitives
    pub primitives: &'a [T],
    /// Max number of primitives that can be stored in a leaf node
    pub primitives_per_leaf: Option<NonZeroUsize>,
}

impl<'a, T: Primitive<i32>> Builder<'a, T> {
    pub fn construct_spatial_sah(self) -> Result<Bvh, BuildError>
    where
        T: SpatialTriangle,
    {
        if self.primitives.is_empty() {
            return Err(BuildError::NoPrimitives);
        }

        if let Some(aabbs) = self.aabbs {
            if aabbs.len() != self.primitives.len() {
                return Err(BuildError::InequalAabbsAndPrimitives(
                    aabbs.len(),
                    self.primitives.len(),
                ));
            }

            Ok(SpatialSahBuilder::new(aabbs, self.primitives, self.primitives_per_leaf).build())
        } else {
            let aabbs = self
                .primitives
                .iter()
                .par_bridge()
                .map(|p| p.aabb())
                .collect::<Vec<_>>();
            Ok(SpatialSahBuilder::new(&aabbs, self.primitives, self.primitives_per_leaf).build())
        }
    }

    pub fn construct_binned_sah(self) -> Result<Bvh, BuildError> {
        if self.primitives.is_empty() {
            return Err(BuildError::NoPrimitives);
        }

        if let Some(aabbs) = self.aabbs {
            if aabbs.len() != self.primitives.len() {
                return Err(BuildError::InequalAabbsAndPrimitives(
                    aabbs.len(),
                    self.primitives.len(),
                ));
            }

            Ok(BinnedSahBuilder::new(&aabbs, self.primitives, self.primitives_per_leaf).build())
        } else {
            let aabbs = self
                .primitives
                .iter()
                .par_bridge()
                .map(|p| p.aabb())
                .collect::<Vec<_>>();

            Ok(BinnedSahBuilder::new(&aabbs, self.primitives, self.primitives_per_leaf).build())
        }
    }

    pub fn construct_locally_ordered_clustered(self) -> Result<Bvh, BuildError> {
        if self.primitives.is_empty() {
            return Err(BuildError::NoPrimitives);
        }

        if let Some(aabbs) = self.aabbs {
            if aabbs.len() != self.primitives.len() {
                return Err(BuildError::InequalAabbsAndPrimitives(
                    aabbs.len(),
                    self.primitives.len(),
                ));
            }

            Ok(LocallyOrderedClusteringBuilder::new(aabbs, self.primitives).build())
        } else {
            let aabbs = self
                .primitives
                .iter()
                .par_bridge()
                .map(|p| p.aabb())
                .collect::<Vec<_>>();

            Ok(LocallyOrderedClusteringBuilder::new(&aabbs, self.primitives).build())
        }
    }
}

// A BVH structure with nodes and primitive indices
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Clone)]
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

    pub fn traverse_iter<'a, 'b, T: 'a + Primitive>(
        &'a self,
        ray: &'b mut Ray,
        primitives: &'a [T],
    ) -> BvhIterator<'a, 'b, T> {
        BvhIterator::new(ray, self, primitives)
    }

    pub fn traverse_iter_packet<'a, 'b, T: 'a + Primitive>(
        &'a self,
        ray: &'b mut RayPacket4,
        primitives: &'a [T],
    ) -> BvhPacketIterator<'a, 'b, T> {
        BvhPacketIterator::new(ray, self, primitives)
    }
}

impl<'a, 'b, T: 'a + Primitive> IntoRayIterator<'a, 'b, T> for Bvh {
    type RIterator = BvhIterator<'a, 'b, T>;

    fn iter(&'a self, ray: &'b mut Ray, primitives: &'a [T]) -> Self::RIterator {
        BvhIterator::new(ray, self, primitives)
    }
}

impl<'a, 'b, T: 'a + Primitive> IntoPacketIterator<'a, 'b, T> for Bvh {
    type RIterator = BvhPacketIterator<'a, 'b, T>;

    fn iter(&'a self, packet: &'b mut RayPacket4, primitives: &'a [T]) -> Self::RIterator {
        BvhPacketIterator::new(packet, self, primitives)
    }
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Clone)]
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

    pub fn into_raw_indices(self) -> Vec<u32> {
        self.prim_indices
    }

    pub fn into_raw_nodes(self) -> Vec<MbvhNode> {
        self.m_nodes
    }

    pub fn into_raw(self) -> (Vec<MbvhNode>, Vec<u32>) {
        (self.m_nodes, self.prim_indices)
    }

    pub fn traverse_iter<'a, 'b, T: Primitive>(
        &'a self,
        ray: &'b mut Ray,
        primitives: &'a [T],
    ) -> MbvhIterator<'a, 'b, T> {
        MbvhIterator::new(ray, self, primitives)
    }

    pub fn traverse_iter_packet<'a, 'b, T: Primitive>(
        &'a self,
        ray: &'b mut RayPacket4,
        primitives: &'a [T],
    ) -> MbvhPacketIterator<'a, 'b, T> {
        MbvhPacketIterator::new(ray, self, primitives)
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

impl<'a, 'b, T: 'a + Primitive> IntoRayIterator<'a, 'b, T> for Mbvh {
    type RIterator = MbvhIterator<'a, 'b, T>;

    fn iter(&'a self, ray: &'b mut Ray, primitives: &'a [T]) -> Self::RIterator {
        MbvhIterator::new(ray, self, primitives)
    }
}

impl<'a, 'b, T: 'a + Primitive> IntoPacketIterator<'a, 'b, T> for Mbvh {
    type RIterator = MbvhPacketIterator<'a, 'b, T>;

    fn iter(&'a self, packet: &'b mut RayPacket4, primitives: &'a [T]) -> Self::RIterator {
        MbvhPacketIterator::new(packet, self, primitives)
    }
}
