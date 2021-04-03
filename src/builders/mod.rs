use crate::{BvhNode, Bvh};

use crate::utils::UnsafeSliceWrapper;
use std::sync::atomic::AtomicUsize;
use std::sync::atomic::Ordering::SeqCst;

pub mod binned_sah;
pub mod locb;
pub mod spatial_sah;

pub trait BuildAlgorithm {
    fn build(self) -> Bvh;
}

#[derive(Debug)]
pub enum BvhType {
    LocallyOrderedClustered = 0,
    BinnedSAH = 1,
}

impl From<u32> for BvhType {
    fn from(i: u32) -> Self {
        match i {
            0 => BvhType::LocallyOrderedClustered,
            1 => BvhType::BinnedSAH,
            _ => BvhType::BinnedSAH,
        }
    }
}

/// Stack for acquiring mutable node references across multiple threads
#[derive(Debug, Clone)]
struct AtomicNodeStack<'a> {
    counter: &'a AtomicUsize,
    nodes: UnsafeSliceWrapper<'a, BvhNode>,
}

#[derive(Debug)]
struct AllocatedNodes<'a> {
    pub left: usize,
    pub right: usize,
    pub left_node: &'a mut BvhNode,
    pub right_node: &'a mut BvhNode,
}

#[allow(dead_code)]
impl<'a> AtomicNodeStack<'a> {
    pub fn new(counter: &'a AtomicUsize, nodes: &'a mut [BvhNode]) -> (Self, &'a mut BvhNode) {
        counter.store(1, SeqCst);
        let nodes = UnsafeSliceWrapper::new(nodes);
        let first_node = nodes.get_mut(0).unwrap();
        (Self { counter, nodes }, first_node)
    }

    pub fn allocate(&self) -> Option<AllocatedNodes<'a>> {
        let left = self.counter.fetch_add(2, SeqCst);
        let right = left + 1;

        let left_node = self.nodes.get_mut(left)?;
        let right_node = self.nodes.get_mut(right)?;

        Some(AllocatedNodes {
            left,
            right,
            left_node,
            right_node,
        })
    }

    pub fn get(&self, idx: usize) -> Option<&'a BvhNode> {
        self.nodes.get(idx)
    }

    pub fn get_mut(&self, idx: usize) -> Option<&'a mut BvhNode> {
        self.nodes.get_mut(idx)
    }
}
