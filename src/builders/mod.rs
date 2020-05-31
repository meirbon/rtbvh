use crate::{BVHNode, BVH};

use crate::utils::UnsafeSliceWrapper;
use std::sync::atomic::AtomicUsize;
use std::sync::atomic::Ordering::SeqCst;

pub mod binned_sah;
pub mod locb;
pub mod spatial_sah;

pub trait Builder {
    fn build(self) -> BVH;
}

#[derive(Debug)]
pub enum BVHType {
    LocallyOrderedClustered,
    BinnedSAH,
}

/// Stack for acquiring mutable node references across multiple threads
#[derive(Debug, Clone)]
struct AtomicNodeStack<'a> {
    counter: &'a AtomicUsize,
    nodes: UnsafeSliceWrapper<'a, BVHNode>,
}

#[derive(Debug)]
struct AllocatedNodes<'a> {
    pub left: usize,
    pub right: usize,
    pub left_node: &'a mut BVHNode,
    pub right_node: &'a mut BVHNode,
}

#[allow(dead_code)]
impl<'a> AtomicNodeStack<'a> {
    pub fn new(counter: &'a AtomicUsize, nodes: &'a mut [BVHNode]) -> (Self, &'a mut BVHNode) {
        counter.store(1, SeqCst);
        let nodes = UnsafeSliceWrapper::new(nodes);
        let first_node = nodes.get_mut(0).unwrap();
        (Self { counter, nodes }, first_node)
    }

    pub fn allocate(&self) -> Option<AllocatedNodes<'a>> {
        let left = self.counter.fetch_add(2, SeqCst);
        let right = left + 1;

        let left_node = self.nodes.get_mut(left);
        let right_node = self.nodes.get_mut(right);

        if left_node.is_none() || right_node.is_none() {
            None
        } else {
            let (left_node, right_node) = (left_node.unwrap(), right_node.unwrap());
            Some(AllocatedNodes {
                left,
                right,
                left_node,
                right_node,
            })
        }
    }

    pub fn get(&self, idx: usize) -> Option<&'a BVHNode> {
        self.nodes.get(idx)
    }

    pub fn get_mut(&self, idx: usize) -> Option<&'a mut BVHNode> {
        self.nodes.get_mut(idx)
    }
}
