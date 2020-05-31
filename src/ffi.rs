use crate::aabb::AABB;
use crate::builders::{spatial_sah::SpatialTriangle, BVHType};
use crate::bvh;
use crate::{bvh_node::BVHNode, MBVHNode, BVH, MBVH};
use glam::*;

#[repr(C)]
pub struct RTBVH {
    prim_count: u32,

    node_count: u32,
    node_capacity: u32,
    nodes: *const BVHNode,

    index_count: u32,
    indices_capacity: u32,
    indices: *const u32,
}

#[repr(C)]
pub struct RTMBVH {
    prim_count: u32,

    node_count: u32,
    node_capacity: u32,
    nodes: *const MBVHNode,

    index_count: u32,
    indices_capacity: u32,
    indices: *const u32,
}

struct RTTriangleWrapper {
    vertices: *const f32,
    vertex0_offset: u32,
    vertex_stride: u32,
}

unsafe impl Send for RTTriangleWrapper {}
unsafe impl Sync for RTTriangleWrapper {}

impl SpatialTriangle for RTTriangleWrapper {
    fn vertex0(&self) -> Vec3 {
        unsafe {
            let ptr = self.vertices.add(self.vertex0_offset as usize);
            let x = *ptr.as_ref().unwrap();
            let y = *ptr.add(1).as_ref().unwrap();
            let z = *ptr.add(2).as_ref().unwrap();
            Vec3::new(x, y, z)
        }
    }

    fn vertex1(&self) -> Vec3 {
        unsafe {
            let ptr = self
                .vertices
                .add((self.vertex0_offset + self.vertex_stride) as usize);
            let x = *ptr.as_ref().unwrap();
            let y = *ptr.add(1).as_ref().unwrap();
            let z = *ptr.add(2).as_ref().unwrap();
            Vec3::new(x, y, z)
        }
    }

    fn vertex2(&self) -> Vec3 {
        unsafe {
            let ptr = self
                .vertices
                .add((self.vertex0_offset + 2 * self.vertex_stride) as usize);
            let x = *ptr.as_ref().unwrap();
            let y = *ptr.add(1).as_ref().unwrap();
            let z = *ptr.add(2).as_ref().unwrap();
            Vec3::new(x, y, z)
        }
    }
}

#[no_mangle]
pub extern "C" fn create_spatial_bvh(
    aabbs: *const AABB,
    prim_count: usize,
    centers: *const f32,
    stride: usize,
    vertices: *const f32,
    vertex_stride: usize,
    triangle_stride: usize,
) -> *const RTBVH {
    assert_eq!(stride % 4, 0);

    let aabbs = unsafe { std::slice::from_raw_parts(aabbs, prim_count) };
    let triangles: Vec<RTTriangleWrapper> = (0..prim_count)
        .into_iter()
        .map(|i| RTTriangleWrapper {
            vertices,
            vertex0_offset: (i * triangle_stride / 4) as u32 / 4,
            vertex_stride: (vertex_stride / 4) as u32 / 4,
        })
        .collect();

    let bvh = if stride == 16 {
        let centers: &[Vec3] =
            unsafe { std::slice::from_raw_parts(centers as *const Vec3, prim_count) };
        bvh::BVH::construct_spatial(aabbs, centers, triangles.as_slice())
    } else {
        let data = unsafe { std::slice::from_raw_parts(centers, prim_count * stride / 4) };
        let offset = stride / 4;
        let centers: Vec<Vec3> = (0..prim_count)
            .into_iter()
            .map(|i| {
                let ix = i * offset;
                let iy = ix + 1;
                let iz = ix + 2;

                let x = data[ix];
                let y = data[iy];
                let z = data[iz];

                Vec3::new(x, y, z)
            })
            .collect();
        bvh::BVH::construct_spatial(aabbs, centers.as_slice(), triangles.as_slice())
    };

    let (nodes_vec, indices_vec) = bvh.as_raw();

    let prim_count = prim_count as u32;

    let node_count = nodes_vec.len() as u32;
    let node_capacity = nodes_vec.capacity() as u32;
    let nodes = nodes_vec.as_ptr();

    let indices_capacity = indices_vec.capacity() as u32;
    let index_count = indices_vec.len() as u32;
    let indices = indices_vec.as_ptr();

    Vec::leak(nodes_vec);
    Vec::leak(indices_vec);

    Box::leak(Box::new(RTBVH {
        prim_count,
        node_count,
        node_capacity,
        nodes,
        indices,
        indices_capacity,
        index_count,
    })) as *const RTBVH
}

#[no_mangle]
pub extern "C" fn create_bvh(
    aabbs: *const AABB,
    prim_count: usize,
    centers: *const f32,
    center_stride: usize,
    bvh_type: BVHType,
) -> *const RTBVH {
    assert_eq!(center_stride % 4, 0);

    let aabbs = unsafe { std::slice::from_raw_parts(aabbs, prim_count) };

    let bvh = if center_stride == 16 {
        let centers: &[Vec3] =
            unsafe { std::slice::from_raw_parts(centers as *const Vec3, prim_count) };
        bvh::BVH::construct(aabbs, centers, bvh_type)
    } else {
        let data = unsafe { std::slice::from_raw_parts(centers, prim_count * center_stride / 4) };
        let offset = center_stride / 4;
        let centers: Vec<Vec3> = (0..prim_count)
            .into_iter()
            .map(|i| {
                let ix = i * offset;
                let iy = ix + 1;
                let iz = ix + 2;

                let x = data[ix];
                let y = data[iy];
                let z = data[iz];

                Vec3::new(x, y, z)
            })
            .collect();
        bvh::BVH::construct(aabbs, centers.as_slice(), bvh_type)
    };

    let (nodes_vec, indices_vec) = bvh.as_raw();

    let prim_count = prim_count as u32;

    let node_count = nodes_vec.len() as u32;
    let node_capacity = nodes_vec.capacity() as u32;
    let nodes = nodes_vec.as_ptr();

    let indices_capacity = indices_vec.capacity() as u32;
    let index_count = indices_vec.len() as u32;
    let indices = indices_vec.as_ptr();

    Vec::leak(nodes_vec);
    Vec::leak(indices_vec);

    Box::leak(Box::new(RTBVH {
        prim_count,
        node_count,
        node_capacity,
        nodes,
        indices,
        indices_capacity,
        index_count,
    })) as *const RTBVH
}

#[no_mangle]
pub extern "C" fn create_mbvh(bvh: *const RTBVH) -> *const RTMBVH {
    let bvh = unsafe { Box::from_raw(bvh as *mut RTBVH) };

    let nodes = unsafe {
        Vec::from_raw_parts(
            bvh.nodes as *mut BVHNode,
            bvh.node_count as usize,
            bvh.node_capacity as usize,
        )
    };

    let indices = unsafe {
        Vec::from_raw_parts(
            bvh.indices as *mut u32,
            bvh.index_count as usize,
            bvh.indices_capacity as usize,
        )
    };

    let mbvh = MBVH::construct_from_raw(nodes.as_slice(), indices.as_slice());

    let (nodes_vec, indices_vec) = mbvh.as_raw();

    let prim_count = bvh.prim_count;

    let node_count = nodes_vec.len() as u32;
    let node_capacity = nodes_vec.capacity() as u32;
    let nodes = nodes_vec.as_ptr();

    let indices_capacity = indices_vec.capacity() as u32;
    let index_count = indices_vec.len() as u32;
    let indices = indices_vec.as_ptr();

    Vec::leak(nodes_vec);
    Vec::leak(indices_vec);

    Box::leak(bvh);
    Box::leak(Box::new(RTMBVH {
        prim_count,
        node_count,
        node_capacity,
        nodes,
        indices,
        indices_capacity,
        index_count,
    })) as *const RTMBVH
}

#[no_mangle]
pub extern "C" fn refit(aabbs: *const AABB, bvh: *const RTBVH) {
    // Retrieve structure in Rust type
    let bvh = unsafe { Box::from_raw(bvh as *mut RTBVH) };

    // Retrieve Vec
    let nodes = unsafe {
        Vec::from_raw_parts(
            bvh.nodes as *mut BVHNode,
            bvh.node_count as usize,
            bvh.node_capacity as usize,
        )
    };
    let indices = unsafe {
        Vec::from_raw_parts(
            bvh.indices as *mut u32,
            bvh.index_count as usize,
            bvh.indices_capacity as usize,
        )
    };

    // Create Rust structure
    let mut instance = BVH {
        nodes,
        prim_indices: indices,
    };

    // Refit BVH
    instance.refit(unsafe { std::slice::from_raw_parts(aabbs, bvh.prim_count as usize) });

    let (nodes, indices) = instance.as_raw();

    // Make sure data does not get freed until user calls free_bvh
    Vec::leak(nodes);
    Vec::leak(indices);
    Box::leak(bvh);
}

#[no_mangle]
pub extern "C" fn free_bvh(bvh: *const RTBVH) {
    unsafe {
        let bvh = Box::from_raw(bvh as *mut RTBVH);
        let _ = Vec::from_raw_parts(
            bvh.nodes as *mut BVHNode,
            bvh.node_count as usize,
            bvh.node_capacity as usize,
        );

        let _ = Vec::from_raw_parts(
            bvh.indices as *mut u32,
            bvh.index_count as usize,
            bvh.indices_capacity as usize,
        );
    }
}

#[no_mangle]
pub extern "C" fn free_mbvh(bvh: *const RTMBVH) {
    unsafe {
        let bvh = Box::from_raw(bvh as *mut RTMBVH);

        let _ = Vec::from_raw_parts(
            bvh.nodes as *mut MBVHNode,
            bvh.node_count as usize,
            bvh.node_capacity as usize,
        );

        let _ = Vec::from_raw_parts(
            bvh.indices as *mut u32,
            bvh.index_count as usize,
            bvh.indices_capacity as usize,
        );
    }
}
