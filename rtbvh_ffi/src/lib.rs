use glam::*;
use rtbvh::aabb::AABB;
use rtbvh::builders::spatial_sah::SpatialTriangle;
use rtbvh::bvh;
use rtbvh::{bvh_node::BVHNode, BVH, MBVH};
use std::sync::Mutex;

static mut MANAGER: Option<StructureManager> = None;

struct StructureManager {
    structures: Mutex<Vec<BVH>>,
    m_structures: Mutex<Vec<MBVH>>,
}

impl Default for StructureManager {
    fn default() -> Self {
        StructureManager {
            structures: Mutex::new(Vec::new()),
            m_structures: Mutex::new(Vec::new()),
        }
    }
}

impl StructureManager {
    pub fn store(&mut self, bvh: BVH) -> RTBVH {
        let mut lock = self.structures.lock().unwrap();
        lock.push(bvh);
        let id = lock.len() - 1;

        let bvh: &BVH = &lock[id];

        RTBVH {
            id: id as u32,
            node_count: bvh.nodes.len() as u32,
            nodes: bvh.nodes.as_ptr() as *const RTBVHNode,
            index_count: bvh.prim_indices.len() as u32,
            indices: bvh.prim_indices.as_ptr(),
        }
    }

    pub fn store_mbvh(&mut self, mbvh: MBVH) -> RTMBVH {
        let mut lock = self.m_structures.lock().unwrap();

        lock.push(mbvh);
        let id = lock.len() - 1;

        let mbvh: &MBVH = &lock[id];

        RTMBVH {
            id: id as u32,
            node_count: mbvh.m_nodes.len() as u32,
            nodes: mbvh.m_nodes.as_ptr() as *const RTMBVHNode,
            index_count: mbvh.prim_indices.len() as u32,
            indices: mbvh.prim_indices.as_ptr(),
        }
    }

    pub fn get<T, B>(&self, id: u32, mut cb: T) -> Option<B>
    where
        T: FnMut(Option<&BVH>) -> B,
    {
        if let Ok(lock) = self.structures.lock() {
            Some(cb(lock.get(id as usize)))
        } else {
            None
        }
    }

    pub fn get_mut<T, B>(&self, id: u32, mut cb: T) -> Option<B>
    where
        T: FnMut(Option<&mut BVH>) -> B,
    {
        if let Ok(mut lock) = self.structures.lock() {
            Some(cb(lock.get_mut(id as usize)))
        } else {
            None
        }
    }

    pub fn free(&mut self, id: u32) -> Result<(), ()> {
        let mut lock = self.structures.lock().unwrap();
        if let Some(_) = lock.get(id as usize) {
            lock[id as usize] = BVH::empty();
            Ok(())
        } else {
            Err(())
        }
    }

    pub fn free_mbvh(&mut self, id: u32) -> Result<(), ()> {
        let mut lock = self.m_structures.lock().unwrap();
        if let Some(_) = lock.get(id as usize) {
            lock[id as usize] = MBVH::empty();
            Ok(())
        } else {
            Err(())
        }
    }
}

#[repr(u32)]
pub enum BVHType {
    LocallyOrderedClustered = 0,
    BinnedSAH = 1,
}

impl Into<rtbvh::builders::BVHType> for BVHType {
    fn into(self) -> rtbvh::builders::BVHType {
        match self {
            BVHType::LocallyOrderedClustered => rtbvh::builders::BVHType::LocallyOrderedClustered,
            BVHType::BinnedSAH => rtbvh::builders::BVHType::BinnedSAH,
            // _ => rtbvh::builders::BVHType::BinnedSAH,
        }
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct RTAABB {
    pub min: [f32; 3],
    pub max: [f32; 3],
}

impl From<AABB> for RTAABB {
    fn from(bb: AABB) -> RTAABB {
        RTAABB {
            min: bb.min,
            max: bb.max,
        }
    }
}

impl Into<AABB> for RTAABB {
    fn into(self) -> AABB {
        AABB {
            min: self.min,
            max: self.max,
        }
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct RTBVHNode {
    pub aabb: RTAABB,
    pub left_first: i32,
    pub count: i32,
}

impl Into<BVHNode> for RTBVHNode {
    fn into(self) -> BVHNode {
        BVHNode {
            bounds: self.aabb.into(),
            left_first: self.left_first,
            count: self.count,
        }
    }
}

impl From<BVHNode> for RTBVHNode {
    fn from(node: BVHNode) -> RTBVHNode {
        RTBVHNode {
            aabb: node.bounds.into(),
            left_first: node.left_first,
            count: node.count,
        }
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct RTMBVHNode {
    min_x: [f32; 4],
    max_x: [f32; 4],
    min_y: [f32; 4],
    max_y: [f32; 4],
    min_z: [f32; 4],
    max_z: [f32; 4],
    pub children: [i32; 4],
    pub counts: [i32; 4],
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct RTBVH {
    id: u32,

    node_count: u32,
    nodes: *const RTBVHNode,

    index_count: u32,
    indices: *const u32,
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct RTMBVH {
    id: u32,

    node_count: u32,
    nodes: *const RTMBVHNode,

    index_count: u32,
    indices: *const u32,
}

#[derive(Debug)]
struct RTTriangleWrapper {
    vertices: *const f32,
    vertex0_offset: u32,
    vertex_stride: u32,
}

unsafe impl Send for RTTriangleWrapper {}
unsafe impl Sync for RTTriangleWrapper {}

impl SpatialTriangle for RTTriangleWrapper {
    fn vertex0(&self) -> [f32; 3] {
        unsafe {
            let ptr = self.vertices.add(self.vertex0_offset as usize);
            let x = *ptr.as_ref().unwrap();
            let y = *ptr.add(1).as_ref().unwrap();
            let z = *ptr.add(2).as_ref().unwrap();
            [x, y, z]
        }
    }

    fn vertex1(&self) -> [f32; 3] {
        unsafe {
            let ptr = self
                .vertices
                .add((self.vertex0_offset + self.vertex_stride) as usize);
            let x = *ptr.as_ref().unwrap();
            let y = *ptr.add(1).as_ref().unwrap();
            let z = *ptr.add(2).as_ref().unwrap();
            [x, y, z]
        }
    }

    fn vertex2(&self) -> [f32; 3] {
        unsafe {
            let ptr = self
                .vertices
                .add((self.vertex0_offset + 2 * self.vertex_stride) as usize);
            let x = *ptr.as_ref().unwrap();
            let y = *ptr.add(1).as_ref().unwrap();
            let z = *ptr.add(2).as_ref().unwrap();
            [x, y, z]
        }
    }
}

#[no_mangle]
pub extern "C" fn create_spatial_bvh(
    aabbs: *const RTAABB,
    prim_count: usize,
    centers: *const f32,
    stride: usize,
    vertices: *const f32,
    vertex_stride: usize,
    triangle_stride: usize,
) -> RTBVH {
    assert_eq!(stride % 4, 0);
    unsafe {
        if MANAGER.is_none() {
            MANAGER = Some(StructureManager {
                structures: Mutex::new(Vec::new()),
                m_structures: Mutex::new(Vec::new()),
            });
        }
    }

    let aabbs = unsafe { std::slice::from_raw_parts(aabbs as *const AABB, prim_count) };
    let triangles: Vec<RTTriangleWrapper> = (0..prim_count)
        .into_iter()
        .map(|i| RTTriangleWrapper {
            vertices,
            vertex0_offset: (i * triangle_stride / 4) as u32,
            vertex_stride: (vertex_stride / 4) as u32,
        })
        .collect();

    let bvh = if stride == 16 {
        let centers: &[Vec3A] =
            unsafe { std::slice::from_raw_parts(centers as *const Vec3A, prim_count) };
        bvh::BVH::construct_spatial(aabbs, centers, triangles.as_slice())
    } else {
        let data = unsafe { std::slice::from_raw_parts(centers, prim_count * stride / 4) };
        let offset = stride / 4;
        let centers: Vec<Vec3A> = (0..prim_count)
            .into_iter()
            .map(|i| {
                let ix = i * offset;
                let iy = ix + 1;
                let iz = ix + 2;

                let x = data[ix];
                let y = data[iy];
                let z = data[iz];

                Vec3A::new(x, y, z)
            })
            .collect();
        bvh::BVH::construct_spatial(aabbs, centers.as_slice(), triangles.as_slice())
    };

    unsafe { MANAGER.as_mut().unwrap().store(bvh) }
}

#[no_mangle]
pub extern "C" fn create_bvh(
    aabbs: *const RTAABB,
    prim_count: usize,
    centers: *const f32,
    center_stride: usize,
    bvh_type: BVHType,
) -> RTBVH {
    assert_eq!(center_stride % 4, 0);
    unsafe {
        if MANAGER.is_none() {
            MANAGER = Some(StructureManager {
                structures: Mutex::new(Vec::new()),
                m_structures: Mutex::new(Vec::new()),
            });
        }
    }

    let aabbs = unsafe { std::slice::from_raw_parts(aabbs as *const AABB, prim_count) };

    let bvh = if center_stride == 16 {
        let centers: &[Vec3A] =
            unsafe { std::slice::from_raw_parts(centers as *const Vec3A, prim_count) };
        bvh::BVH::construct(aabbs, centers, bvh_type.into())
    } else {
        let data = unsafe { std::slice::from_raw_parts(centers, prim_count * center_stride / 4) };
        let offset = center_stride / 4;
        let centers: Vec<Vec3A> = (0..prim_count)
            .into_iter()
            .map(|i| {
                let ix = i * offset;
                let iy = ix + 1;
                let iz = ix + 2;

                let x = data[ix];
                let y = data[iy];
                let z = data[iz];

                Vec3A::new(x, y, z)
            })
            .collect();
        bvh::BVH::construct(aabbs, centers.as_slice(), bvh_type.into())
    };

    unsafe { MANAGER.as_mut().unwrap().store(bvh) }
}

#[no_mangle]
pub extern "C" fn create_mbvh(bvh: RTBVH) -> RTMBVH {
    unsafe {
        MANAGER
            .as_mut()
            .unwrap()
            .get(bvh.id, |bvh| {
                let mbvh = MBVH::construct(bvh.unwrap());
                MANAGER.as_mut().unwrap().store_mbvh(mbvh)
            })
            .unwrap()
    }
}

#[no_mangle]
pub extern "C" fn refit(aabbs: *const RTAABB, bvh: RTBVH) {
    unsafe {
        MANAGER
            .as_mut()
            .unwrap()
            .get_mut(bvh.id, |bvh| {
                let bvh = bvh.unwrap();
                bvh.refit(std::slice::from_raw_parts(
                    aabbs as *const AABB,
                    bvh.prim_count(),
                ));
            })
            .unwrap();
    }
}

#[no_mangle]
pub extern "C" fn free_bvh(bvh: RTBVH) {
    unsafe {
        if let Err(_) = MANAGER.as_mut().unwrap().free(bvh.id) {
            println!("Could not free bvh with id: {}", bvh.id);
        }
    }
}

#[no_mangle]
pub extern "C" fn free_mbvh(bvh: RTMBVH) {
    unsafe {
        if let Err(_) = MANAGER.as_mut().unwrap().free_mbvh(bvh.id) {
            println!("Could not free bvh with id: {}", bvh.id);
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::*;
    use glam::*;
    use rtbvh::*;

    #[test]
    fn same_size() {
        assert_eq!(
            std::mem::size_of::<BVHNode>(),
            std::mem::size_of::<RTBVHNode>()
        );
        assert_eq!(
            std::mem::size_of::<MBVHNode>(),
            std::mem::size_of::<RTMBVHNode>()
        );
        assert_eq!(std::mem::size_of::<AABB>(), std::mem::size_of::<RTAABB>());
    }

    #[test]
    fn create_delete() {
        let mut vertices: Vec<Vec3A> = Vec::with_capacity(81);

        // 27 Triangles
        for x in 0..=9 {
            for y in 0..=9 {
                vertices.push(Vec3A::new(x as f32, y as f32, 0 as f32));
            }
        }

        let aabbs: Vec<AABB> = (0..27)
            .map(|i| {
                let v0: Vec3A = vertices[i * 3 + 0];
                let v1: Vec3A = vertices[i * 3 + 1];
                let v2: Vec3A = vertices[i * 3 + 2];
                aabb!(v0, v1, v2)
            })
            .collect();

        let centers: Vec<Vec3A> = aabbs.iter().map(|bb| bb.center()).collect();

        let bvh = create_bvh(
            aabbs.as_ptr() as *const RTAABB,
            27,
            centers.as_ptr() as *const f32,
            16,
            BVHType::BinnedSAH,
        );

        let mbvh = create_mbvh(bvh);

        free_bvh(bvh);
        free_mbvh(mbvh);
    }
}
