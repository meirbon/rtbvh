use glam::*;
use lazy_static::lazy_static;
use rtbvh::{spatial_sah::SpatialTriangle, *};
use std::sync::Mutex;

lazy_static! {
    static ref MANAGER: StructureManager = StructureManager::default();
}

struct StructureManager {
    structures: Mutex<Vec<Bvh>>,
    m_structures: Mutex<Vec<Mbvh>>,
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
    pub fn store(&self, bvh: Bvh) -> RTBvh {
        let mut lock = self.structures.lock().unwrap();
        lock.push(bvh);
        let id = lock.len() - 1;

        let bvh: &Bvh = &lock[id];

        RTBvh {
            id: id as u32,
            node_count: bvh.nodes().len() as u32,
            nodes: bvh.nodes().as_ptr() as *const RTBvhNode,
            index_count: bvh.indices().len() as u32,
            indices: bvh.indices().as_ptr(),
        }
    }

    pub fn store_mbvh(&self, mbvh: Mbvh) -> RTMbvh {
        let mut lock = self.m_structures.lock().unwrap();

        lock.push(mbvh);
        let id = lock.len() - 1;

        let mbvh: &Mbvh = &lock[id];

        RTMbvh {
            id: id as u32,
            node_count: mbvh.quad_nodes().len() as u32,
            nodes: mbvh.quad_nodes().as_ptr() as *const RTMbvhNode,
            index_count: mbvh.indices().len() as u32,
            indices: mbvh.indices().as_ptr(),
        }
    }

    pub fn get<T, B>(&self, id: u32, mut cb: T) -> Option<B>
    where
        T: FnMut(Option<&Bvh>) -> B,
    {
        if let Ok(lock) = self.structures.lock() {
            Some(cb(lock.get(id as usize)))
        } else {
            None
        }
    }

    pub fn get_mut<T, B>(&self, id: u32, mut cb: T) -> Option<B>
    where
        T: FnMut(Option<&mut Bvh>) -> B,
    {
        if let Ok(mut lock) = self.structures.lock() {
            Some(cb(lock.get_mut(id as usize)))
        } else {
            None
        }
    }

    pub fn free(&self, id: u32) -> Result<(), ()> {
        let mut lock = self.structures.lock().unwrap();
        if lock.get(id as usize).is_some() {
            lock[id as usize] = Bvh::default();
            Ok(())
        } else {
            Err(())
        }
    }

    pub fn free_mbvh(&self, id: u32) -> Result<(), ()> {
        let mut lock = self.m_structures.lock().unwrap();
        if lock.get(id as usize).is_some() {
            lock[id as usize] = Mbvh::default();
            Ok(())
        } else {
            Err(())
        }
    }
}

#[repr(u32)]
pub enum BvhType {
    LocallyOrderedClustered = 0,
    BinnedSAH = 1,
}

impl From<BvhType> for rtbvh::BvhType {
    fn from(this: BvhType) -> Self {
        match this {
            BvhType::LocallyOrderedClustered => rtbvh::BvhType::LocallyOrderedClustered,
            BvhType::BinnedSAH => rtbvh::BvhType::BinnedSAH,
        }
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct RTAabb {
    pub min: [f32; 3],
    pub count: i32,
    pub max: [f32; 3],
    pub left_first: i32,
}

impl From<Aabb> for RTAabb {
    fn from(bb: Aabb) -> RTAabb {
        RTAabb {
            min: bb.min.into(),
            count: bb.extra1,
            max: bb.max.into(),
            left_first: bb.extra2,
        }
    }
}

impl From<RTAabb> for Aabb {
    fn from(bb: RTAabb) -> Self {
        Self {
            min: bb.min.into(),
            extra1: bb.count,
            max: bb.max.into(),
            extra2: bb.left_first,
        }
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct RTBvhNode {
    pub aabb: RTAabb,
}

impl From<RTBvhNode> for BvhNode {
    fn from(n: RTBvhNode) -> Self {
        BvhNode {
            bounds: n.aabb.into(),
        }
    }
}

impl From<BvhNode> for RTBvhNode {
    fn from(node: BvhNode) -> RTBvhNode {
        RTBvhNode {
            aabb: node.bounds.into(),
        }
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct RTMbvhNode {
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
pub struct RTBvh {
    id: u32,

    node_count: u32,
    nodes: *const RTBvhNode,

    index_count: u32,
    indices: *const u32,
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct RTMbvh {
    id: u32,

    node_count: u32,
    nodes: *const RTMbvhNode,

    index_count: u32,
    indices: *const u32,
}

#[derive(Debug, Copy, Clone)]
struct RTTriangleWrapper {
    centers: *const f32,
    center_stride: u32,
    center_i: u32,
    vertices: *const f32,
    vertex0_offset: u32,
    vertex_stride: u32,
}

impl Primitive for RTTriangleWrapper {
    fn center(&self) -> Vec3 {
        unsafe {
            let ptr = self
                .centers
                .add((self.center_i * self.center_stride / 4) as usize);
            let x = *ptr.as_ref().unwrap();
            let y = *ptr.add(1).as_ref().unwrap();
            let z = *ptr.add(2).as_ref().unwrap();
            vec3(x, y, z)
        }
    }

    fn aabb(&self) -> Aabb {
        let mut aabb = Aabb::empty();
        unsafe {
            let ptr0 = self
                .vertices
                .add((self.vertex0_offset * self.vertex_stride / 4) as usize);
            let ptr1 = ptr0.add((self.vertex_stride / 4) as usize);
            let ptr2 = ptr0.add((self.vertex_stride / 4) as usize * 2);

            aabb.grow(vec3(*ptr0, *ptr0.add(1), *ptr0.add(2)));
            aabb.grow(vec3(*ptr1, *ptr1.add(1), *ptr1.add(2)));
            aabb.grow(vec3(*ptr2, *ptr2.add(1), *ptr2.add(2)));
        }

        aabb
    }
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
            vec3(x, y, z)
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
            vec3(x, y, z)
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
            vec3(x, y, z)
        }
    }
}

#[no_mangle]
pub extern "C" fn create_spatial_Bvh(
    aabbs: *const RTAabb,
    prim_count: usize,
    centers: *const f32,
    stride: usize,
    vertices: *const f32,
    vertex_stride: usize,
    triangle_stride: usize,
    prims_per_leaf: u32,
) -> RTBvh {
    assert_eq!(stride % 4, 0);

    let aabbs = unsafe { std::slice::from_raw_parts(aabbs as *const Aabb, prim_count) };
    let primitives: Vec<RTTriangleWrapper> = (0..prim_count)
        .into_iter()
        .map(|i| RTTriangleWrapper {
            vertices,
            centers,
            center_stride: stride as u32,
            center_i: i as u32,
            vertex0_offset: (i * triangle_stride / 4) as u32,
            vertex_stride: (vertex_stride / 4) as u32,
        })
        .collect();
    let builder = Builder {
        aabbs,
        primitives: primitives.as_slice(),
        primitives_per_leaf: prims_per_leaf as usize,
    };
    let bvh = builder.construct_spatial_sah();

    MANAGER.store(bvh)
}

#[derive(Debug, Copy, Clone)]
struct Vector3 {
    data: [f32; 3],
}

impl Primitive for Vector3 {
    fn center(&self) -> Vec3 {
        Vec3::from(self.data)
    }

    fn aabb(&self) -> Aabb {
        let mut aabb = Aabb::empty();
        aabb.grow(self.data.into());
        aabb
    }
}

#[derive(Debug, Copy, Clone)]
struct Vector4 {
    data: [f32; 4],
}
impl Primitive for Vector4 {
    fn center(&self) -> Vec3 {
        vec3(self.data[0], self.data[1], self.data[2])
    }

    fn aabb(&self) -> Aabb {
        let mut aabb = Aabb::empty();
        aabb.grow(vec3(self.data[0], self.data[1], self.data[2]));
        aabb
    }
}

#[no_mangle]
pub extern "C" fn create_bvh(
    aabbs: *const RTAabb,
    prim_count: usize,
    centers: *const f32,
    center_stride: usize,
    prims_per_leaf: usize,
    bvh_type: BvhType,
) -> RTBvh {
    assert_eq!(center_stride % 4, 0);
    assert!(
        center_stride >= 12,
        "Only centers of 12 <= stride <= 16 should be used."
    );
    assert!(
        center_stride <= 16,
        "Only centers of 12 <= stride <= 16 should be used."
    );

    let aabbs = unsafe { std::slice::from_raw_parts(aabbs as *const Aabb, prim_count) };
    let bvh = unsafe {
        match center_stride {
            12 => {
                let primitives = std::slice::from_raw_parts(centers as *const Vector3, prim_count);
                let builder = rtbvh::Builder {
                    aabbs,
                    primitives,
                    primitives_per_leaf: prims_per_leaf as usize,
                };
                match bvh_type {
                    BvhType::LocallyOrderedClustered => {
                        builder.construct_locally_ordered_clustered()
                    }
                    BvhType::BinnedSAH => builder.construct_binned_sah(),
                }
            }
            16 => {
                let primitives = std::slice::from_raw_parts(centers as *const Vector4, prim_count);
                let builder = rtbvh::Builder {
                    aabbs,
                    primitives,
                    primitives_per_leaf: prims_per_leaf as usize,
                };

                match bvh_type {
                    BvhType::LocallyOrderedClustered => {
                        builder.construct_locally_ordered_clustered()
                    }
                    BvhType::BinnedSAH => builder.construct_binned_sah(),
                }
            }
            _ => panic!("Invalid stride, only centers of 12 <= stride <= 16 should be used."),
        }
    };

    MANAGER.store(bvh)
}

#[no_mangle]
pub extern "C" fn create_mbvh(bvh: RTBvh) -> RTMbvh {
    MANAGER
        .get(bvh.id, |bvh| {
            let mbvh = Mbvh::construct(bvh.unwrap());
            MANAGER.store_mbvh(mbvh)
        })
        .unwrap()
}

#[no_mangle]
pub extern "C" fn refit(aabbs: *const RTAabb, bvh: RTBvh) {
    unsafe {
        MANAGER
            .get_mut(bvh.id, |bvh| {
                let bvh = bvh.unwrap();
                bvh.refit(std::slice::from_raw_parts(
                    aabbs as *const Aabb,
                    bvh.prim_count(),
                ));
            })
            .unwrap();
    }
}

#[no_mangle]
pub extern "C" fn free_bvh(bvh: RTBvh) {
    if MANAGER.free(bvh.id).is_err() {
        eprintln!("Could not free bvh with id: {}", bvh.id);
    }
}

#[no_mangle]
pub extern "C" fn free_mbvh(bvh: RTMbvh) {
    if MANAGER.free_mbvh(bvh.id).is_err() {
        eprintln!("Could not free bvh with id: {}", bvh.id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_size() {
        assert_eq!(
            std::mem::size_of::<BvhNode>(),
            std::mem::size_of::<RTBvhNode>()
        );
        assert_eq!(
            std::mem::size_of::<MbvhNode>(),
            std::mem::size_of::<RTMbvhNode>()
        );
        assert_eq!(std::mem::size_of::<Aabb>(), std::mem::size_of::<RTAabb>());
    }

    #[test]
    fn create_delete() {
        let mut vertices: Vec<Vec3> = Vec::with_capacity(81);

        // 27 Triangles
        for x in 0..=9 {
            for y in 0..=9 {
                vertices.push(Vec3::new(x as f32, y as f32, 0 as f32));
            }
        }

        let aabbs: Vec<Aabb> = (0..27)
            .map(|i| {
                let v0: Vec3 = vertices[i * 3];
                let v1: Vec3 = vertices[i * 3 + 1];
                let v2: Vec3 = vertices[i * 3 + 2];
                aabb!(v0, v1, v2)
            })
            .collect();

        let centers: Vec<Vec3> = aabbs.iter().map(|bb| bb.center()).collect();

        let bvh = create_bvh(
            aabbs.as_ptr() as *const RTAabb,
            27,
            centers.as_ptr() as *const f32,
            16,
            1,
            crate::BvhType::BinnedSAH,
        );

        let mbvh = create_mbvh(bvh);

        free_bvh(bvh);
        free_mbvh(mbvh);
    }
}
