#![allow(clippy::upper_case_acronyms)]
use glam::*;
use lazy_static::lazy_static;
use parking_lot::RwLock;
use rtbvh::*;
use std::{num::NonZeroUsize, slice};

lazy_static! {
    static ref MANAGER: StructureManager = StructureManager::default();
}

struct StructureManager {
    structures: RwLock<Vec<RwLock<Bvh>>>,
    m_structures: RwLock<Vec<RwLock<Mbvh>>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(C)]
pub enum ResultCode {
    Ok = 0,
    Error = 1,
    NoPrimitives = 2,
    InequalAabbsAndPrimitives = 3,
    Nan = 4,
}

impl From<BuildError> for ResultCode {
    fn from(err: BuildError) -> Self {
        match err {
            BuildError::NoPrimitives => Self::NoPrimitives,
            BuildError::InequalAabbsAndPrimitives(_, _) => Self::InequalAabbsAndPrimitives,
        }
    }
}

impl Default for StructureManager {
    fn default() -> Self {
        StructureManager {
            structures: RwLock::new(Vec::new()),
            m_structures: RwLock::new(Vec::new()),
        }
    }
}

impl StructureManager {
    pub fn store(&self, bvh: Bvh) -> RTBvh {
        let mut lock = self.structures.write();
        lock.push(RwLock::new(bvh));
        let id = lock.len() - 1;

        let bvh = lock[id].read();

        RTBvh {
            id: id as u32,
            node_count: bvh.nodes().len() as u32,
            nodes: bvh.nodes().as_ptr() as *const RTBvhNode,
            index_count: bvh.indices().len() as u32,
            indices: bvh.indices().as_ptr(),
        }
    }

    pub fn store_mbvh(&self, mbvh: Mbvh) -> RTMbvh {
        let mut lock = self.m_structures.write();

        lock.push(RwLock::new(mbvh));
        let id = lock.len() - 1;

        let mbvh = lock[id].read();
        let mbvh: &Mbvh = &mbvh;

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
        T: FnMut(&Bvh) -> B,
    {
        let lock = self.structures.read();
        if let Some(bvh) = lock.get(id as usize) {
            let bvh = bvh.read();
            let bvh: &Bvh = &bvh;
            Some(cb(bvh))
        } else {
            None
        }
    }

    pub fn get_mut<T, B>(&self, id: u32, mut cb: T) -> Option<B>
    where
        T: FnMut(&mut Bvh) -> B,
    {
        let lock = self.structures.read();
        if let Some(bvh) = lock.get(id as usize) {
            let mut bvh = bvh.write();
            let bvh: &mut Bvh = &mut bvh;
            Some(cb(bvh))
        } else {
            None
        }
    }

    pub fn free(&self, id: u32) -> Result<(), ()> {
        let mut lock = self.structures.write();
        if let Some(m) = lock.get_mut(id as usize) {
            *m.get_mut() = Bvh::default();
            Ok(())
        } else {
            Err(())
        }
    }

    pub fn free_mbvh(&self, id: u32) -> Result<(), ()> {
        let mut lock = self.m_structures.write();
        if let Some(m) = lock.get_mut(id as usize) {
            *m.get_mut() = Mbvh::default();
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

impl Default for RTBvh {
    fn default() -> Self {
        Self {
            id: u32::MAX,
            node_count: 0,
            nodes: std::ptr::null(),
            index_count: 0,
            indices: std::ptr::null(),
        }
    }
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

impl Default for RTMbvh {
    fn default() -> Self {
        Self {
            id: u32::MAX,
            node_count: 0,
            nodes: std::ptr::null(),
            index_count: 0,
            indices: std::ptr::null(),
        }
    }
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

/// Creates a new spatial bvh from triangles.
/// If aabbs is null, aabbs will be computed from the input triangles.
///
/// # Safety
///
/// This function is unsafe as it dereferences pointers.
#[no_mangle]
pub unsafe extern "C" fn create_spatial_Bvh(
    aabbs: *const RTAabb,
    prim_count: usize,
    centers: *const f32,
    stride: usize,
    vertices: *const f32,
    vertex_stride: usize,
    triangle_stride: usize,
    prims_per_leaf: u32,
    result: *mut RTBvh,
) -> ResultCode {
    assert_eq!(stride % 4, 0);

    let aabbs = if !aabbs.is_null() {
        Some(slice::from_raw_parts(aabbs as *const Aabb, prim_count))
    } else {
        None
    };

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

    match (Builder {
        aabbs,
        primitives: primitives.as_slice(),
        primitives_per_leaf: NonZeroUsize::new(prims_per_leaf as usize),
    })
    .construct_spatial_sah()
    {
        Ok(bvh) => {
            *result = MANAGER.store(bvh);
            ResultCode::Ok
        }
        Err(e) => e.into(),
    }
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

/// # Safety
///
/// This function is unsafe as it dereferences pointers.
#[no_mangle]
pub unsafe extern "C" fn create_bvh(
    aabbs: *const RTAabb,
    prim_count: usize,
    centers: *const f32,
    center_stride: usize,
    prims_per_leaf: usize,
    bvh_type: BvhType,
    result: *mut RTBvh,
) -> ResultCode {
    if centers.is_null() || result.is_null() {
        return ResultCode::Error;
    }

    assert_eq!(center_stride % 4, 0);
    assert!(
        center_stride >= 12,
        "Only centers of 12 <= stride <= 16 should be used."
    );
    assert!(
        center_stride <= 16,
        "Only centers of 12 <= stride <= 16 should be used."
    );

    let aabbs = if !aabbs.is_null() {
        Some(slice::from_raw_parts(aabbs as *const Aabb, prim_count))
    } else {
        None
    };

    let bvh = match center_stride {
        12 => {
            let primitives = slice::from_raw_parts(centers as *const Vector3, prim_count);
            let builder = rtbvh::Builder {
                aabbs,
                primitives,
                primitives_per_leaf: NonZeroUsize::new(prims_per_leaf as usize),
            };
            match bvh_type {
                BvhType::LocallyOrderedClustered => builder.construct_locally_ordered_clustered(),
                BvhType::BinnedSAH => builder.construct_binned_sah(),
            }
        }
        16 => {
            let primitives = slice::from_raw_parts(centers as *const Vector4, prim_count);
            let builder = rtbvh::Builder {
                aabbs,
                primitives,
                primitives_per_leaf: NonZeroUsize::new(prims_per_leaf as usize),
            };

            match bvh_type {
                BvhType::LocallyOrderedClustered => builder.construct_locally_ordered_clustered(),
                BvhType::BinnedSAH => builder.construct_binned_sah(),
            }
        }
        _ => panic!("Invalid stride, only centers of 12 <= stride <= 16 should be used."),
    };

    match bvh {
        Ok(bvh) => {
            *result = MANAGER.store(bvh);
            ResultCode::Ok
        }
        Err(e) => e.into(),
    }
}

/// # Safety
///
/// This function is unsafe as it dereferences pointers.
#[no_mangle]
pub unsafe extern "C" fn create_mbvh(bvh: RTBvh, mbvh: *mut RTMbvh) -> ResultCode {
    if mbvh.is_null() || bvh.nodes.is_null() || bvh.indices.is_null() {
        return ResultCode::Error;
    }

    if let Some(m) = MANAGER.get(bvh.id, |bvh| {
        let mbvh = Mbvh::construct(bvh);
        MANAGER.store_mbvh(mbvh)
    }) {
        *mbvh = m;
        ResultCode::Ok
    } else {
        ResultCode::Error
    }
}

/// # Safety
///
/// This function is unsafe as it dereferences pointers.
#[no_mangle]
pub unsafe extern "C" fn refit(aabbs: *const RTAabb, bvh: RTBvh) -> ResultCode {
    if aabbs.is_null() || bvh.nodes.is_null() || bvh.indices.is_null() {
        return ResultCode::Error;
    }

    let refitted = MANAGER
        .get_mut(bvh.id, |bvh| {
            bvh.refit(slice::from_raw_parts(
                aabbs as *const Aabb,
                bvh.prim_count(),
            ));
        })
        .is_some();

    if refitted {
        ResultCode::Ok
    } else {
        ResultCode::Error
    }
}

/// # Safety
///
/// This function uses external function pointers which are unsafe.
/// origin must be a pointer to a 3-component float array
/// direction must be a pointer to a 3-component float array
/// user_data may be null
/// intersect must not be null and is called for every potential primitive.
/// intersect takes the following arguments:
///     (in primitive_id, inout t_value, user_data)
/// if intersect returns true, the intersection loop will early out
#[no_mangle]
pub unsafe extern "C" fn intersect(
    bvh: RTBvh,
    origin: *const f32,
    direction: *const f32,
    t: *mut f32,
    user_data: *mut std::ffi::c_void,
    intersect: extern "C" fn(u32, *mut f32, *mut std::ffi::c_void) -> bool,
) -> ResultCode {
    let origin = Vec3::new(*origin, *origin.add(1), *origin.add(2));
    let direction = Vec3::new(*direction, *direction.add(1), *direction.add(2));
    if origin.is_nan() || direction.is_nan() {
        return ResultCode::Nan;
    }

    let mut ray = Ray::new(origin, direction);
    ray.t = *t;

    let nodes = slice::from_raw_parts(bvh.nodes as *const BvhNode, bvh.node_count as usize);
    let indices = slice::from_raw_parts(bvh.indices, bvh.index_count as usize);

    let iter = BvhIndexIterator::from_slices(&mut ray, nodes, indices);
    for (index, ray) in iter {
        if intersect(index, &mut ray.t, user_data) {
            break;
        }
    }

    *t = ray.t;

    ResultCode::Ok
}

/// # Safety
///
/// This function uses external function pointers which are unsafe.
///
/// origin_x must be a pointer to a 4-component float array
/// origin_y must be a pointer to a 4-component float array
/// origin_z must be a pointer to a 4-component float array
/// direction_x must be a pointer to a 3-component float array
/// direction_y must be a pointer to a 3-component float array
/// direction_z must be a pointer to a 3-component float array
/// user_data may be null
/// intersect must not be null and is called for every potential primitive.
/// intersect takes the following arguments:
///     (in primitive_id, inout 4-wide t_value, user_data)
/// if intersect returns true, the intersection loop will early out
#[no_mangle]
pub unsafe extern "C" fn intersect_packet(
    bvh: RTBvh,
    origin_x: *const f32,
    origin_y: *const f32,
    origin_z: *const f32,
    direction_x: *const f32,
    direction_y: *const f32,
    direction_z: *const f32,
    t: *mut f32,
    user_data: *mut std::ffi::c_void,
    intersect: extern "C" fn(u32, *mut f32, *mut std::ffi::c_void) -> bool,
) -> ResultCode {
    let origin_x = Vec4::new(
        *origin_x,
        *origin_x.add(1),
        *origin_x.add(2),
        *origin_x.add(3),
    );
    let origin_y = Vec4::new(
        *origin_y,
        *origin_y.add(1),
        *origin_y.add(2),
        *origin_y.add(3),
    );
    let origin_z = Vec4::new(
        *origin_z,
        *origin_z.add(1),
        *origin_z.add(2),
        *origin_z.add(3),
    );
    let direction_x = Vec4::new(
        *direction_x,
        *direction_x.add(1),
        *direction_x.add(2),
        *direction_x.add(3),
    );
    let direction_y = Vec4::new(
        *direction_y,
        *direction_y.add(1),
        *direction_y.add(2),
        *direction_y.add(3),
    );
    let direction_z = Vec4::new(
        *direction_z,
        *direction_z.add(1),
        *direction_z.add(2),
        *direction_z.add(3),
    );

    let t_value = Vec4::new(*t, *t.add(1), *t.add(2), *t.add(3));

    if origin_x.is_nan()
        || origin_y.is_nan()
        || origin_z.is_nan()
        || direction_x.is_nan()
        || direction_y.is_nan()
        || direction_z.is_nan()
    {
        return ResultCode::Nan;
    }

    let mut ray = RayPacket4 {
        origin_x,
        origin_y,
        origin_z,
        direction_x,
        direction_y,
        direction_z,
        inv_direction_x: Vec4::ONE / direction_x,
        inv_direction_y: Vec4::ONE / direction_y,
        inv_direction_z: Vec4::ONE / direction_z,
        t: t_value,
    };

    let nodes = slice::from_raw_parts(bvh.nodes as *const BvhNode, bvh.node_count as usize);
    let indices = slice::from_raw_parts(bvh.indices, bvh.index_count as usize);

    let iter = BvhPacketIndexIterator::from_slices(&mut ray, nodes, indices);
    for (index, ray) in iter {
        if intersect(index, &mut ray.t as *mut Vec4 as *mut f32, user_data) {
            break;
        }
    }

    std::ptr::copy_nonoverlapping(&ray.t as *const Vec4, t as *mut Vec4, 1);

    ResultCode::Ok
}

/// # Safety
///
/// This function uses external function pointers which are unsafe.
///
/// origin must be a pointer to a 3-component float array
/// direction must be a pointer to a 3-component float array
/// user_data may be null
/// intersect must not be null and is called for every potential primitive.
/// intersect takes the following arguments:
///     (in primitive_id, inout t_value, user_data)
/// if intersect returns true, the intersection loop will early out
#[no_mangle]
pub unsafe extern "C" fn intersect_mbvh(
    bvh: RTMbvh,
    origin: *const f32,
    direction: *const f32,
    t: *mut f32,
    user_data: *mut std::ffi::c_void,
    intersect: extern "C" fn(u32, *mut f32, *mut std::ffi::c_void) -> bool,
) -> ResultCode {
    let origin = Vec3::new(*origin, *origin.add(1), *origin.add(2));
    let direction = Vec3::new(*direction, *direction.add(1), *direction.add(2));

    if origin.is_nan() || direction.is_nan() {
        return ResultCode::Nan;
    }

    let mut ray = Ray::new(origin, direction);
    ray.t = *t;

    let nodes = slice::from_raw_parts(bvh.nodes as *const MbvhNode, bvh.node_count as usize);
    let indices = slice::from_raw_parts(bvh.indices, bvh.index_count as usize);

    let iter = MbvhIndexIterator::from_slices(&mut ray, nodes, indices);
    for (index, ray) in iter {
        if intersect(index, &mut ray.t, user_data) {
            break;
        }
    }

    *t = ray.t;

    ResultCode::Ok
}

/// # Safety
///
/// This function uses external function pointers which are unsafe.
///
/// origin_x must be a pointer to a 4-component float array
/// origin_y must be a pointer to a 4-component float array
/// origin_z must be a pointer to a 4-component float array
/// direction_x must be a pointer to a 3-component float array
/// direction_y must be a pointer to a 3-component float array
/// direction_z must be a pointer to a 3-component float array
/// user_data may be null
/// intersect must not be null and is called for every potential primitive.
/// intersect takes the following arguments:
///     (in primitive_id, inout 4-wide t_value, user_data)
/// if intersect returns true, the intersection loop will early out
#[no_mangle]
pub unsafe extern "C" fn intersect_mbvh_packet(
    bvh: RTMbvh,
    origin_x: *const f32,
    origin_y: *const f32,
    origin_z: *const f32,
    direction_x: *const f32,
    direction_y: *const f32,
    direction_z: *const f32,
    t: *mut f32,
    user_data: *mut std::ffi::c_void,
    intersect: extern "C" fn(u32, *mut f32, *mut std::ffi::c_void) -> bool,
) -> ResultCode {
    let origin_x = Vec4::new(
        *origin_x,
        *origin_x.add(1),
        *origin_x.add(2),
        *origin_x.add(3),
    );
    let origin_y = Vec4::new(
        *origin_y,
        *origin_y.add(1),
        *origin_y.add(2),
        *origin_y.add(3),
    );
    let origin_z = Vec4::new(
        *origin_z,
        *origin_z.add(1),
        *origin_z.add(2),
        *origin_z.add(3),
    );
    let direction_x = Vec4::new(
        *direction_x,
        *direction_x.add(1),
        *direction_x.add(2),
        *direction_x.add(3),
    );
    let direction_y = Vec4::new(
        *direction_y,
        *direction_y.add(1),
        *direction_y.add(2),
        *direction_y.add(3),
    );
    let direction_z = Vec4::new(
        *direction_z,
        *direction_z.add(1),
        *direction_z.add(2),
        *direction_z.add(3),
    );
    let t_value = Vec4::new(*t, *t.add(1), *t.add(2), *t.add(3));

    if origin_x.is_nan()
        || origin_y.is_nan()
        || origin_z.is_nan()
        || direction_x.is_nan()
        || direction_y.is_nan()
        || direction_z.is_nan()
    {
        return ResultCode::Nan;
    }

    let mut ray = RayPacket4 {
        origin_x,
        origin_y,
        origin_z,
        direction_x,
        direction_y,
        direction_z,
        inv_direction_x: Vec4::ONE / direction_x,
        inv_direction_y: Vec4::ONE / direction_y,
        inv_direction_z: Vec4::ONE / direction_z,
        t: t_value,
    };

    let nodes = slice::from_raw_parts(bvh.nodes as *const MbvhNode, bvh.node_count as usize);
    let indices = slice::from_raw_parts(bvh.indices, bvh.index_count as usize);

    let iter = MbvhPacketIndexIterator::from_slices(&mut ray, nodes, indices);
    for (index, ray) in iter {
        if intersect(index, &mut ray.t as *mut Vec4 as *mut f32, user_data) {
            break;
        }
    }

    std::ptr::copy_nonoverlapping(&ray.t as *const Vec4, t as *mut Vec4, 1);

    ResultCode::Ok
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
        let mut bvh = RTBvh::default();

        assert_eq!(
            unsafe {
                create_bvh(
                    std::ptr::null(),
                    27,
                    std::ptr::null(),
                    16,
                    1,
                    crate::BvhType::BinnedSAH,
                    &mut bvh,
                )
            },
            ResultCode::Error
        );

        assert_eq!(
            unsafe {
                create_bvh(
                    std::ptr::null(),
                    27,
                    centers.as_ptr() as *const f32,
                    16,
                    1,
                    crate::BvhType::BinnedSAH,
                    &mut bvh,
                )
            },
            ResultCode::Ok
        );

        free_bvh(bvh);

        assert_eq!(
            unsafe {
                create_bvh(
                    aabbs.as_ptr() as *const RTAabb,
                    27,
                    centers.as_ptr() as *const f32,
                    16,
                    1,
                    crate::BvhType::BinnedSAH,
                    &mut bvh,
                )
            },
            ResultCode::Ok
        );

        let mut mbvh = RTMbvh::default();
        assert_eq!(unsafe { create_mbvh(bvh, &mut mbvh) }, ResultCode::Ok);

        free_bvh(bvh);
        free_mbvh(mbvh);
    }

    #[test]
    fn intersect() {
        let vertices = vec![
            vec3(-1.0, -1.0, 1.0),
            vec3(1.0, -1.0, 1.0),
            vec3(1.0, 1.0, 1.0),
            vec3(-1.0, -1.0, 1.0),
            vec3(1.0, 1.0, 1.0),
            vec3(-1.0, 1.0, 1.0),
        ];

        let aabbs: Vec<Aabb> = (0..2)
            .map(|i| {
                let v0: Vec3 = vertices[i * 3];
                let v1: Vec3 = vertices[i * 3 + 1];
                let v2: Vec3 = vertices[i * 3 + 2];
                aabb!(v0, v1, v2)
            })
            .collect();

        let centers: Vec<Vec3> = aabbs.iter().map(|bb| bb.center()).collect();
        let mut bvh = RTBvh::default();
        let mut mbvh = RTMbvh::default();

        assert_eq!(
            unsafe {
                create_bvh(
                    aabbs.as_ptr() as *const RTAabb,
                    2,
                    centers.as_ptr() as *const f32,
                    std::mem::size_of::<Vec3>() as _,
                    1,
                    crate::BvhType::BinnedSAH,
                    &mut bvh,
                )
            },
            ResultCode::Ok
        );

        assert_eq!(unsafe { create_mbvh(bvh, &mut mbvh,) }, ResultCode::Ok);

        let origin = Vec3::ZERO;
        let direction = Vec3::Z;
        let mut t = 1e26_f32;
        let mut user_data = UserData {
            origin,
            direction,
            vertices: &vertices,
        };

        unsafe {
            crate::intersect(
                bvh,
                &origin as *const Vec3 as *const f32,
                &direction as *const Vec3 as *const f32,
                &mut t,
                &mut user_data as *mut UserData as *mut std::ffi::c_void,
                intersect_test,
            );
        }
        assert!((t - 1.0).abs() < f32::EPSILON);

        t = 1e26_f32;
        unsafe {
            crate::intersect_mbvh(
                mbvh,
                &origin as *const Vec3 as *const f32,
                &direction as *const Vec3 as *const f32,
                &mut t,
                &mut user_data as *mut UserData as *mut std::ffi::c_void,
                intersect_test,
            );
        }
        assert!((t - 1.0).abs() < f32::EPSILON);
    }

    struct UserData<'a> {
        origin: Vec3,
        direction: Vec3,
        vertices: &'a [Vec3],
    }

    extern "C" fn intersect_test(id: u32, t: *mut f32, data: *mut std::ffi::c_void) -> bool {
        let data = unsafe { &*(data as *const UserData) };

        let v0 = data.vertices[id as usize * 3];
        let v1 = data.vertices[id as usize * 3 + 1];
        let v2 = data.vertices[id as usize * 3 + 2];

        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let h_val = data.direction.cross(edge2);
        let a_val = edge1.dot(h_val);
        if a_val > -1e-5 && a_val < 1e-5 {
            return false;
        }
        let f_val = 1.0 / a_val;
        let s_val = data.origin - v0.xyz();
        let u_val = f_val * s_val.dot(h_val);
        if !(0.0..=1.0).contains(&u_val) {
            return false;
        }
        let q_val = s_val.cross(edge1);
        let v_val = f_val * data.direction.dot(q_val);
        if v_val < 0. || (u_val + v_val) > 1.0 {
            return false;
        }

        let t_val: f32 = f_val * edge2.dot(q_val);
        unsafe {
            if t_val > 1e-5 && t_val < *t {
                *t = t_val;
            }
        }
        false
    }
}
