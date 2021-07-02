use crate::*;

pub trait RayIndexIterator<'a>: Iterator<Item = (u32, &'a mut Ray)> {}
pub trait PacketIndexIterator<'a>: Iterator<Item = (u32, &'a mut RayPacket4)> {}

pub trait IntoRayIndexIterator<'a> {
    type RIterator: RayIndexIterator<'a>;

    fn iter(&'a self, ray: &'a mut Ray) -> Self::RIterator;
}

pub trait IntoPacketIndexIterator<'a> {
    type RIterator: PacketIndexIterator<'a>;

    fn iter(&'a self, packet: &'a mut RayPacket4) -> Self::RIterator;
}

#[derive(Debug)]
pub struct BvhIndexIterator<'a> {
    ray: &'a mut Ray,
    i: i32,
    stack: [i32; 32],
    stack_ptr: i32,
    nodes: &'a [BvhNode],
    indices: &'a [u32],
}

impl<'a> RayIndexIterator<'a> for BvhIndexIterator<'a> {}

impl<'a> BvhIndexIterator<'a> {
    pub fn new(ray: &'a mut Ray, bvh: &'a Bvh) -> Self {
        let stack_ptr = if bvh.nodes.is_empty() || ray.origin.is_nan() || ray.direction.is_nan() {
            -1
        } else {
            0
        };

        Self {
            ray,
            i: 0,
            stack: [0; 32],
            stack_ptr,
            nodes: &bvh.nodes,
            indices: &bvh.prim_indices,
        }
    }

    pub fn from_slices(ray: &'a mut Ray, nodes: &'a [BvhNode], indices: &'a [u32]) -> Self {
        let stack_ptr = if nodes.is_empty() || ray.origin.is_nan() || ray.direction.is_nan() {
            -1
        } else {
            0
        };

        Self {
            ray,
            i: 0,
            stack: [0; 32],
            stack_ptr,
            nodes,
            indices,
        }
    }
}

impl<'a> Iterator for BvhIndexIterator<'a> {
    type Item = (u32, &'a mut Ray);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.stack_ptr < 0 {
                return None;
            }

            let node = unsafe {
                self.nodes
                    .get_unchecked(*self.stack.get_unchecked(self.stack_ptr as usize) as usize)
            };
            self.stack_ptr -= 1;

            let count = node.get_count_unchecked();
            let left_first = node.get_left_first_unchecked();

            if count > -1 {
                if self.i < count {
                    let prim_id =
                        unsafe { *self.indices.get_unchecked((left_first + self.i) as usize) };
                    self.i += 1;
                    self.stack_ptr += 1;
                    return Some((prim_id, unsafe {
                        std::mem::transmute::<&mut Ray, &'a mut Ray>(self.ray)
                    }));
                }
                self.i = 0;
            } else if left_first > -1 {
                self.stack_ptr = BvhNode::sort_nodes(
                    unsafe { self.nodes.get_unchecked(left_first as usize) }.intersect(&self.ray),
                    unsafe { self.nodes.get_unchecked((left_first + 1) as usize) }
                        .intersect(&self.ray),
                    &mut self.stack,
                    self.stack_ptr,
                    left_first,
                );
            }
        }
    }
}

#[derive(Debug)]
pub struct BvhPacketIndexIterator<'a> {
    ray: &'a mut RayPacket4,
    i: i32,
    stack: [i32; 32],
    stack_ptr: i32,
    nodes: &'a [BvhNode],
    indices: &'a [u32],
}

impl<'a> PacketIndexIterator<'a> for BvhPacketIndexIterator<'a> {}

impl<'a> BvhPacketIndexIterator<'a> {
    pub fn new(ray: &'a mut RayPacket4, bvh: &'a Bvh) -> Self {
        let stack_ptr = if bvh.nodes.is_empty()
            || ray.origin_x.is_nan()
            || ray.origin_y.is_nan()
            || ray.origin_z.is_nan()
            || ray.direction_x.is_nan()
            || ray.direction_y.is_nan()
            || ray.direction_z.is_nan()
        {
            -1
        } else {
            0
        };
        Self {
            ray,
            i: 0,
            stack: [0; 32],
            stack_ptr,
            nodes: &bvh.nodes,
            indices: &bvh.prim_indices,
        }
    }

    pub fn from_slices(ray: &'a mut RayPacket4, nodes: &'a [BvhNode], indices: &'a [u32]) -> Self {
        let stack_ptr = if nodes.is_empty()
            || ray.origin_x.is_nan()
            || ray.origin_y.is_nan()
            || ray.origin_z.is_nan()
            || ray.direction_x.is_nan()
            || ray.direction_y.is_nan()
            || ray.direction_z.is_nan()
        {
            -1
        } else {
            0
        };
        Self {
            ray,
            i: 0,
            stack: [0; 32],
            stack_ptr,
            nodes,
            indices,
        }
    }
}

impl<'a> Iterator for BvhPacketIndexIterator<'a> {
    type Item = (u32, &'a mut RayPacket4);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.stack_ptr < 0 {
                return None;
            }

            let node = unsafe {
                self.nodes
                    .get_unchecked(*self.stack.get_unchecked(self.stack_ptr as usize) as usize)
            };
            self.stack_ptr -= 1;

            let count = node.get_count_unchecked();
            let left_first = node.get_left_first_unchecked();

            if count > -1 {
                if self.i < count {
                    let prim_id =
                        unsafe { *self.indices.get_unchecked((left_first + self.i) as usize) };
                    self.i += 1;
                    self.stack_ptr += 1;
                    return Some((prim_id, unsafe {
                        std::mem::transmute::<&mut RayPacket4, &'a mut RayPacket4>(self.ray)
                    }));
                }
                self.i = 0;
            } else if left_first > -1 {
                self.stack_ptr = BvhNode::sort_nodes4(
                    unsafe { self.nodes.get_unchecked(left_first as usize) }.intersect4(self.ray),
                    unsafe { self.nodes.get_unchecked((left_first + 1) as usize) }
                        .intersect4(self.ray),
                    &mut self.stack,
                    self.stack_ptr,
                    left_first,
                );
            }
        }
    }
}

#[derive(Debug)]
pub struct MbvhIndexIterator<'a> {
    ray: &'a mut Ray,
    hit: MbvhHit,
    current: i32,
    i: i32,
    j: i32,
    stack: [i32; 32],
    stack_ptr: i32,
    indices: &'a [u32],
    nodes: &'a [MbvhNode],
}

impl<'a> RayIndexIterator<'a> for MbvhIndexIterator<'a> {}

impl<'a> MbvhIndexIterator<'a> {
    pub fn new(ray: &'a mut Ray, bvh: &'a Mbvh) -> Self {
        let hit = bvh
            .m_nodes
            .get(0)
            .map(|n| n.intersect(&ray))
            .unwrap_or_default();
        Self {
            ray,
            hit,
            current: 0,
            i: 0,
            j: 0,
            stack: [0; 32],
            stack_ptr: -1,
            indices: &bvh.prim_indices,
            nodes: &bvh.m_nodes,
        }
    }

    pub fn from_slices(ray: &'a mut Ray, nodes: &'a [MbvhNode], indices: &'a [u32]) -> Self {
        let hit = nodes.get(0).map(|n| n.intersect(&ray)).unwrap_or_default();

        Self {
            ray,
            hit,
            current: 0,
            i: 0,
            j: 0,
            stack: [0; 32],
            stack_ptr: -1,
            indices,
            nodes,
        }
    }
}

impl<'a> Iterator for MbvhIndexIterator<'a> {
    type Item = (u32, &'a mut Ray);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let node = unsafe { self.nodes.get_unchecked(self.current as usize) };

            if self.i >= 4 {
                if self.stack_ptr < 0 {
                    return None;
                }

                self.current = self.stack[self.stack_ptr as usize];
                self.stack_ptr -= 1;
                self.hit = unsafe {
                    self.nodes
                        .get_unchecked(self.current as usize)
                        .intersect(&self.ray)
                };
                self.i = 0;
            } else {
                let id = unsafe { *self.hit.ids.get_unchecked((3 - self.i) as usize) } as usize;
                if unsafe { *self.hit.result.get_unchecked(id) } {
                    let count = node.counts[id];
                    let left_first = node.children[id];

                    if count > -1 {
                        if self.j < count {
                            let prim_id = unsafe {
                                *self.indices.get_unchecked((left_first + self.j) as usize)
                            };
                            self.j += 1;

                            return Some((prim_id, unsafe {
                                std::mem::transmute::<&mut Ray, &'a mut Ray>(self.ray)
                            }));
                        }
                        self.j = 0;
                    } else if left_first > -1 {
                        self.stack_ptr += 1;
                        unsafe {
                            *self.stack.get_unchecked_mut(self.stack_ptr as usize) = left_first;
                        }
                    }
                }
                self.i += 1;
            }
        }
    }
}

#[derive(Debug)]
pub struct MbvhPacketIndexIterator<'a> {
    ray: &'a mut RayPacket4,
    hit: MbvhHit,
    current: i32,
    i: i32,
    j: i32,
    stack: [i32; 32],
    stack_ptr: i32,
    indices: &'a [u32],
    nodes: &'a [MbvhNode],
}

impl<'a> PacketIndexIterator<'a> for MbvhPacketIndexIterator<'a> {}

impl<'a> MbvhPacketIndexIterator<'a> {
    pub fn new(ray: &'a mut RayPacket4, bvh: &'a Mbvh) -> Self {
        let hit = bvh
            .m_nodes
            .get(0)
            .map(|n| n.intersect4(&ray))
            .unwrap_or_default();
        Self {
            ray,
            hit,
            current: 0,
            i: 0,
            j: 0,
            stack: [0; 32],
            stack_ptr: -1,
            indices: &bvh.prim_indices,
            nodes: &bvh.m_nodes,
        }
    }

    pub fn from_slices(ray: &'a mut RayPacket4, nodes: &'a [MbvhNode], indices: &'a [u32]) -> Self {
        let hit = nodes.get(0).map(|n| n.intersect4(&ray)).unwrap_or_default();

        Self {
            ray,
            hit,
            current: 0,
            i: 0,
            j: 0,
            stack: [0; 32],
            stack_ptr: -1,
            indices,
            nodes,
        }
    }
}

impl<'a> Iterator for MbvhPacketIndexIterator<'a> {
    type Item = (u32, &'a mut RayPacket4);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let node = unsafe { self.nodes.get_unchecked(self.current as usize) };

            if self.i >= 4 {
                if self.stack_ptr < 0 {
                    return None;
                }

                self.current = self.stack[self.stack_ptr as usize];
                self.stack_ptr -= 1;
                self.hit = unsafe {
                    self.nodes
                        .get_unchecked(self.current as usize)
                        .intersect4(&self.ray)
                };
                self.i = 0;
            } else {
                let id = unsafe { *self.hit.ids.get_unchecked((3 - self.i) as usize) } as usize;
                if unsafe { *self.hit.result.get_unchecked(id) } {
                    let count = node.counts[id];
                    let left_first = node.children[id];

                    if count > -1 {
                        if self.j < count {
                            let prim_id = unsafe {
                                *self.indices.get_unchecked((left_first + self.j) as usize)
                            };
                            self.j += 1;
                            return Some((prim_id, unsafe {
                                std::mem::transmute::<&mut RayPacket4, &'a mut RayPacket4>(self.ray)
                            }));
                        }
                        self.j = 0;
                    } else if left_first > -1 {
                        self.stack_ptr += 1;
                        unsafe {
                            *self.stack.get_unchecked_mut(self.stack_ptr as usize) = left_first;
                        }
                    }
                }
                self.i += 1;
            }
        }
    }
}
