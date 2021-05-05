use crate::*;

pub trait RayIterator<'a, 'b, T: 'a + Primitive>: Iterator<Item = (&'a T, &'b mut Ray)> {}
pub trait PacketIterator<'a, 'b, T: 'a + Primitive>:
    Iterator<Item = (&'a T, &'b mut RayPacket4)>
{
}

pub trait IntoRayIterator<'a, 'b, T: 'a + Primitive> {
    type RIterator: RayIterator<'a, 'b, T>;

    fn iter(&'a self, ray: &'b mut Ray, primitives: &'a [T]) -> Self::RIterator;
}

pub trait IntoPacketIterator<'a, 'b, T: 'a + Primitive> {
    type RIterator: PacketIterator<'a, 'b, T>;

    fn iter(&'a self, packet: &'b mut RayPacket4, primitives: &'a [T]) -> Self::RIterator;
}

#[derive(Debug)]
pub struct BvhIterator<'a, 'b, T: 'a + Primitive> {
    ray: &'b mut Ray,
    i: i32,
    stack: [i32; 32],
    stack_ptr: i32,
    bvh: &'a Bvh,
    primitives: &'a [T],
}

impl<'a, 'b, T: 'a + Primitive> RayIterator<'a, 'b, T> for BvhIterator<'a, 'b, T> {}

impl<'a, 'b, T: 'a + Primitive> BvhIterator<'a, 'b, T> {
    pub fn new(ray: &'b mut Ray, bvh: &'a Bvh, primitives: &'a [T]) -> Self {
        let stack_ptr = if bvh.nodes.is_empty() || primitives.is_empty() {
            -1
        } else {
            0
        };

        Self {
            ray,
            i: 0,
            stack: [0; 32],
            stack_ptr,
            bvh,
            primitives,
        }
    }
}

impl<'a, 'b, T: 'a + Primitive> Iterator for BvhIterator<'a, 'b, T> {
    type Item = (&'a T, &'b mut Ray);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.stack_ptr < 0 {
                return None;
            }

            let node = unsafe {
                self.bvh
                    .nodes
                    .get_unchecked(*self.stack.get_unchecked(self.stack_ptr as usize) as usize)
            };
            self.stack_ptr -= 1;

            let count = node.get_count_unchecked();
            let left_first = node.get_left_first_unchecked();

            if count > -1 {
                if self.i < count {
                    let prim_id = unsafe {
                        *self
                            .bvh
                            .prim_indices
                            .get_unchecked((left_first + self.i) as usize)
                    };
                    self.i += 1;
                    self.stack_ptr += 1;
                    if let Some(prim) = self.primitives.get(prim_id as usize) {
                        return Some((prim, unsafe {
                            std::mem::transmute::<&mut Ray, &'b mut Ray>(self.ray)
                        }));
                    }
                }
                self.i = 0;
            } else if left_first > -1 {
                self.stack_ptr = BvhNode::sort_nodes(
                    unsafe { self.bvh.nodes.get_unchecked(left_first as usize) }
                        .intersect(&self.ray),
                    unsafe { self.bvh.nodes.get_unchecked((left_first + 1) as usize) }
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
pub struct BvhPacketIterator<'a, 'b, T: 'a + Primitive> {
    ray: &'b mut RayPacket4,
    i: i32,
    stack: [i32; 32],
    stack_ptr: i32,
    bvh: &'a Bvh,
    primitives: &'a [T],
}

impl<'a, 'b, T: 'a + Primitive> PacketIterator<'a, 'b, T> for BvhPacketIterator<'a, 'b, T> {}

impl<'a, 'b, T: 'a + Primitive> BvhPacketIterator<'a, 'b, T> {
    pub fn new(ray: &'b mut RayPacket4, bvh: &'a Bvh, primitives: &'a [T]) -> Self {
        let stack_ptr = if bvh.nodes.is_empty() || primitives.is_empty() {
            -1
        } else {
            0
        };
        Self {
            ray,
            i: 0,
            stack: [0; 32],
            stack_ptr,
            bvh,
            primitives,
        }
    }
}

impl<'a, 'b, T: 'a + Primitive> Iterator for BvhPacketIterator<'a, 'b, T> {
    type Item = (&'a T, &'b mut RayPacket4);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.stack_ptr < 0 {
                return None;
            }

            let node = unsafe {
                self.bvh
                    .nodes
                    .get_unchecked(*self.stack.get_unchecked(self.stack_ptr as usize) as usize)
            };
            self.stack_ptr -= 1;

            let count = node.get_count_unchecked();
            let left_first = node.get_left_first_unchecked();

            if count > -1 {
                if self.i < count {
                    let prim_id = unsafe {
                        *self
                            .bvh
                            .prim_indices
                            .get_unchecked((left_first + self.i) as usize)
                    };
                    self.i += 1;
                    self.stack_ptr += 1;
                    if let Some(prim) = self.primitives.get(prim_id as usize) {
                        return Some((prim, unsafe {
                            std::mem::transmute::<&mut RayPacket4, &'b mut RayPacket4>(self.ray)
                        }));
                    }
                }
                self.i = 0;
            } else if left_first > -1 {
                self.stack_ptr = BvhNode::sort_nodes4(
                    unsafe { self.bvh.nodes.get_unchecked(left_first as usize) }
                        .intersect4(self.ray),
                    unsafe { self.bvh.nodes.get_unchecked((left_first + 1) as usize) }
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
pub struct MbvhIterator<'a, 'b, T: 'a + Primitive> {
    ray: &'b mut Ray,
    hit: MbvhHit,
    current: i32,
    i: i32,
    j: i32,
    stack: [i32; 32],
    stack_ptr: i32,
    indices: &'a [u32],
    nodes: &'a [MbvhNode],
    primitives: &'a [T],
}

impl<'a, 'b, T: 'a + Primitive> RayIterator<'a, 'b, T> for MbvhIterator<'a, 'b, T> {}

impl<'a, 'b, T: Primitive> MbvhIterator<'a, 'b, T> {
    pub fn new(ray: &'b mut Ray, bvh: &'a Mbvh, primitives: &'a [T]) -> Self {
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
            primitives,
        }
    }
}

impl<'a, 'b, T: 'a + Primitive> Iterator for MbvhIterator<'a, 'b, T> {
    type Item = (&'a T, &'b mut Ray);

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
                            if let Some(prim) = self.primitives.get(prim_id as usize) {
                                return Some((prim, unsafe {
                                    std::mem::transmute::<&mut Ray, &'b mut Ray>(self.ray)
                                }));
                            }
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
pub struct MbvhPacketIterator<'a, 'b, T: 'a + Primitive> {
    ray: &'b mut RayPacket4,
    hit: MbvhHit,
    current: i32,
    i: i32,
    j: i32,
    stack: [i32; 32],
    stack_ptr: i32,
    indices: &'a [u32],
    nodes: &'a [MbvhNode],
    primitives: &'a [T],
}

impl<'a, 'b, T: 'a + Primitive> PacketIterator<'a, 'b, T> for MbvhPacketIterator<'a, 'b, T> {}

impl<'a, 'b, T: 'a + Primitive> MbvhPacketIterator<'a, 'b, T> {
    pub fn new(ray: &'b mut RayPacket4, bvh: &'a Mbvh, primitives: &'a [T]) -> Self {
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
            primitives,
        }
    }
}

impl<'a, 'b, T: 'a + Primitive> Iterator for MbvhPacketIterator<'a, 'b, T> {
    type Item = (&'a T, &'b mut RayPacket4);

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
                            if let Some(prim) = self.primitives.get(prim_id as usize) {
                                return Some((prim, unsafe {
                                    std::mem::transmute::<&mut RayPacket4, &'b mut RayPacket4>(
                                        self.ray,
                                    )
                                }));
                            }
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
