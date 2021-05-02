use crate::*;

pub struct BvhIterator<'a, T: Primitive> {
    ray: Ray,
    i: i32,
    stack: [i32; 32],
    stack_ptr: i32,
    bvh: &'a Bvh,
    primitives: &'a [T],
}

impl<'a, T: Primitive> BvhIterator<'a, T> {
    pub fn new(ray: Ray, bvh: &'a Bvh, primitives: &'a [T]) -> Self {
        Self {
            ray,
            i: 0,
            stack: [0; 32],
            stack_ptr: 0,
            bvh,
            primitives,
        }
    }
}

impl<'a, T: Primitive> Iterator for BvhIterator<'a, T> {
    type Item = (&'a T, &'a mut Ray);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.stack_ptr < 0 {
                return None;
            }

            #[cfg(all(
                any(target_arch = "x86_64", target_arch = "x86"),
                not(feature = "wasm_support")
            ))]
            unsafe {
                core::arch::x86_64::_mm_prefetch(
                    self.bvh
                        .nodes
                        .as_ptr()
                        .add(self.stack[self.stack_ptr as usize] as usize)
                        as *const i8,
                    core::arch::x86_64::_MM_HINT_T0,
                )
            };

            let node = &self.bvh.nodes[self.stack[self.stack_ptr as usize] as usize];
            self.stack_ptr -= 1;

            let count = node.get_count_unchecked();
            let left_first = node.get_left_first_unchecked();

            if count > -1 {
                if self.i < count {
                    let prim_id = self.bvh.prim_indices[(left_first + self.i) as usize];
                    self.i += 1;
                    self.stack_ptr += 1;
                    return Some((&self.primitives[prim_id as usize], unsafe {
                        std::mem::transmute::<&mut Ray, &'a mut Ray>(&mut self.ray)
                    }));
                }
                self.i = 0;
            } else if left_first > -1 {
                let hit_left = self.bvh.nodes[left_first as usize].intersect(&self.ray);
                let hit_right = self.bvh.nodes[(left_first + 1) as usize].intersect(&self.ray);
                self.stack_ptr = BvhNode::sort_nodes(
                    hit_left,
                    hit_right,
                    &mut self.stack,
                    self.stack_ptr,
                    left_first,
                );
            }
        }
    }
}

pub struct BvhPacketIterator<'a, T: Primitive> {
    ray: RayPacket4,
    i: i32,
    stack: [i32; 32],
    stack_ptr: i32,
    bvh: &'a Bvh,
    primitives: &'a [T],
}

impl<'a, T: Primitive> BvhPacketIterator<'a, T> {
    pub fn new(ray: RayPacket4, bvh: &'a Bvh, primitives: &'a [T]) -> Self {
        Self {
            ray,
            i: 0,
            stack: [0; 32],
            stack_ptr: 0,
            bvh,
            primitives,
        }
    }
}

impl<'a, T: Primitive> Iterator for BvhPacketIterator<'a, T> {
    type Item = (&'a T, &'a mut RayPacket4);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.stack_ptr < 0 {
                return None;
            }

            #[cfg(all(
                any(target_arch = "x86_64", target_arch = "x86"),
                not(feature = "wasm_support")
            ))]
            unsafe {
                core::arch::x86_64::_mm_prefetch(
                    self.bvh
                        .nodes
                        .as_ptr()
                        .add(self.stack[self.stack_ptr as usize] as usize)
                        as *const i8,
                    core::arch::x86_64::_MM_HINT_T0,
                )
            };

            let node = &self.bvh.nodes[self.stack[self.stack_ptr as usize] as usize];
            self.stack_ptr -= 1;

            let count = node.get_count_unchecked();
            let left_first = node.get_left_first_unchecked();

            if count > -1 {
                if self.i < count {
                    let prim_id = self.bvh.prim_indices[(left_first + self.i) as usize];
                    self.i += 1;
                    self.stack_ptr += 1;
                    return Some((&self.primitives[prim_id as usize], unsafe {
                        std::mem::transmute::<&mut RayPacket4, &'a mut RayPacket4>(&mut self.ray)
                    }));
                }
                self.i = 0;
            } else if left_first > -1 {
                let hit_left = self.bvh.nodes[left_first as usize]
                    .bounds
                    .intersect4(&self.ray);
                let hit_right = self.bvh.nodes[(left_first + 1) as usize]
                    .bounds
                    .intersect4(&self.ray);
                self.stack_ptr = BvhNode::sort_nodes4(
                    hit_left,
                    hit_right,
                    &mut self.stack,
                    self.stack_ptr,
                    left_first,
                );
            }
        }
    }
}

pub struct MbvhIterator<'a, T: Primitive> {
    ray: Ray,
    hit: MbvhHit,
    current: usize,
    i: i32,
    j: i32,
    stack: [i32; 32],
    stack_ptr: i32,
    indices: &'a [u32],
    nodes: &'a [MbvhNode],
    primitives: &'a [T],
}

impl<'a, T: Primitive> MbvhIterator<'a, T> {
    pub fn new(ray: Ray, bvh: &'a Mbvh, primitives: &'a [T]) -> Self {
        let hit = bvh.m_nodes[0].intersect(&ray);
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

impl<'a, T: Primitive> Iterator for MbvhIterator<'a, T> {
    type Item = (&'a T, &'a mut Ray);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let node = unsafe { self.nodes.get_unchecked(self.current as usize) };

            if self.i >= 4 {
                if self.stack_ptr < 0 {
                    return None;
                }

                self.current = self.stack[self.stack_ptr as usize] as usize;
                self.stack_ptr -= 1;
                self.hit = unsafe { self.nodes.get_unchecked(self.current).intersect(&self.ray) };
                self.i = 0;
            } else {
                let id = unsafe { *self.hit.ids.get_unchecked(self.i as usize) } as usize;
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
                                    std::mem::transmute::<&mut Ray, &'a mut Ray>(&mut self.ray)
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

pub struct MbvhPacketIterator<'a, T: Primitive> {
    ray: RayPacket4,
    hit: MbvhHit,
    current: i32,
    i: i32,
    j: i32,
    stack: [i32; 32],
    stack_ptr: i32,
    bvh: &'a Mbvh,
    primitives: &'a [T],
}

impl<'a, T: Primitive> MbvhPacketIterator<'a, T> {
    pub fn new(ray: RayPacket4, bvh: &'a Mbvh, primitives: &'a [T]) -> Self {
        let hit = bvh.m_nodes[0].intersect4(&ray);
        Self {
            ray,
            hit,
            current: 0,
            i: 0,
            j: 0,
            stack: [0; 32],
            stack_ptr: -1,
            bvh,
            primitives,
        }
    }
}

impl<'a, T: Primitive> Iterator for MbvhPacketIterator<'a, T> {
    type Item = (&'a T, &'a mut RayPacket4);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let node_id = self.current as usize;

            if self.i >= 4 {
                if self.stack_ptr < 0 {
                    return None;
                }

                self.current = self.stack[self.stack_ptr as usize];
                self.stack_ptr -= 1;
                self.hit = self.bvh.m_nodes[self.current as usize].intersect4(&self.ray);
                self.i = 0;
            } else {
                let id = self.hit.ids[self.i as usize] as usize;
                if self.hit.result[id] {
                    let count = self.bvh.m_nodes[node_id].counts[id];
                    let left_first = self.bvh.m_nodes[node_id].children[id];
                    if count > -1 {
                        if self.j < count {
                            let prim_id = self.bvh.prim_indices[(left_first + self.j) as usize];
                            self.j += 1;
                            return Some((&self.primitives[prim_id as usize], unsafe {
                                std::mem::transmute::<&mut RayPacket4, &'a mut RayPacket4>(
                                    &mut self.ray,
                                )
                            }));
                        }
                        self.j = 0;
                    } else if left_first > -1 {
                        self.stack_ptr += 1;
                        self.stack[self.stack_ptr as usize] = left_first;
                    }
                }
                self.i += 1;
            }
        }
    }
}
