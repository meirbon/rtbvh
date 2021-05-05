[![Build Status](https://github.com/MeirBon/rtbvh/actions/workflows/rust.yml/badge.svg)](https://github.com/MeirBon/rtbvh/actions/workflows/rust.yml)
[![crates.io](https://img.shields.io/crates/v/rtbvh.svg)](https://crates.io/crates/rtbvh)

https://github.com/MeirBon/rtbvh/actions/workflows/rust.yml/badge.svg

# RTBVH
A fast BVH library written in Rust. This library contains implementations of the following BVH building algorithms:
- SAH with Spatial splits (Highest quality, slow build times)
- Binned SAH (High quality, decently fast)
- Locally Ordered Clustering Builder (Very fast build times)

All building algorithms make efficient use of multithreading where possible.
An MBVH implementation is included as well and all the BVH structures implement traversal algorithms for both single rays and quad rays.
This library emits a C/C++ library which is used in my [hobby renderer](https://github.com/meirbon/rendering-fw).

## Performance
In a scene with a teapot of ~6300 triangles, an AMD 32-core 5950x achieves the following performance:
```
Bvh construction with spatial sah type of 6320 primitives took 25.171 ms
Single-threaded rays: 100000000 rays in 22446.148 ms, 4.455107 million rays per second
32 threads rays: 100000000 rays in 1384.02 ms, 72.25329 million rays per second
Single-threaded packets: 100000000 rays in 6407.564 ms, 15.606555 million rays per second
32 threads packets: 100000000 rays in 377.237 ms, 265.08536 million rays per second

Mbvh construction took 1.341 ms
Single-threaded rays: 100000000 rays in 17492.605 ms, 5.7167015 million rays per second
32 threads rays: 100000000 rays in 829.837 ms, 120.5056 million rays per second
Single-threaded packets: 100000000 rays in 5456.565 ms, 18.326548 million rays per second
32 threads packets: 100000000 rays in 287.798 ms, 347.4659 million rays per second

Bvh construction with binned sah type of 6320 primitives took 3.81 ms
Single-threaded rays: 100000000 rays in 21255.336 ms, 4.704701 million rays per second
32 threads rays: 100000000 rays in 1292.286 ms, 77.38225 million rays per second
Single-threaded packets: 100000000 rays in 6052.331 ms, 16.52256 million rays per second
32 threads packets: 100000000 rays in 353.105 ms, 283.20187 million rays per second

Mbvh construction took 1.156 ms
Single-threaded rays: 100000000 rays in 16748.16 ms, 5.970805 million rays per second
32 threads rays: 100000000 rays in 797.133 ms, 125.44958 million rays per second
Single-threaded packets: 100000000 rays in 5180.24 ms, 19.304125 million rays per second
32 threads packets: 100000000 rays in 271.036 ms, 368.95465 million rays per second


bvh-0.5 of 6320 primitives construction took 10.832 ms
Single-threaded rays: 100000000 rays  in 30220.916 ms, 3.3089666 million rays per second
32 threads rays: 100000000 rays in 2153.541 ms, 46.43515 million rays per second
```

## Usage

```rust
use rtbvh::*;
use glam::*;

#[derive(Debug, Copy, Clone)]
struct Triangle {
    vertex0: Vec3,
    vertex1: Vec3,
    vertex2: Vec3,
}

impl Primitive for Triangle {
    fn center(&self) -> Vec3 {
        (self.vertex0 + self.vertex1 + self.vertex2) / 3.0
    }

    fn aabb(&self) -> Aabb {
        let mut aabb = Aabb::new();
        aabb.grow(self.vertex0);
        aabb.grow(self.vertex1);
        aabb.grow(self.vertex2);
        aabb
    }
}

impl SpatialTriangle for Triangle {
    fn vertex0(&self) -> Vec3 {
        self.vertex0
    }

    fn vertex1(&self) -> Vec3 {
        self.vertex1
    }

    fn vertex2(&self) -> Vec3 {
        self.vertex2
    }
}

let vertices: Vec<Vec3> = vec![
    vec3(-1.0, -1.0, 0.0),
    vec3(1.0, -1.0, 0.0),
    vec3(1.0, 1.0, 0.0),
    vec3(-1.0, 1.0, 0.0),
];

let primitives: Vec<Triangle> = vec![
    Triangle {
        vertex0: vertices[0],
        vertex1: vertices[1],
        vertex2: vertices[2],
    },
    Triangle {
        vertex0: vertices[0],
        vertex1: vertices[2],
        vertex2: vertices[3],
    },
];

let aabbs = primitives.iter().map(|t| t.aabb()).collect::<Vec<Aabb>>();

let builder = Builder {
    aabbs: aabbs.as_slice(),
    primitives: primitives.as_slice(),
};

// Choose one of these algorithms:
let bvh = builder.clone().construct_locally_ordered_clustered();
let bvh = builder.clone().construct_binned_sah();
let bvh = builder.construct_spatial_sah();
```

# Planning
- Benchmark
- Add support for packets larger than 4
- Increase coverage of API for FFI