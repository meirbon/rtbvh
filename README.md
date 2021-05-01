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
In a scene with a teapot of ~6300 triangles, my MacBook Pro with an 8-core i9 9980HK achieves the following performance:
```
Bvh construction with spatial sah type of 6320 primitives took 30.366 ms
Single-threaded rays: 100000000 rays in 18312.771 ms, 5.46067 million rays per second
16 threads rays: 100000000 rays in 2674.832 ms, 37.385525 million rays per second
Single-threaded packets: 100000000 rays in 6490.808 ms, 15.406403 million rays per second
16 threads packets: 100000000 rays in 623.427 ms, 160.4037 million rays per second

Mbvh construction took 1.044 ms
Single-threaded rays: 100000000 rays in 10352.585 ms, 9.659423 million rays per second
16 threads rays: 100000000 rays in 1184.369 ms, 84.43314 million rays per second
Single-threaded packets: 100000000 rays in 2976.157 ms, 33.60038 million rays per second
16 threads packets: 100000000 rays in 399.419 ms, 250.36365 million rays per second

Bvh construction with spatial sah type of 6320 primitives took 5.308 ms
Single-threaded rays: 100000000 rays in 17986.008 ms, 5.5598774 million rays per second
16 threads rays: 100000000 rays in 2655.165 ms, 37.66244 million rays per second
Single-threaded packets: 100000000 rays in 6659.378 ms, 15.0164175 million rays per second
16 threads packets: 100000000 rays in 641.921 ms, 155.78241 million rays per second

Mbvh construction took 1.041 ms
Single-threaded rays: 100000000 rays in 10361.051 ms, 9.65153 million rays per second
16 threads rays: 100000000 rays in 1178.068 ms, 84.88474 million rays per second
Single-threaded packets: 100000000 rays in 3007.244 ms, 33.25304 million rays per second
16 threads packets: 100000000 rays in 401.582 ms, 249.01515 million rays per second

bvh-0.5 of 6320 primitives construction took 18.067 ms
Single-threaded rays: 100000000 rays  in 13449.62 ms, 7.4351544 million rays per second
16 threads rays: 100000000 rays in 2064.21 ms, 48.444683 million rays per second
```

An AMD 12-core 3900x with 3600 MHz RAM achieves the following performance:
```
Bvh construction with spatial_sah type of 6320 primitives took 21.489 ms
Single-threaded rays: 1.000.000 rays in 188.951 ms, 5.2923775 million rays per second
24 threads rays: 1.000.000 rays in 14.228 ms, 70.28395 million rays per second
Single-threaded packets: 1.000.000 rays in 36.914 ms, 27.089993 million rays per second
24 threads packets: 1.000.000 rays in 3.201 ms, 312.40237 million rays per second
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