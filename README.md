[![Build Status](https://travis-ci.org/MeirBon/rtbvh.svg?branch=master)](https://travis-ci.org/MeirBon/rtbvh)

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
Bvh construction with spatial_sah type of 6320 primitives took 33.324 ms
Single-threaded rays: 1.000.000 rays in 192.859 ms, 5.1851354 million rays per second
16 threads rays: 1.000.000 rays in 22.546 ms, 44.353767 million rays per second
Single-threaded packets: 1.000.000 rays in 42.497 ms, 23.531073 million rays per second
16 threads packets: 1.000.000 rays in 4.992 ms, 200.32051 million rays per second
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

#[derive(Debug, Copy, Clone)]
struct Triangle {
    vertex0: [f32; 3],
    vertex1: [f32; 3],
    vertex2: [f32; 3],
}

impl Primitive for Triangle {
    fn center(&self) -> [f32; 3] {
        let mut result = [0.0; 3];
        for i in 0..3 {
            result[i] = (self.vertex0[i] + self.vertex1[i] + self.vertex2[i]) / 3.0;
        }
        result
    }

    fn aabb(&self) -> crate::AABB {
        let mut aabb = crate::AABB::new();
        aabb.grow(self.vertex0);
        aabb.grow(self.vertex1);
        aabb.grow(self.vertex2);
        aabb
    }
}

impl SpatialTriangle for Triangle {
    fn vertex0(&self) -> [f32; 3] {
        self.vertex0
    }

    fn vertex1(&self) -> [f32; 3] {
        self.vertex1
    }

    fn vertex2(&self) -> [f32; 3] {
        self.vertex2
    }
}

let vertices: Vec<[f32; 3]> = vec![
    [-1.0, -1.0, 0.0],
    [1.0, -1.0, 0.0],
    [1.0, 1.0, 0.0],
    [-1.0, 1.0, 0.0],
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

let aabbs = primitives.iter().map(|t| t.aabb()).collect::<Vec<AABB>>();

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