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
Single-threaded rays: 1.000.000 rays in 245.95 ms, 4.065867 million rays per second
16 threads rays: 1.000.000 rays in 29.846 ms, 33.505325 million rays per second
Single-threaded packets: 1.000.000 rays in 111.995 ms, 8.928969 million rays per second
16 threads packets: 1.000.000 rays in 10.669 ms, 93.7295 million rays per second
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