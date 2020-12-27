pub mod aabb;
pub mod builders;
pub mod bvh;
pub mod bvh_node;
pub mod mbvh_node;
pub mod morton;
pub mod ray;
mod utils;

pub use aabb::*;
pub use builders::*;
pub use bvh::*;
pub use bvh_node::*;
pub use mbvh_node::*;
pub use morton::*;
pub use ray::*;

#[cfg(test)]
mod tests {
    use crate::{spatial_sah::SpatialTriangle, Builder, Primitive, AABB};

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

    #[test]
    fn test_sah() {
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
        builder.construct_binned_sah();
    }

    #[test]
    fn test_locb() {
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
        builder.construct_locally_ordered_clustered();
    }

    #[test]
    fn test_spatial() {
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
        builder.construct_spatial_sah();
    }
}
