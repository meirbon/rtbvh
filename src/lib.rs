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
    use crate::{spatial_sah::SpatialTriangle, Aabb, Builder, Primitive};

    #[test]
    fn test_sah() {
        let vertices: [Vec4; 4] = [
            Vec4::new(-1.0, -1.0, 0.0, 1.0),
            Vec4::new(1.0, -1.0, 0.0, 1.0),
            Vec4::new(1.0, 1.0, 0.0, 1.0),
            Vec4::new(-1.0, 1.0, 0.0, 1.0),
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
        builder.construct_binned_sah();
    }

    #[test]
    fn test_locb() {
        let vertices: [Vec4; 4] = [
            Vec4::new(-1.0, -1.0, 0.0, 1.0),
            Vec4::new(1.0, -1.0, 0.0, 1.0),
            Vec4::new(1.0, 1.0, 0.0, 1.0),
            Vec4::new(-1.0, 1.0, 0.0, 1.0),
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
        builder.construct_locally_ordered_clustered();
    }

    #[test]
    fn test_spatial() {
        let vertices: [Vec4; 4] = [
            Vec4::new(-1.0, -1.0, 0.0, 1.0),
            Vec4::new(1.0, -1.0, 0.0, 1.0),
            Vec4::new(1.0, 1.0, 0.0, 1.0),
            Vec4::new(-1.0, 1.0, 0.0, 1.0),
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
        builder.construct_spatial_sah();
    }

    use glam::*;
    use l3d::prelude::*;

    #[derive(Debug, Copy, Clone)]
    pub struct Triangle {
        vertex0: Vec4,
        vertex1: Vec4,
        vertex2: Vec4,
    }

    impl Primitive for Triangle {
        fn center(&self) -> [f32; 3] {
            ((self.vertex0.xyz() + self.vertex1.xyz() + self.vertex2.xyz()) * (1.0 / 3.0)).into()
        }

        fn aabb(&self) -> Aabb {
            let mut aabb = Aabb::empty();
            aabb.grow(self.vertex0.xyz());
            aabb.grow(self.vertex1.xyz());
            aabb.grow(self.vertex2.xyz());
            aabb
        }
    }

    impl SpatialTriangle for Triangle {
        fn vertex0(&self) -> [f32; 3] {
            self.vertex0.xyz().into()
        }

        fn vertex1(&self) -> [f32; 3] {
            self.vertex1.xyz().into()
        }

        fn vertex2(&self) -> [f32; 3] {
            self.vertex2.xyz().into()
        }
    }

    pub(crate) fn load_teapot() -> (Vec<Aabb>, Vec<Triangle>) {
        use std::path::PathBuf;
        let loader = l3d::LoadInstance::new().with_default();
        let result = loader.load(LoadOptions {
            path: PathBuf::from("objects").join("teapot.obj"),
            ..Default::default()
        });

        let mesh = match result {
            LoadResult::Mesh(m) => m,
            _ => {
                let vertices: [Vec4; 4] = [
                    Vec4::new(-1.0, -1.0, 0.0, 1.0),
                    Vec4::new(1.0, -1.0, 0.0, 1.0),
                    Vec4::new(1.0, 1.0, 0.0, 1.0),
                    Vec4::new(-1.0, 1.0, 0.0, 1.0),
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
                return (aabbs, primitives);
            }
        };

        let primitives = mesh
            .vertices
            .chunks_exact(3)
            .map(|c| Triangle {
                vertex0: Vec4::from(c[0]),
                vertex1: Vec4::from(c[1]),
                vertex2: Vec4::from(c[2]),
            })
            .collect::<Vec<Triangle>>();
        let aabbs = primitives.iter().map(|t| t.aabb()).collect::<Vec<Aabb>>();

        (aabbs, primitives)
    }
}
