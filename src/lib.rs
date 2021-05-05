mod aabb;
mod builders;
mod bvh;
mod bvh_node;
mod iter;
mod mbvh_node;
mod morton;
mod ray;
mod utils;

pub use crate::bvh::*;
pub use aabb::*;
pub use builders::*;
pub use bvh_node::*;
pub use iter::*;
pub use mbvh_node::*;
pub use morton::*;
pub use ray::*;

#[cfg(test)]
mod tests {
    use crate::*;

    #[test]
    fn test_invalid_input() {
        let builder: Builder<Triangle> = Builder {
            aabbs: None,
            primitives: &[],
            primitives_per_leaf: None,
        };
        assert_eq!(
            builder.construct_binned_sah().unwrap_err(),
            BuildError::NoPrimitives
        );

        let builder: Builder<Triangle> = Builder {
            aabbs: None,
            primitives: &[Triangle {
                vertex0: Default::default(),
                vertex1: Default::default(),
                vertex2: Default::default(),
            }],
            primitives_per_leaf: None,
        };
        assert!(builder.construct_binned_sah().is_ok());

        let builder: Builder<Triangle> = Builder {
            aabbs: Some(&[]),
            primitives: &[Triangle {
                vertex0: Default::default(),
                vertex1: Default::default(),
                vertex2: Default::default(),
            }],
            primitives_per_leaf: None,
        };
        assert_eq!(
            builder.construct_binned_sah().unwrap_err(),
            BuildError::InequalAabbsAndPrimitives(0, 1)
        );
    }

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
            aabbs: Some(aabbs.as_slice()),
            primitives: primitives.as_slice(),
            primitives_per_leaf: None,
        };
        assert!(builder.construct_binned_sah().is_ok());
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
            aabbs: Some(aabbs.as_slice()),
            primitives: primitives.as_slice(),
            primitives_per_leaf: None,
        };
        assert!(builder.construct_locally_ordered_clustered().is_ok());
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
            aabbs: Some(aabbs.as_slice()),
            primitives: primitives.as_slice(),
            primitives_per_leaf: None,
        };
        assert!(builder.construct_spatial_sah().is_ok());
    }

    use glam::*;
    use l3d::prelude::*;

    #[derive(Debug, Copy, Clone)]
    pub struct Triangle {
        vertex0: Vec4,
        vertex1: Vec4,
        vertex2: Vec4,
    }

    impl Primitive<i32> for Triangle {
        fn center(&self) -> Vec3 {
            (self.vertex0.xyz() + self.vertex1.xyz() + self.vertex2.xyz()) * (1.0 / 3.0)
        }

        fn aabb(&self) -> Aabb<i32> {
            let mut aabb = Aabb::empty();
            aabb.grow(self.vertex0.xyz());
            aabb.grow(self.vertex1.xyz());
            aabb.grow(self.vertex2.xyz());
            aabb
        }
    }

    impl SpatialTriangle for Triangle {
        fn vertex0(&self) -> Vec3 {
            self.vertex0.xyz()
        }

        fn vertex1(&self) -> Vec3 {
            self.vertex1.xyz()
        }

        fn vertex2(&self) -> Vec3 {
            self.vertex2.xyz()
        }
    }

    pub(crate) fn load_teapot() -> (Vec<Aabb<i32>>, Vec<Triangle>) {
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
