use nalgebra::Point3;
use simulation::{math::Isometry3, Mesh};

use crate::math::{DVector, Number, Vector3};

pub struct Cloth {
    pub particle_masses: Vec<Number>,
    pub particle_positions: DVector,
    pub prev_particle_positions: DVector,
    pub springs: Vec<Spring>,
    pub attachments: Vec<Attachment>,
}

impl Cloth {
    pub fn from_slice(masses: &[Number], positions: &[Number]) -> Self {
        assert_eq!(masses.len() * 3, positions.len());
        let particle_masses = masses.to_vec();
        let particle_positions = DVector::from_row_slice(positions);
        let prev_particle_positions = particle_positions.clone();
        Cloth {
            particle_masses,
            particle_positions,
            prev_particle_positions,
            springs: vec![],
            attachments: vec![],
        }
    }

    pub fn add_springs(&mut self, springs: impl Iterator<Item = Spring>) {
        self.springs.extend(springs);
    }

    pub fn add_attachments(&mut self, attachments: impl IntoIterator<Item = Attachment>) {
        self.attachments.extend(attachments)
    }

    pub fn num_particles(&self) -> usize {
        self.particle_positions.len() / 3
    }

    pub fn num_springs(&self) -> usize {
        self.springs.len()
    }

    #[inline]
    pub fn num_constraints(&self) -> usize {
        self.num_springs() + self.attachments.len()
    }

    pub fn get_particle_position(&self, index: usize) -> Vector3 {
        let x = self.particle_positions[index * 3];
        let y = self.particle_positions[index * 3 + 1];
        let z = self.particle_positions[index * 3 + 2];
        Vector3::new(x, y, z)
    }
}

#[derive(Clone)]
pub struct Spring {
    pub particle_index_0: usize,
    pub particle_index_1: usize,
    pub stiffness: Number,
    pub rest_length: Number,
}

#[derive(Clone)]
pub struct Attachment {
    pub particle_index: usize,
    pub target_position: Vector3,
    pub stiffness: Number,
}

pub struct ClothFromMeshBuilder<'a> {
    pub mesh: &'a Mesh,
    pub mass: f32,
    pub spring_stiffness: f32,
}

impl<'a> ClothFromMeshBuilder<'a> {
    pub fn build(self) -> Cloth {
        let vertices = self.mesh.vertices();
        let num_particles = self.mesh.vertices().len();
        let mut particle_positions = Vec::with_capacity(num_particles * 3);
        particle_positions.extend(vertices.iter().flatten());
        let mut springs = vec![];
        let particle_mass = self.mass / num_particles as Number;
        let edges = self.mesh.compute_edges();
        for edge in edges {
            let index0 = edge.v0();
            let index1 = edge.v1();
            let stiffness = self.spring_stiffness;
            let p0 = vertices[index0];
            let p1 = vertices[index1];
            springs.push(Spring {
                particle_index_0: index0,
                particle_index_1: index1,
                stiffness,
                rest_length: (p0 - p1).magnitude(),
            });
        }
        let prev_particle_positions = particle_positions.clone();
        Cloth {
            particle_masses: vec![particle_mass; num_particles],
            particle_positions: DVector::from_vec(particle_positions),
            prev_particle_positions: DVector::from_vec(prev_particle_positions),
            springs,
            attachments: vec![],
        }
    }
}

/// Build a cloth that modeled as a grid plane. The order of the vertices is from -y to y, from -x to x.
pub struct ClothBuilder {
    pub size: Number,
    pub resolution: usize,
    pub structural_spring_stiffness: f32,
    pub shear_spring_stiffness: f32,
    pub mass: Number,
    pub transform: Isometry3,
}

impl ClothBuilder {
    pub fn build(self) -> Cloth {
        let resolution = self.resolution;
        let num_vertices = resolution * resolution;
        let mut vertices = Vec::with_capacity(num_vertices * 3);
        let cell_size = self.size / ((resolution as Number) - 1.0);
        for i in 0..resolution {
            for j in 0..resolution {
                let local_point = Point3::new(
                    -0.5 * self.size + i as Number * cell_size,
                    -0.5 * self.size + j as Number * cell_size,
                    0.0,
                );
                let point = self.transform * local_point;
                vertices.extend([point.x, point.y, point.z]);
            }
        }
        let particle_mass = self.mass / num_vertices as Number;

        let rest_length = |i: usize, j: usize| {
            let p0 = Vector3::from_column_slice(&vertices[i * 3..i * 3 + 3]);
            let p1 = Vector3::from_column_slice(&vertices[j * 3..j * 3 + 3]);
            (p0 - p1).magnitude()
        };

        //generate structural springs
        let mut springs = vec![];
        for i in 0..resolution {
            for j in 0..resolution {
                let index = i * resolution + j;
                if i + 1 < resolution {
                    let index1 = (i + 1) * resolution + j;
                    springs.push(Spring {
                        particle_index_0: index,
                        particle_index_1: index1,
                        stiffness: self.structural_spring_stiffness,
                        rest_length: rest_length(index, index1),
                    });
                }
                if j + 1 < resolution {
                    let index1 = i * resolution + j + 1;
                    springs.push(Spring {
                        particle_index_0: index,
                        particle_index_1: index1,
                        stiffness: self.structural_spring_stiffness,
                        rest_length: rest_length(index, index1),
                    });
                }
            }
        }

        //generate shear springs
        for i in 0..resolution {
            for j in 0..resolution {
                let index = i * resolution + j;
                if i + 1 < resolution && j + 1 < resolution {
                    let index1 = (i + 1) * resolution + j + 1;
                    springs.push(Spring {
                        particle_index_0: index,
                        particle_index_1: index1,
                        stiffness: self.shear_spring_stiffness,
                        rest_length: rest_length(index, index1),
                    });
                }
                if i + 1 < resolution && j > 0 {
                    let index1 = (i + 1) * resolution + j - 1;
                    springs.push(Spring {
                        particle_index_0: index,
                        particle_index_1: index1,
                        stiffness: self.shear_spring_stiffness,
                        rest_length: rest_length(index, index1),
                    });
                }
            }
        }
        Cloth {
            particle_masses: vec![particle_mass; num_vertices],
            particle_positions: DVector::from_vec(vertices.clone()),
            prev_particle_positions: DVector::from_vec(vertices),
            springs,
            attachments: vec![],
        }
    }

    pub fn down_left_vertex_index(&self) -> usize {
        0
    }

    pub fn top_left_vertex_index(&self) -> usize {
        self.resolution - 1
    }

    pub fn down_right_vertex_index(&self) -> usize {
        self.resolution * (self.resolution - 1)
    }

    pub fn top_right_vertex_index(&self) -> usize {
        self.resolution * self.resolution - 1
    }
}
