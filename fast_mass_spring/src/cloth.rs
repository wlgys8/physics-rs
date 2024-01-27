use simulation::Mesh;

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
