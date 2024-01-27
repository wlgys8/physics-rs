use std::ops::AddAssign;

use nalgebra::{point, Cholesky, Dyn, Matrix3};
use simulation::{Collider, TransformedCollider};

use crate::{
    cloth::Cloth,
    math::{DMatrix, DVector, Isometry3, Number, Vector3},
};

pub struct FastMassSpringSolver {
    cloth: Cloth,
    vector_d: DVector,              // size = 3 * numSprings
    h2_matrix_j: DMatrix,           // size = (3 * numParticles) x (3 * numSprings)
    matrix_m: DMatrix,              // size = (3 * numParticles) x (3 * numParticles)
    impulse_term: DVector,          // size = 3 * numParticles
    inertial_impluse_term: DVector, // size = 3 * numParticles
    time_step: Number,
    h2: Number,
    cholesky: Cholesky<Number, Dyn>,
    num_iterations: usize,
    damping: Number,
    colliders: Vec<TransformedCollider>,
}

impl FastMassSpringSolver {
    pub fn new(cloth: Cloth, time_step: Number) -> Self {
        let h2 = time_step * time_step;
        let num_constraints = cloth.num_constraints();
        let matrix_l = compute_matrix_l(&cloth); // size = (3 * numParticles) x (3 * numParticles)
        let matrix_j = compute_matrix_j(&cloth);
        let matrix_m = compute_matrix_m(&cloth);
        let system_matrix = &matrix_m + h2 * &matrix_l;
        let cholesky = nalgebra::linalg::Cholesky::new(system_matrix).unwrap();
        let impulse_term = DVector::zeros(cloth.num_particles() * 3);
        Self {
            vector_d: DVector::zeros(num_constraints * 3),
            h2_matrix_j: h2 * matrix_j,
            matrix_m,
            inertial_impluse_term: DVector::zeros(cloth.num_particles() * 3),
            cloth,
            time_step,
            h2,
            cholesky,
            impulse_term,
            num_iterations: 2,
            damping: 1.0,
            colliders: vec![],
        }
    }

    pub fn set_num_iterations(&mut self, num_iterations: usize) {
        self.num_iterations = num_iterations;
    }

    pub fn set_gravity(&mut self, gravity: Vector3) {
        for (i, &mass) in self.cloth.particle_masses.iter().enumerate() {
            self.impulse_term
                .fixed_rows_mut::<3>(i * 3)
                .copy_from(&(mass * gravity * self.h2));
        }
    }

    pub fn cloth(&self) -> &Cloth {
        &self.cloth
    }

    pub fn time_step(&self) -> Number {
        self.time_step
    }

    pub fn add_collider(&mut self, collider: impl Into<Collider>, transform: Isometry3) {
        self.colliders.push(TransformedCollider {
            collider: collider.into(),
            transform,
        });
    }

    pub fn step(&mut self) {
        self.pre_compute_terms();
        self.cloth
            .prev_particle_positions
            .copy_from(&self.cloth.particle_positions);

        for _ in 0..self.num_iterations {
            self.local_step();
            self.global_step();
        }

        self.solve_collision();
    }

    fn solve_collision(&mut self) {
        for collider in &self.colliders {
            for i in 0..self.cloth.num_particles() {
                let mut x = self.cloth.particle_positions.fixed_rows_mut::<3>(i * 3);
                let point = point![x[0], x[1], x[2]];
                if let Some(new_point) = collider.compute_collision_with_point(point) {
                    x.copy_from(&new_point.coords);
                }
            }
        }
    }

    fn pre_compute_terms(&mut self) {
        let damping = self.damping;
        let positions = &self.cloth.particle_positions;
        let prev_positions = &self.cloth.prev_particle_positions;
        // inertial_impluse_term = M * y + h^2 * f_ext
        self.inertial_impluse_term = &self.matrix_m
            * ((1.0 + damping) * positions - damping * prev_positions)
            + &self.impulse_term;
    }

    fn local_step(&mut self) {
        compute_vector_d(&self.cloth, &mut self.vector_d);
    }

    fn global_step(&mut self) {
        let b = &self.h2_matrix_j * &self.vector_d + &self.inertial_impluse_term;
        self.cloth.particle_positions = self.cholesky.solve(&b);
    }
}

fn compute_vector_d(cloth: &Cloth, vector_d: &mut DVector) {
    debug_assert!(vector_d.len() == cloth.num_constraints() * 3);

    let mut constraint_index = 0;

    for attachment in &cloth.attachments {
        let d = attachment.target_position;
        vector_d
            .fixed_rows_mut::<3>(constraint_index * 3)
            .copy_from(&d);
        constraint_index += 1;
    }

    for spring in &cloth.springs {
        let p0 = cloth
            .particle_positions
            .fixed_rows::<3>(spring.particle_index_0 * 3);
        let p1 = cloth
            .particle_positions
            .fixed_rows::<3>(spring.particle_index_1 * 3);
        let delta = p0 - p1;
        //compute the projection of delta onto the spring direction
        let d = delta.normalize() * spring.rest_length;
        vector_d
            .fixed_rows_mut::<3>(constraint_index * 3)
            .copy_from(&d);
        constraint_index += 1;
    }
}

/// calculate the matrix L in projective dynamics.
///
/// L = sum_i(k_i *A_i * A_i^T) ⊗ I_3
/// where
/// - A_i is a R^m matrix, m is the number of particles.
/// - I_3 is a 3x3 identity matrix.
/// - ⊗ is the Kronecker product.
/// - A_i is the incidence matrix of the i-th spring.
fn compute_matrix_l(cloth: &Cloth) -> DMatrix {
    let i3 = Matrix3::identity();
    let mut matrix_l = DMatrix::zeros(3 * cloth.num_particles(), 3 * cloth.num_particles());

    for attachment in &cloth.attachments {
        let k = attachment.stiffness;
        let i = attachment.particle_index;
        matrix_l
            .fixed_view_mut::<3, 3>(3 * i, 3 * i)
            .add_assign(&(k * i3));
    }

    for spring in &cloth.springs {
        let k = spring.stiffness;
        let i = spring.particle_index_0;
        let j = spring.particle_index_1;
        matrix_l
            .fixed_view_mut::<3, 3>(3 * i, 3 * i)
            .add_assign(&(k * i3));
        matrix_l
            .fixed_view_mut::<3, 3>(3 * j, 3 * j)
            .add_assign(&(k * i3));
        matrix_l
            .fixed_view_mut::<3, 3>(3 * i, 3 * j)
            .add_assign(-k * i3);
        matrix_l
            .fixed_view_mut::<3, 3>(3 * j, 3 * i)
            .add_assign(-k * i3);
    }
    matrix_l
}

fn compute_matrix_j(cloth: &Cloth) -> DMatrix {
    let i3 = Matrix3::identity();
    let mut matrix_j = DMatrix::zeros(3 * cloth.num_particles(), 3 * cloth.num_constraints());
    let mut constraint_index = 0;
    for attachment in cloth.attachments.iter() {
        let i = attachment.particle_index;
        let k = attachment.stiffness;
        matrix_j
            .fixed_view_mut::<3, 3>(3 * i, 3 * constraint_index)
            .copy_from(&(k * i3));
        constraint_index += 1;
    }

    for spring in cloth.springs.iter() {
        let i = spring.particle_index_0;
        let j = spring.particle_index_1;
        let k = spring.stiffness;
        matrix_j
            .fixed_view_mut::<3, 3>(3 * i, 3 * constraint_index)
            .copy_from(&(k * i3));
        matrix_j
            .fixed_view_mut::<3, 3>(3 * j, 3 * constraint_index)
            .copy_from(&(-k * i3));
        constraint_index += 1;
    }
    matrix_j
}

fn compute_matrix_m(cloth: &Cloth) -> DMatrix {
    let i3 = Matrix3::identity();
    let mut matrix_m = DMatrix::zeros(3 * cloth.num_particles(), 3 * cloth.num_particles());
    for (i, &mass) in cloth.particle_masses.iter().enumerate() {
        matrix_m
            .fixed_view_mut::<3, 3>(3 * i, 3 * i)
            .copy_from(&(mass * i3));
    }
    matrix_m
}
