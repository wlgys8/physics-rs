use std::collections::HashSet;

use nalgebra::Point3;

type Vector3 = nalgebra::Vector3<f32>;
type Isometry3 = nalgebra::Isometry3<f32>;

pub struct Mesh {
    vertices: Vec<Vector3>,
    indices: Vec<u32>,
}

impl Mesh {
    #[inline]
    pub fn vertices(&self) -> &[Vector3] {
        &self.vertices
    }

    #[inline]
    pub fn indices(&self) -> &[u32] {
        &self.indices
    }

    /// Compute the edges of the mesh without duplicates.
    pub fn compute_edges(&self) -> Vec<Edge> {
        let mut edge_set = HashSet::new();
        let mut edge_vec = vec![];
        for triangle_index in 0..self.indices.len() / 3 {
            let i0 = self.indices[triangle_index * 3] as usize;
            let i1 = self.indices[triangle_index * 3 + 1] as usize;
            let i2 = self.indices[triangle_index * 3 + 2] as usize;
            let edges = [Edge::new(i0, i1), Edge::new(i1, i2), Edge::new(i2, i0)];
            for edge in edges {
                if !edge_set.contains(&edge) {
                    edge_set.insert(edge);
                    edge_vec.push(edge);
                }
            }
        }
        edge_vec
    }
}

#[derive(Hash, Eq, PartialEq, Copy, Clone, Debug)]
pub struct Edge(usize, usize);

impl Edge {
    pub fn new(i0: usize, i1: usize) -> Self {
        if i0 < i1 {
            Self(i0, i1)
        } else {
            Self(i1, i0)
        }
    }

    #[inline]
    pub fn v0(&self) -> usize {
        self.0
    }

    #[inline]
    pub fn v1(&self) -> usize {
        self.1
    }
}

pub struct GridPlaneBuilder {
    pub width: f32,
    pub height: f32,
    pub width_segments: usize,
    pub height_segments: usize,
    pub transform: Isometry3,
}

impl GridPlaneBuilder {
    pub fn new(width: f32, height: f32, width_segments: usize, height_segments: usize) -> Self {
        Self {
            width,
            height,
            width_segments,
            height_segments,
            transform: Isometry3::identity(),
        }
    }

    #[inline]
    pub fn with_transform(mut self, transform: Isometry3) -> Self {
        self.transform = transform;
        self
    }
}

impl GridPlaneBuilder {
    pub fn build(self) -> Mesh {
        let dx = self.width / self.width_segments as f32;
        let dy = self.height / self.height_segments as f32;
        let num_vertex_width = self.width_segments + 1;
        let num_vertex_height = self.height_segments + 1;
        let mut vertices = Vec::with_capacity(num_vertex_width * num_vertex_height);
        let mut indices = Vec::with_capacity(self.width_segments * self.height_segments * 6);
        for i in 0..num_vertex_width {
            for j in 0..num_vertex_height {
                let x = i as f32 * dx - self.width / 2.0;
                let y = j as f32 * dy - self.height / 2.0;
                let z = 0.0;
                let vertex = self.transform * Point3::new(x, y, z);
                vertices.push(vertex.coords);
            }
        }

        for i in 0..self.width_segments {
            for j in 0..self.height_segments {
                let i0 = i * num_vertex_height + j;
                let i1 = i0 + 1;
                let i2 = i0 + num_vertex_height;
                let i3 = i2 + 1;
                indices.push(i0 as u32);
                indices.push(i2 as u32);
                indices.push(i1 as u32);
                indices.push(i1 as u32);
                indices.push(i2 as u32);
                indices.push(i3 as u32);
            }
        }

        Mesh { vertices, indices }
    }

    pub fn down_left_vertex_index(&self) -> usize {
        0
    }

    pub fn top_left_vertex_index(&self) -> usize {
        self.height_segments
    }

    pub fn down_right_vertex_index(&self) -> usize {
        self.width_segments * (self.height_segments + 1)
    }

    pub fn top_right_vertex_index(&self) -> usize {
        self.width_segments * (self.height_segments + 1) + self.height_segments
    }
}
