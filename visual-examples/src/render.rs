use std::collections::HashSet;

use three_d::{
    vec3, Camera, Context, CpuMaterial, CpuMesh, Cull, ElementBuffer, InnerSpace, InstanceBuffer,
    Mat4, Matrix, PhysicalMaterial, Program, Quat, RenderStates, SquareMatrix, Srgba, Vector3,
    Vector4, VertexBuffer, Viewport, Zero,
};

pub struct ClothRender {
    positions: VertexBuffer,
    elements: ElementBuffer,
    program: Program,
    wireframe: RenderWireframe,
    indices: Vec<u32>,
    vertices: Vec<Vector3<f32>>,
    transform_dirty: bool,
}

impl ClothRender {
    pub fn new(context: &three_d::Context) -> Self {
        let positions = VertexBuffer::new(context);
        let elements = ElementBuffer::new(context);
        let program = Program::from_source(
            context,
            include_str!("shaders/triangle.vert"),
            include_str!("shaders/triangle.frag"),
        )
        .unwrap();
        Self {
            positions,
            elements,
            program,
            wireframe: RenderWireframe::new(context),
            indices: vec![],
            vertices: vec![],
            transform_dirty: false,
        }
    }

    pub fn set_indices(&mut self, indices: &[u32]) {
        self.indices.resize(indices.len(), 0);
        self.indices.copy_from_slice(indices);
        self.elements.fill(indices);
    }

    pub fn set_vertices_from_slice(&mut self, vertices: &[f32]) {
        assert_eq!(vertices.len() % 3, 0);
        self.vertices.resize(vertices.len() / 3, Vector3::zero());
        for i in 0..vertices.len() / 3 {
            self.vertices[i] = vec3(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);
        }
        self.transform_dirty = true;
    }

    pub fn draw(&mut self, camera: &Camera, viewport: Viewport) {
        if self.transform_dirty {
            self.transform_dirty = false;
            self.positions.fill(&self.vertices);
            self.wireframe.set(&self.vertices, &self.indices, 0.003);
        }
        let program = &self.program;
        program.use_uniform("model", Mat4::identity());
        program.use_uniform("viewProjection", camera.projection() * camera.view());
        program.use_uniform("color", vec3(1.0, 0.0, 0.0));
        program.use_vertex_attribute("position", &self.positions);
        program.draw_elements(RenderStates::default(), viewport, &self.elements);

        self.wireframe.draw(camera, viewport);
    }
}

struct RenderWireframe {
    vertices: VertexBuffer,
    indices: ElementBuffer,
    program: Program,
    transform_row1: Vec<Vector4<f32>>,
    transform_row2: Vec<Vector4<f32>>,
    transform_row3: Vec<Vector4<f32>>,
    transform_row1_buffer: InstanceBuffer,
    transform_row2_buffer: InstanceBuffer,
    transform_row3_buffer: InstanceBuffer,
}

impl RenderWireframe {
    pub fn new(context: &Context) -> Self {
        let program = Program::from_source(
            context,
            include_str!("shaders/wireframe.vert"),
            include_str!("shaders/triangle.frag"),
        )
        .unwrap();

        let mut wireframe_material = PhysicalMaterial::new_opaque(
            context,
            &CpuMaterial {
                albedo: Srgba::new_opaque(220, 50, 50),
                roughness: 0.7,
                metallic: 0.8,
                ..Default::default()
            },
        );
        wireframe_material.render_states.cull = Cull::Back;
        let cylinder = CpuMesh::cylinder(10);
        Self {
            vertices: VertexBuffer::new_with_data(context, &cylinder.positions.to_f32()),
            indices: ElementBuffer::new_with_data(context, &cylinder.indices.to_u32().unwrap()),
            program,
            transform_row1: vec![],
            transform_row2: vec![],
            transform_row3: vec![],
            transform_row1_buffer: InstanceBuffer::new(context),
            transform_row2_buffer: InstanceBuffer::new(context),
            transform_row3_buffer: InstanceBuffer::new(context),
        }
    }

    pub fn set(&mut self, vertices: &[Vector3<f32>], indices: &[u32], thickness: f32) {
        let mut edge_set = HashSet::new();
        self.transform_row1.clear();
        self.transform_row2.clear();
        self.transform_row3.clear();
        for triangle_index in 0..indices.len() / 3 {
            let i0 = indices[triangle_index * 3] as usize;
            let i1 = indices[triangle_index * 3 + 1] as usize;
            let i2 = indices[triangle_index * 3 + 2] as usize;
            let edges = [Edge::new(i0, i1), Edge::new(i1, i2), Edge::new(i2, i0)];
            for edge in edges {
                if !edge_set.contains(&edge) {
                    edge_set.insert(edge);
                    let transform = edge.compute_transform(vertices, thickness);
                    self.transform_row1.push(transform.row(0));
                    self.transform_row2.push(transform.row(1));
                    self.transform_row3.push(transform.row(2));
                }
            }
        }
        self.transform_row1_buffer.fill(&self.transform_row1[..]);
        self.transform_row2_buffer.fill(&self.transform_row2[..]);
        self.transform_row3_buffer.fill(&self.transform_row3[..]);
    }

    pub fn draw(&self, camera: &Camera, viewport: Viewport) {
        let instance_count = self.transform_row1.len() as u32;
        let program = &self.program;
        program.use_uniform("model", Mat4::identity());
        program.use_uniform("viewProjection", camera.projection() * camera.view());
        program.use_uniform("color", vec3(0.0, 0.0, 0.0));
        program.use_vertex_attribute("position", &self.vertices);
        program.use_instance_attribute("instanceTransformRow0", &self.transform_row1_buffer);
        program.use_instance_attribute("instanceTransformRow1", &self.transform_row2_buffer);
        program.use_instance_attribute("instanceTransformRow2", &self.transform_row3_buffer);
        program.draw_elements_instanced(
            RenderStates::default(),
            viewport,
            &self.indices,
            instance_count,
        );
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct Edge(usize, usize);

impl Edge {
    pub fn new(i0: usize, i1: usize) -> Self {
        if i0 < i1 {
            Self(i0, i1)
        } else {
            Self(i1, i0)
        }
    }

    pub fn compute_transform(&self, vertices: &[Vector3<f32>], thickness: f32) -> Mat4 {
        let v0 = vertices[self.0];
        let v1 = vertices[self.1];
        let v = v1 - v0;
        let rot = Quat::from_arc(vec3(1., 0., 0.), v1 - v0, None);
        let t = Mat4::from_translation(v0);
        let r = Mat4::from(rot);
        let s = Mat4::from_nonuniform_scale(v.magnitude(), thickness, thickness);
        t * r * s
    }
}
