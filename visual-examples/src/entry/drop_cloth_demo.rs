use std::time::Instant;

use fast_mass_spring::{
    cloth::{Cloth, ClothBuilder},
    solver::FastMassSpringSolver,
};
use simulation::{math::Isometry3, FixedFrames, GridPlaneBuilder, SphereCollider};
use three_d::{
    AmbientLight, Camera, CpuMaterial, CpuMesh, DirectionalLight, FrameInput, Gm, PhysicalMaterial,
    Srgba,
};

use crate::{
    common::{ClothOptions, Demo, DemoLoopResult, SolverOptions},
    gui::{ClothOptionsGUI, SolverOptionsGUI},
    render::ClothRender,
};

pub struct DropClothScene {
    solver: FastMassSpringSolver,
    cloth_render: ClothRender,
    fixed_frames: FixedFrames,
    sphere_render: Gm<three_d::Mesh, PhysicalMaterial>,
    lights: Lights,
}

impl DropClothScene {
    fn new(context: &three_d::Context, scene_options: SceneOptions) -> Self {
        let solver_options = scene_options.solver_options;
        let mut render = ClothRender::new(context);
        let (cloth, mesh) = create_cloth(scene_options.cloth_options);
        render.set_indices(mesh.indices());

        let mut solver: FastMassSpringSolver =
            FastMassSpringSolver::new(cloth, solver_options.time_step);
        solver.set_num_iterations(solver_options.num_iterations);
        solver.set_gravity(solver_options.gravity);
        solver.add_collider(
            SphereCollider { radius: 1.0 },
            simulation::math::Isometry3::identity(),
        );

        let fixed_frame_generator = FixedFrames::new(solver_options.time_step);

        Self {
            solver,
            cloth_render: render,
            fixed_frames: fixed_frame_generator,
            sphere_render: create_sphere_render(context),
            lights: Lights::new(context),
        }
    }

    pub fn on_frame_loop(&mut self, camera: &Camera, frame_input: &FrameInput) -> DemoLoopResult {
        let mut step_count = 0;
        let time = Instant::now();
        for _ in self
            .fixed_frames
            .iter((frame_input.accumulated_time / 1000.0) as f32, 1)
        {
            self.solver.step();
            step_count += 1;
        }

        let result = if step_count > 0 {
            let step_cost = time.elapsed() / step_count;
            self.cloth_render
                .set_vertices_from_slice(self.solver.cloth().particle_positions.as_slice());
            DemoLoopResult {
                updated: true,
                step_cost,
            }
        } else {
            DemoLoopResult::not_updated()
        };

        frame_input
            .screen()
            .write(|| {
                self.cloth_render.draw(camera, frame_input.viewport);
            })
            .render(camera, [&self.sphere_render], &self.lights.array());
        result
    }
}

#[derive(Default)]
pub struct DropClothDemo {
    scene: Option<DropClothScene>,
    scene_options: SceneOptions,
}

impl Demo for DropClothDemo {
    fn name(&self) -> &'static str {
        "Drop Cloth"
    }

    fn restart(&mut self, context: &three_d::Context) {
        self.scene = Some(DropClothScene::new(context, self.scene_options));
    }

    fn on_frame_loop(&mut self, camera: &Camera, frame_input: &FrameInput) -> DemoLoopResult {
        if let Some(scene) = &mut self.scene {
            scene.on_frame_loop(camera, frame_input)
        } else {
            DemoLoopResult::not_updated()
        }
    }

    fn show_options_gui(&mut self, ui: &mut three_d::egui::Ui) {
        SolverOptionsGUI::new(&mut self.scene_options.solver_options).show_ui(ui);
        ClothOptionsGUI::new(&mut self.scene_options.cloth_options).show_ui(ui)
    }
}

fn create_cloth(options: ClothOptions) -> (Cloth, simulation::Mesh) {
    let resolution = options.resolution;
    let cloth_size = 4.0;
    let transform = Isometry3 {
        rotation: simulation::math::UnitQuaternion::from_axis_angle(
            &simulation::math::Vector3::x_axis(),
            std::f32::consts::PI / 2.0,
        ),
        translation: simulation::math::Vector3::new(0.0, 1.2, 0.0).into(),
    };
    let render_mesh_data =
        GridPlaneBuilder::new(cloth_size, cloth_size, resolution - 1, resolution - 1)
            .with_transform(transform)
            .build();

    let physics_cloth = ClothBuilder {
        size: cloth_size,
        resolution,
        structural_spring_stiffness: options.structual_spring_stiffness,
        shear_spring_stiffness: options.shear_spring_stiffness,
        mass: options.mass,
        transform,
    }
    .build();

    (physics_cloth, render_mesh_data)
}

fn create_sphere_render(context: &three_d::Context) -> Gm<three_d::Mesh, PhysicalMaterial> {
    use three_d::Mat4;
    let mut sphere = CpuMesh::sphere(64);
    sphere.transform(&Mat4::from_scale(0.95)).unwrap();
    Gm::new(
        three_d::Mesh::new(context, &sphere),
        PhysicalMaterial::new_opaque(
            context,
            &CpuMaterial {
                albedo: Srgba::BLUE,
                metallic: 1.0,
                ..Default::default()
            },
        ),
    )
}

#[derive(Clone, Copy)]
struct SceneOptions {
    solver_options: SolverOptions,
    cloth_options: ClothOptions,
}

impl Default for SceneOptions {
    fn default() -> Self {
        Self {
            solver_options: SolverOptions {
                time_step: 1.0 / 120.0,
                ..Default::default()
            },
            cloth_options: ClothOptions {
                structual_spring_stiffness: 80.0,
                shear_spring_stiffness: 0.2,
                ..Default::default()
            },
        }
    }
}

struct Lights {
    ambient: AmbientLight,
    directional: DirectionalLight,
}

impl Lights {
    pub fn new(context: &three_d::Context) -> Self {
        Self {
            ambient: AmbientLight::new(context, 0.4, Srgba::WHITE),
            directional: DirectionalLight::new(
                context,
                2.0,
                Srgba::WHITE,
                &three_d::vec3(-1.0, -1.0, -1.0),
            ),
        }
    }

    pub fn array(&self) -> [&dyn three_d::Light; 2] {
        [&self.ambient, &self.directional]
    }
}
