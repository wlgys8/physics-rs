use crate::common::{ClothOptions, PhysicsOptions};

#[derive(Default)]
pub struct PhysicsOptionsGUI {
    options: PhysicsOptions,
}

impl PhysicsOptionsGUI {
    pub fn show_ui(&mut self, ui: &mut three_d::egui::Ui) {
        use three_d::egui::*;
        CollapsingHeader::new("Physics Options").show(ui, |ui| {
            let mut physics_fps = (1.0 / self.options.time_step).ceil() as u32;
            let response = ui.add(
                Slider::new(&mut physics_fps, 30..=120)
                    .text("Steps/Sec")
                    .clamp_to_range(true),
            );
            if response.changed() {
                self.options.time_step = 1.0 / physics_fps as f32;
            }

            Slider::new(&mut self.options.num_iterations, 1..=10)
                .text("Num Iterations")
                .clamp_to_range(true)
                .ui(ui);
        });
    }

    pub fn options(&self) -> PhysicsOptions {
        self.options
    }
}

pub(crate) struct ClothOptionsGUI<'a> {
    pub data: &'a mut ClothOptions,
}

impl<'a> ClothOptionsGUI<'a> {
    pub fn new(data: &'a mut ClothOptions) -> Self {
        Self { data }
    }

    pub fn show_ui(&mut self, ui: &mut three_d::egui::Ui) {
        use three_d::egui::*;
        CollapsingHeader::new("Cloth Options").show(ui, |ui| {
            Slider::new(&mut self.data.spring_stiffness, 0.1..=100.0)
                .text("Spring Stiffness")
                .clamp_to_range(true)
                .ui(ui);
            Slider::new(&mut self.data.mass, 0.01..=100.0)
                .text("mass")
                .clamp_to_range(true)
                .ui(ui);
            Slider::new(&mut self.data.resolution, 2..=30)
                .text("Resolution")
                .clamp_to_range(true)
                .ui(ui);
        });
    }
}
