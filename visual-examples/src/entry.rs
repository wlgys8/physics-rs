mod drop_cloth_demo;
mod hang_cloth_demo;

use std::{collections::VecDeque, time::Duration};

use simulation::FPSCounter;
use three_d::{Camera, FrameInput};

use crate::common::Demo;

use self::{drop_cloth_demo::DropClothDemo, hang_cloth_demo::HangClothDemo};

pub struct DemoEntry {
    gui: three_d::GUI,
    demos: Vec<Box<dyn Demo>>,
    selected_demo_index: Option<usize>,
    fps_counter: FPSCounter,
    stats: Stats,
}

impl DemoEntry {
    pub fn new(context: &three_d::Context) -> Self {
        let gui = three_d::GUI::new(context);
        let mut slf = Self {
            gui,
            demos: vec![],
            selected_demo_index: None,
            fps_counter: FPSCounter::default(),
            stats: Stats::default(),
        };
        slf.add_demo(HangClothDemo::default());
        slf.add_demo(DropClothDemo::default());
        slf
    }

    pub fn add_demo(&mut self, demo: impl Demo + 'static) {
        self.demos.push(Box::new(demo));
    }

    pub fn render_loop(
        &mut self,
        context: &three_d::Context,
        camera: &Camera,
        frame_input: &mut FrameInput,
    ) {
        if let Some(index) = self.selected_demo_index {
            let demo = &mut self.demos[index];
            let result = demo.on_frame_loop(camera, frame_input);
            if result.updated {
                self.stats.add_step_cost(result.step_cost);
            }
        }
        self.fps_counter.update();

        self.gui.update(
            &mut frame_input.events,
            frame_input.accumulated_time,
            frame_input.viewport,
            frame_input.device_pixel_ratio,
            |gui_context| {
                use three_d::egui::*;
                SidePanel::left("panel").show(gui_context, |ui| {
                    let select_text = self
                        .selected_demo_index
                        .map(|index| self.demos[index].name())
                        .unwrap_or("[Select a demo]");
                    ComboBox::from_label("")
                        .selected_text(select_text)
                        .show_ui(ui, |ui| {
                            for (index, demo) in self.demos.iter_mut().enumerate() {
                                let r = ui.selectable_value(
                                    &mut self.selected_demo_index,
                                    Some(index),
                                    demo.name(),
                                );
                                if r.changed() && Some(index) == self.selected_demo_index {
                                    demo.restart(context);
                                }
                            }
                        });

                    if let Some(index) = self.selected_demo_index {
                        self.demos[index].show_options_gui(ui);
                    }

                    if ui.button("restart").clicked() {
                        if let Some(index) = self.selected_demo_index {
                            self.demos[index].restart(context);
                        }
                    }
                });

                Area::new("screen_overlay").show(gui_context, |ui| {
                    ui.vertical(|ui| {
                        ui.colored_label(Rgba::BLACK, format!("fps: {}", self.fps_counter.fps()));
                        ui.colored_label(
                            Rgba::BLACK,
                            format!(
                                "step: {:.2} ms",
                                self.stats.avg_step_cost().as_secs_f64() * 1000.0
                            ),
                        );
                    });
                });
            },
        );
        frame_input.screen().write(|| {
            self.gui.render();
        });
    }
}

#[derive(Default)]
struct Stats {
    step_costs: VecDeque<Duration>,
    sum_step_costs: Duration,
}

impl Stats {
    pub fn add_step_cost(&mut self, cost: Duration) {
        if self.step_costs.len() == 1 {
            let first = self.step_costs.pop_front().unwrap();
            self.sum_step_costs -= first;
        }
        self.step_costs.push_back(cost);
        self.sum_step_costs += cost;
    }

    pub fn avg_step_cost(&self) -> Duration {
        if self.step_costs.is_empty() {
            return Duration::default();
        }
        self.sum_step_costs / self.step_costs.len() as u32
    }
}
