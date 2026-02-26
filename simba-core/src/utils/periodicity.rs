use std::sync::Arc;

use simba_macros::config_derives;

use crate::{
    config::NumberConfig,
    constants::TIME_ROUND,
    utils::{
        determinist_random_variable::{
            DeterministRandomVariable, DeterministRandomVariableFactory, RandomVariableTypeConfig,
        },
        distributions::fixed::{DeterministFixedRandomVariable, FixedRandomVariableConfig},
        maths::round_precision,
    },
};
#[cfg(feature = "gui")]
use crate::{constants::TIME_ROUND_DECIMALS, gui::UIComponent};

#[config_derives]
pub struct PeriodicityConfig {
    pub period: NumberConfig,
    pub offset: Option<NumberConfig>,
    pub table: Option<Vec<f32>>,
}

impl Default for PeriodicityConfig {
    fn default() -> Self {
        Self {
            period: NumberConfig::Num(1.0),
            offset: None,
            table: None,
        }
    }
}

#[cfg(feature = "gui")]
impl UIComponent for PeriodicityConfig {
    fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.label("Period:");
                match &self.period {
                    NumberConfig::Num(num) => {
                        ui.label(format!("{}", num));
                    }
                    NumberConfig::Rand(rand_config) => {
                        rand_config.show(ui, ctx, unique_id);
                    }
                }
            });
            ui.horizontal(|ui| {
                ui.label("Offset:");
                if let Some(offset) = &self.offset {
                    match offset {
                        NumberConfig::Num(num) => {
                            ui.label(format!("{}", num));
                        }
                        NumberConfig::Rand(rand_config) => {
                            rand_config.show(ui, ctx, unique_id);
                        }
                    }
                } else {
                    ui.label("Same as period");
                }
            });
            if let Some(table) = &self.table {
                ui.horizontal(|ui| {
                    ui.label("Table:");
                    for value in table {
                        ui.label(format!("{}", value));
                    }
                });
            }
        });
    }

    fn show_mut(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        buffer_stack: &mut std::collections::BTreeMap<String, String>,
        global_config: &crate::simulator::SimulatorConfig,
        current_node_name: Option<&String>,
        unique_id: &str,
    ) {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.label("Period:");
                match &mut self.period {
                    NumberConfig::Num(num) => {
                        ui.add(egui::DragValue::new(num).max_decimals(TIME_ROUND_DECIMALS));
                    }
                    NumberConfig::Rand(rand_config) => {
                        rand_config.show_mut(
                            ui,
                            ctx,
                            buffer_stack,
                            global_config,
                            current_node_name,
                            unique_id,
                        );
                    }
                }
                if ui.button("Switch number type").clicked() {
                    self.period = match &self.period {
                        NumberConfig::Num(_) => {
                            NumberConfig::Rand(RandomVariableTypeConfig::default())
                        }
                        NumberConfig::Rand(_) => NumberConfig::Num(1.),
                    }
                }
            });
            ui.horizontal(|ui| {
                ui.label("Offset:");
                if let Some(offset) = &mut self.offset {
                    match offset {
                        NumberConfig::Num(num) => {
                            ui.add(egui::DragValue::new(num).max_decimals(TIME_ROUND_DECIMALS));
                        }
                        NumberConfig::Rand(rand_config) => {
                            rand_config.show_mut(
                                ui,
                                ctx,
                                buffer_stack,
                                global_config,
                                current_node_name,
                                unique_id,
                            );
                        }
                    }
                    if ui.button("Switch number type").clicked() {
                        self.offset = match &self.offset {
                            Some(NumberConfig::Num(_)) => {
                                Some(NumberConfig::Rand(RandomVariableTypeConfig::default()))
                            }
                            Some(NumberConfig::Rand(_)) => Some(NumberConfig::Num(1.)),
                            None => None,
                        }
                    }
                } else {
                    ui.label("Same as period");
                    if ui.button("Set custom offset").clicked() {
                        self.offset = Some(self.period.clone());
                    }
                }
            });
            ui.horizontal(|ui| {
                ui.label("Table:");
                if let Some(table) = &mut self.table {
                    let mut to_remove = None;
                    for (i, value) in table.iter_mut().enumerate() {
                        ui.vertical(|ui| {
                            ui.add(egui::DragValue::new(value).max_decimals(TIME_ROUND_DECIMALS));
                            if ui.button("X").clicked() {
                                to_remove = Some(i);
                            }
                        });
                    }
                    if let Some(i) = to_remove {
                        table.remove(i);
                    }
                    if ui.button("Add").clicked() {
                        table.push(0.);
                    }
                } else {
                    ui.label("No table");
                    if ui.button("Add table").clicked() {
                        self.table = Some(vec![0., 1.]);
                    }
                }
            });
        });
    }
}

#[derive(Debug, Clone)]
pub struct Periodicity {
    period: DeterministRandomVariable,
    next_activation_time: f32,
    periodic_table: Option<Vec<f32>>,
    table_index: usize,
    table_offset: f32,
}

impl Periodicity {
    pub fn from_config(
        config: &PeriodicityConfig,
        va_factory: &DeterministRandomVariableFactory,
        initial_time: f32,
    ) -> Self {
        let period = match &config.period {
            NumberConfig::Num(num) => {
                assert!(*num > 0., "Periodicity period should be positive");
                DeterministRandomVariable::Fixed(DeterministFixedRandomVariable::from_config(
                    0.,
                    FixedRandomVariableConfig {
                        values: vec![round_precision(*num, TIME_ROUND).unwrap()],
                    },
                ))
            }
            NumberConfig::Rand(rand_config) => va_factory.make_variable(rand_config.clone()),
        };
        let offset = match &config.offset {
            Some(NumberConfig::Num(num)) => {
                assert!(*num >= 0., "Periodicity offset should be positive or null");
                round_precision(*num, TIME_ROUND).unwrap()
            }
            Some(NumberConfig::Rand(rand_config)) => round_precision(
                *va_factory
                    .make_variable(rand_config.clone())
                    .generate(0.)
                    .first()
                    .expect("Periodicity offset random variable should generate at least one value"),
                TIME_ROUND,
            )
            .unwrap(),
            None => round_precision(
                *period.generate(0.)
                    .first()
                    .expect("Periodicity period random variable should generate at least one value"),
                TIME_ROUND,
            )
            .unwrap(),
        };
        Self {
            period,
            next_activation_time: round_precision(initial_time + offset, TIME_ROUND).unwrap(),
            periodic_table: match &config.table {
                Some(table) => Some({
                    let mut table = table.clone();
                    table.sort_by(|a, b| {
                        a.partial_cmp(b)
                            .expect("Periodic table values should not be NaN")
                    });
                    assert!(
                        table.first().unwrap() >= &0.,
                        "Periodic table values should be positive or null"
                    );
                    table
                        .iter()
                        .map(|v| round_precision(*v, TIME_ROUND).unwrap())
                        .collect()
                }),
                None => None,
            },
            table_index: 0,
            table_offset: offset,
        }
    }

    pub fn next_time(&self) -> f32 {
        self.next_activation_time
    }

    pub fn update(&mut self, time: f32) {
        if (time - self.next_activation_time) >= -TIME_ROUND / 2. {
            if let Some(table) = &self.periodic_table {
                if self.table_index >= table.len() {
                    self.table_index = 0;
                    self.table_offset += self.period.generate(time)[0];
                }
                self.next_activation_time = round_precision(
                    (self.table_offset + table[self.table_index]).max(time + TIME_ROUND),
                    TIME_ROUND,
                )
                .unwrap();
                self.table_index = self.table_index + 1;
            } else {
                self.next_activation_time = round_precision(
                    time + self.period.generate(time)[0].max(TIME_ROUND),
                    TIME_ROUND,
                )
                .unwrap();
            }
        }
    }
}
