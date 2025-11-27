use std::{collections::BTreeMap, fmt::Debug, path::Path};

use egui::Color32;

use crate::utils::enum_tools::ToVec;

pub fn enum_combobox<EnumType>(ui: &mut egui::Ui, value: &mut EnumType, id: impl std::hash::Hash)
where
    EnumType: ToVec<&'static str> + ToVec<EnumType> + Debug + PartialEq,
{
    egui::ComboBox::from_id_salt(id)
        .selected_text(format!("{:?}", value))
        .show_ui(ui, |ui| {
            for (e, s) in std::iter::zip(
                <EnumType as ToVec<EnumType>>::to_vec(),
                <EnumType as ToVec<&str>>::to_vec(),
            ) {
                ui.selectable_value(value, e, s);
            }
        });
}

pub fn string_combobox(
    ui: &mut egui::Ui,
    possible_values: &Vec<String>,
    value: &mut String,
    id: impl std::hash::Hash,
) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(format!("{:?}", value))
        .show_ui(ui, |ui| {
            for s in possible_values {
                ui.selectable_value(value, s.clone(), s);
            }
        });
}

pub fn string_checkbox(
    ui: &mut egui::Ui,
    possible_values: &Vec<String>,
    mut_list: &mut Vec<String>,
) {
    let mut checkboxes_values = Vec::new();
    for v in possible_values {
        checkboxes_values.push((v.clone(), mut_list.contains(v)));
        ui.checkbox(&mut checkboxes_values.last_mut().unwrap().1, v.clone());
    }

    for (e, b) in checkboxes_values {
        if b {
            if !mut_list.contains(&e) {
                mut_list.push(e.clone());
            }
        } else if let Some(i) = mut_list.iter().rposition(|x| x == &e) {
            mut_list.remove(i);
        }
    }
}

pub fn enum_radio<EnumType>(ui: &mut egui::Ui, value: &mut EnumType)
where
    EnumType: ToVec<&'static str> + ToVec<EnumType> + Debug + PartialEq,
{
    for (e, s) in std::iter::zip(
        <EnumType as ToVec<EnumType>>::to_vec(),
        <EnumType as ToVec<&str>>::to_vec(),
    ) {
        ui.radio_value(value, e, s);
    }
}

pub fn enum_checkbox<EnumType>(ui: &mut egui::Ui, values: &mut Vec<EnumType>)
where
    EnumType: ToVec<&'static str> + ToVec<EnumType> + Debug + PartialEq + Clone,
{
    let mut checkboxes_values = Vec::new();
    for (e, s) in std::iter::zip(
        <EnumType as ToVec<EnumType>>::to_vec(),
        <EnumType as ToVec<&str>>::to_vec(),
    ) {
        checkboxes_values.push((e.clone(), values.contains(&e)));
        ui.checkbox(&mut checkboxes_values.last_mut().unwrap().1, s);
    }
    for (e, b) in checkboxes_values {
        if b {
            if !values.contains(&e) {
                values.push(e.clone());
            }
        } else if let Some(i) = values.iter().rposition(|x| x == &e) {
            values.remove(i);
        }
    }
}

pub fn path_finder(ui: &mut egui::Ui, path: &mut String, _base: &Path) {
    ui.text_edit_singleline(path);
}

pub fn json_config(
    ui: &mut egui::Ui,
    buffer_key: &String,
    error_key: &String,
    buffer_stack: &mut BTreeMap<String, String>,
    value: &mut serde_json::Value,
) {
    if !buffer_stack.contains_key(buffer_key) {
        buffer_stack.insert(buffer_key.clone(), serde_json::to_string(&value).unwrap());
    }
    ui.code_editor(buffer_stack.get_mut(buffer_key).unwrap());

    if let Some(error_msg) = buffer_stack.get(error_key) {
        ui.colored_label(Color32::RED, error_msg);
    }
    if ui.button("Apply").clicked() {
        match serde_json::from_str(buffer_stack.get(buffer_key).unwrap().as_str()) {
            Ok(p) => {
                *value = p;
                let _ = buffer_stack.remove(error_key);
                buffer_stack.remove(buffer_key);
            }
            Err(e) => {
                buffer_stack.insert(error_key.to_string(), e.to_string());
            }
        }
    }
}

pub fn text_singleline_with_apply(
    ui: &mut egui::Ui,
    buffer_key: &str,
    buffer_stack: &mut BTreeMap<String, String>,
    value: &mut String,
) {
    if !buffer_stack.contains_key(buffer_key) {
        buffer_stack.insert(buffer_key.to_string(), value.clone());
    }
    ui.text_edit_singleline(buffer_stack.get_mut(buffer_key).unwrap());

    if ui.button("Apply").clicked() {
        *value = buffer_stack.get(buffer_key).unwrap().clone();
        buffer_stack.remove(buffer_key);
    }
}

pub fn state_widget(_ui: &mut egui::Ui) {}
