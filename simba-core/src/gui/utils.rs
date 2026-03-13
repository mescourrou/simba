//! This module contains utility functions for the GUI, such as comboboxes, checkboxes, path editors, etc. These functions are used to create common UI elements in the GUI, and to avoid code duplication.

use std::{collections::BTreeMap, fmt::Debug, path::Path};

use egui::Color32;

use crate::utils::enum_tools::ToVec;

/// Combobox for an enum. It allows to select one value of the enum, and store it in a mutable reference.
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

/// Combobox for a list of strings. It allows to select one value of the list, and store it in a mutable reference.
pub fn string_combobox(
    ui: &mut egui::Ui,
    possible_values: &[&str],
    value: &mut String,
    id: impl std::hash::Hash,
) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(format!("{:?}", value))
        .show_ui(ui, |ui| {
            for s in possible_values {
                ui.selectable_value(value, s.to_string(), *s);
            }
        });
}

/// Checkbox list for a list of strings. It allows to select multiple values of the list, and store them in a vector.
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

/// Radio button list for an enum. It allows to select one value of the enum, and store it in a mutable reference.
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

/// Checkbox list for an enum. It allows to select multiple values of the enum, and store them in a vector.
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

/// Path editor. It is a text edit with an "Apply" button, that allows to edit a path string and apply it when the button is clicked.
/// 
/// Future: add a file explorer to select the path
pub fn path_finder(ui: &mut egui::Ui, path: &mut String, _base: &Path) {
    ui.text_edit_singleline(path);
}

/// Json editor with an "Apply" button. The json is only applied to the value when the button is clicked, otherwise it is stored in the buffer stack.
/// 
/// Future: add coloring and error highlighting in the editor, to make it easier to edit the json.
/// 
/// # Arguments
/// * `ui` - The egui UI to display the editor and the button.
/// * `buffer_key` - The key to use in the buffer stack to store the editor value. It should be unique for each editor to avoid conflicts.
/// * `error_key` - The key to use in the buffer stack to store the error message. It should be unique for each editor to avoid conflicts.
/// * `buffer_stack` - The buffer stack to store the editor value and the error message while "Apply" is not clicked.
/// * `value` - The value to update when "Apply" is clicked. It should be a mutable reference to the value to update.
pub fn json_config(
    ui: &mut egui::Ui,
    buffer_key: &String,
    error_key: &String,
    buffer_stack: &mut BTreeMap<String, String>,
    value: &mut serde_json::Value,
) {
    if !buffer_stack.contains_key(buffer_key) {
        buffer_stack.insert(buffer_key.clone(), value.to_string());
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

/// Text edit with an "Apply" button. The text is only applied to the value when the button is clicked, otherwise it is stored in the buffer stack.
/// 
/// # Arguments
/// * `ui` - The egui UI to display the text edit and the button.
/// * `buffer_key` - The key to use in the buffer stack to store the text edit value. It should be unique for each text edit to avoid conflicts.
/// * `buffer_stack` - The buffer stack to store the text edit value while "Apply" is not clicked.
/// * `value` - The value to update when "Apply" is clicked. It should be a mutable reference to the value to update.
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
