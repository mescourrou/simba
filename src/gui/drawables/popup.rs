pub struct Popup {
    title: String,
    description: String,
    buttons: Vec<String>,
    clicked_button: Option<usize>,
    action_on_click: Box<dyn FnMut(usize) -> ()>,
}

impl Popup {
    pub fn new(
        title: String,
        description: String,
        buttons: Vec<String>,
        action_on_click: fn(usize) -> (),
    ) -> Self {
        Self {
            title,
            description,
            buttons,
            clicked_button: None,
            action_on_click: Box::new(action_on_click),
        }
    }

    pub fn new_ok(title: String, description: String, action_on_click: fn(usize) -> ()) -> Self {
        Self {
            title,
            description,
            buttons: Vec::from(["OK".to_string()]),
            clicked_button: None,
            action_on_click: Box::new(action_on_click),
        }
    }

    pub fn new_ok_cancel(
        title: String,
        description: String,
        action_on_click: fn(usize) -> (),
    ) -> Self {
        Self {
            title,
            description,
            buttons: Vec::from(["OK".to_string(), "Cancel".to_string()]),
            clicked_button: None,
            action_on_click: Box::new(action_on_click),
        }
    }

    pub fn draw(&mut self, ctx: &egui::Context) -> Option<usize> {
        egui::Window::new(&self.title).show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.label(&self.description);

                ui.horizontal(|ui| {
                    for (i, btn) in self.buttons.iter().enumerate() {
                        if ui.button(btn).clicked() {
                            self.clicked_button = Some(i);
                        }
                    }
                });
            });
        });
        self.clicked_button
    }

    pub fn is_clicked(&mut self) -> bool {
        if let Some(i) = self.clicked_button {
            (self.action_on_click)(i);
            self.clicked_button = None;
            true
        } else {
            false
        }
    }
}
