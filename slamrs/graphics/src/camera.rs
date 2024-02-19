use eframe::egui;
use nalgebra::{Isometry3, Matrix4, Orthographic3, Point2, Vector2, Vector3};

pub struct Camera {
    position: Vector2<f32>,
    zoom: f32,
    viewport_width: f32,
    viewport_height: f32,
    has_changed: bool,
    current_screen_size: egui::Vec2,
    // matrices for the Camera projection
    combined: Matrix4<f32>,
}

impl Camera {
    pub fn new() -> Self {
        Camera {
            position: Vector2::new(0.0, 0.0),
            zoom: 1.0,
            viewport_width: 1.0,
            viewport_height: 1.0,
            has_changed: true,
            current_screen_size: egui::Vec2::new(1.0, 1.0),

            combined: Matrix4::zeros(),
        }
    }

    pub fn pan(&mut self, screen_change: egui::Vec2) {
        if screen_change.x == 0.0 && screen_change.y == 0.0 {
            return;
        }

        let viewport_change = Vector2::new(
            screen_change.x / self.current_screen_size.x * self.viewport_width * self.zoom,
            screen_change.y / self.current_screen_size.y * self.viewport_height * self.zoom,
        );

        self.position += viewport_change;
        self.has_changed = true;
    }

    pub fn resize(&mut self, new_size: egui::Vec2) {
        // only do something if the screen size has actually changed
        if new_size == self.current_screen_size {
            return;
        }

        self.current_screen_size = new_size;

        // recalculate the size of the viewport here
        self.viewport_width = 10.0;
        self.viewport_height =
            self.viewport_width * self.current_screen_size.y / self.current_screen_size.x;

        self.has_changed = true;
    }

    pub fn zoom(&mut self, factor: f32) {
        if factor == 1.0 {
            return;
        }

        self.zoom *= factor;

        // clamp zoom
        if self.zoom < 0.1 {
            self.zoom = 0.1;
        }

        self.has_changed = true;
    }

    pub fn unproject(&self, screen_coord: egui::Pos2) -> Point2<f32> {
        // let r = self
        //     .combined
        //     .try_inverse()
        //     .unwrap()
        //     .transform_point(&Point3::new(screen_coord.x, screen_coord.y, 0.0));

        // dbg!(r);
        // r.xy()

        let mut v = Vector2::new(
            screen_coord.x / self.current_screen_size.x * self.viewport_width * self.zoom,
            (self.current_screen_size.y - screen_coord.y - 1.0) / self.current_screen_size.y
                * self.viewport_height
                * self.zoom,
        );

        // adjust for the viewport size
        v -= Vector2::new(
            self.viewport_width * self.zoom / 2.0,
            self.viewport_height * self.zoom / 2.0,
        );

        // adjust for the fact that the center of the screen is at "position"
        v -= self.position;

        Point2::new(v.x, v.y)
    }

    pub fn update(&mut self) {
        if !self.has_changed {
            return;
        }
        self.has_changed = true;

        // recreate the projection matrix
        let projection = Orthographic3::new(
            self.zoom * -self.viewport_width / 2.0,
            self.zoom * self.viewport_width / 2.0,
            self.zoom * -self.viewport_height / 2.0,
            self.zoom * self.viewport_height / 2.0,
            -1.0,
            1.0,
        );

        // recreate the view matrix containing the camera translation
        let view = Isometry3::new(
            Vector3::new(self.position.x, self.position.y, 0.0),
            nalgebra::zero(),
        );

        // calculate the combined transformation
        self.combined = projection.as_matrix() * view.to_homogeneous();
    }

    pub fn get_mvp(&self) -> Matrix4<f32> {
        self.combined
    }
}

impl Default for Camera {
    fn default() -> Self {
        Self::new()
    }
}
