#![allow(unused)]

use std::cmp::Ordering;

use nalgebra::{Point2, Vector2};

use graphics::{primitiverenderer::Color, shaperenderer::ShapeRenderer};

pub struct Ray {
    origin: Point2<f32>,
    direction: Vector2<f32>,
}

impl Ray {
    pub fn from_origin_direction(origin: Point2<f32>, direction: Vector2<f32>) -> Self {
        Self { origin, direction }
    }

    pub fn from_origin_angle(origin: Point2<f32>, angle: f32) -> Self {
        Self {
            origin,
            direction: Vector2::new(angle.cos(), angle.sin()),
        }
    }
}

pub trait Intersect {
    /// Returns the intersection between the object and the `Ray` as a
    /// length `u` along the `direction` of the ray such that the
    /// intersection point can be described by `ray.origin + u*ray.direction`,
    /// or `None` if no intersection occurs.
    fn intersect(&self, ray: &Ray) -> Option<f32>;
}

pub struct LineSegment {
    p1: Point2<f32>,
    p2: Point2<f32>,
}

impl LineSegment {
    // pub fn new(p1: Point2<f32>, p2: Point2<f32>) -> Self {
    //     Self { p1, p2 }
    // }

    pub fn new(x1: f32, y1: f32, x2: f32, y2: f32) -> Self {
        Self {
            p1: Point2::new(x1, y1),
            p2: Point2::new(x2, y2),
        }
    }
}

impl Intersect for LineSegment {
    fn intersect(&self, ray: &Ray) -> Option<f32> {
        // compute stuff!
        let x1 = self.p1.x;
        let y1 = self.p1.y;
        let x2 = self.p2.x;
        let y2 = self.p2.y;

        let x3 = ray.origin.x;
        let y3 = ray.origin.y;
        let x4 = ray.origin.x + ray.direction.x;
        let y4 = ray.origin.y + ray.direction.y;

        let denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        // make sure lines are not parallell
        if denom == 0.0 {
            return None;
        }

        let t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
        let u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;

        if 0.0 <= t && t <= 1.0 && u > 0.0 {
            return Some(u);
        }
        None
    }
}

pub trait Draw {
    fn draw(&self, r: &mut ShapeRenderer, color: Color);
}

impl Draw for LineSegment {
    fn draw(&self, r: &mut ShapeRenderer, color: Color) {
        r.line(self.p1.x, self.p1.y, self.p2.x, self.p2.y, color);
    }
}

pub trait SceneObject: Intersect + Draw {}
impl<T: Intersect + Draw> SceneObject for T {}
pub struct Scene {
    objects: Vec<Box<dyn SceneObject + Send + Sync>>,
}

impl Scene {
    pub fn new() -> Self {
        Self {
            objects: Vec::new(),
        }
    }

    pub fn add(&mut self, obj: Box<dyn SceneObject + Send + Sync>) -> &mut Self {
        self.objects.push(obj);
        self
    }

    pub fn add_rect(&mut self, origin: Point2<f32>, size: Vector2<f32>) -> &mut Self {
        self.add(Box::new(LineSegment::new(
            origin.x,
            origin.y,
            origin.x + size.x,
            origin.y,
        )))
        .add(Box::new(LineSegment::new(
            origin.x + size.x,
            origin.y,
            origin.x + size.x,
            origin.y + size.y,
        )))
        .add(Box::new(LineSegment::new(
            origin.x + size.x,
            origin.y + size.y,
            origin.x,
            origin.y + size.y,
        )))
        .add(Box::new(LineSegment::new(
            origin.x,
            origin.y + size.y,
            origin.x,
            origin.y,
        )))
    }
}

impl Draw for Scene {
    fn draw(&self, r: &mut ShapeRenderer, color: Color) {
        for o in &self.objects {
            o.draw(r, color);
        }
    }
}

impl Intersect for Scene {
    fn intersect(&self, ray: &Ray) -> Option<f32> {
        // keep lowest u value to only get closest intersection
        self.objects
            .iter()
            .filter_map(|o| o.intersect(ray))
            .min_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Less))
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn simple_intersection() {
        let ray = Ray {
            origin: Point2::new(0.0, 0.0),
            direction: Vector2::new(1.0, 1.0),
        };

        let line = LineSegment {
            p1: Point2::new(1.0, 2.0),
            p2: Point2::new(2.0, -2.0),
        };

        if let Some(u) = line.intersect(&ray) {
            let p = ray.origin + u * ray.direction;
            println!("Point of intersection: {p} (u={u})")
        } else {
            println!("No intersection!");
        }
    }

    #[test]
    fn scene_intersection() {
        let ray = Ray {
            origin: Point2::new(0.0, 0.0),
            direction: Vector2::new(1.0, 0.0),
        };

        let mut scene = Scene::new();
        scene
            .add(Box::new(LineSegment {
                p1: Point2::new(2.0, 2.0),
                p2: Point2::new(2.0, -2.0),
            }))
            .add(Box::new(LineSegment {
                p1: Point2::new(1.0, 2.0),
                p2: Point2::new(2.0, -2.0),
            }));

        if let Some(u) = scene.intersect(&ray) {
            let p = ray.origin + u * ray.direction;
            println!("Point of intersection: {p} (u={u})")
        } else {
            println!("No intersection!");
        }
    }
}
