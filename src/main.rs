extern crate piston;
extern crate graphics;
extern crate glutin_window;
extern crate opengl_graphics;
extern crate rand;

use rand::Rng;
use std::collections::btree_map::Range;
use std::collections::LinkedList;
use std::iter::FromIterator;
use opengl_graphics::TextureSettings;
use piston::window::WindowSettings;
use piston::event_loop::*;
use piston::input::*;
use glutin_window::GlutinWindow;
use opengl_graphics::{GlGraphics, OpenGL, GlyphCache, Filter};
use graphics::{color};
use graphics::math::Vec2d;

const WIDTH: f64 = 800.0;
const HEIGHT: f64 = 800.0;

const BOID_SIZE: f64 = 10.0;
const MAX_BOID_SPEED: f64 = 10.0;
const NB_BOIDS: i32 = 100;

const FRAME_RATE: u64 = 24;
const VELCIRAPTOR_SPEED: f64 = 100.;


const EDGE_DETECTION_DISTANCE: f64 = 50.0;

struct Marker {
    x: f64,
    y: f64,
}

impl Marker {
    fn new(x: f64, y: f64) -> Marker {
        Marker {
            x: x,
            y: y,
        }
    }

    fn render(&self, gl: &mut GlGraphics, args: &RenderArgs) {
        use graphics::*;

        gl.draw(args.viewport(), |c, gl| {
            let transform = c.transform;
            let red = [1.0, 0.0, 0.0, 0.5];
            let square = rectangle::centered_square(self.x, self.y, 10.0);
            rectangle(red, square, transform, gl);
        });
    }

}


struct App {
    gl: GlGraphics,
    boids: LinkedList<Boid>,
    markers: LinkedList<Marker>,
}

impl App {
    fn render(&mut self, args: &RenderArgs) {
        use graphics::*;

        let BLACK: [f32; 4] = [0.0, 0.0, 0.0, 1.0];

        // Clear the screen
        self.gl.draw(args.viewport(), |_c, gl| {
            clear(BLACK, gl);
        });

        //draw blue lines around the edges of the screen
        self.gl.draw(args.viewport(), |c, gl| {
            let transform = c.transform;
            let blue = [0.0, 0.0, 1.0, 1.0];
            let line = line::Line::new(blue, 1.0);
            
            let vertices = [
                [0.0, 0.0, WIDTH, 0.0],
                [0.0, 0.0, 0.0, HEIGHT],
                [0.0, HEIGHT, WIDTH, HEIGHT],
                [WIDTH, 0.0, WIDTH, HEIGHT],
            ];

            for v in &vertices {
                line.draw(*v, &c.draw_state, transform, gl);
            }
        });

        //render each boid
        for boid in &self.boids {
            boid.render(&mut self.gl, args);
        }

        //render the markers
        for marker in &self.markers {
            marker.render(&mut self.gl, args);
        }
    }

    fn update(&mut self, _args: &UpdateArgs) {
        //update and render each boid
        for boid in &mut self.boids {
            boid.update(_args);
        }
    }


    fn print_boids(&self) {
        for boid in &self.boids {
            println!("x: {}, y: {}", boid.x, boid.y);
        }
    }
}

struct Boid {
    x: f64,
    y: f64,
    dx: f64,
    dy: f64,
    distance_from_edges : [f64; 4],
    // angle: f64,
    angle : f64,
    velocity: [f64; 2],
    separation: [f64; 2],
    alignment: [f64; 2],
    cohesion: [f64; 2],
}

impl Boid {
    fn new(x: f64, y: f64, angle : f64) -> Boid { 
        Boid {
            x: x,
            y: y,
            dx: 0.0,
            dy: 0.0,
            angle: angle,
            // velocity: [1.0, 1.0],
            distance_from_edges: [0.0, 0.0, 0.0, 0.0],
            velocity: [angle.cos() * VELCIRAPTOR_SPEED/FRAME_RATE as f64, angle.sin() * VELCIRAPTOR_SPEED/FRAME_RATE as f64],
            separation: [0.0, 0.0],
            alignment: [0.0, 0.0],
            cohesion: [0.0, 0.0],
        }
    }

    /**
     * Update the boid's position at each frame
     */
    fn update(&mut self, _args: &UpdateArgs) {
        //update the boid's position
        self.update_distance_from_edges();

        self.avoid_edges();
        self.enforce_max_speed();
        
        // update boid's position
        self.x += self.velocity[0];
        self.y += self.velocity[1];
        //update the boid's angle
        self.angle = self.velocity[1].atan2(self.velocity[0]);

    }    


    fn update_separation(&mut self) {
        //TODO
    }

    /**
     * Ensure that the boid's speed does not exceed the maximum speed
     */
    fn enforce_max_speed(&mut self) {
        let speed = (self.velocity[0].powi(2) + self.velocity[1].powi(2)).sqrt();
        if speed > MAX_BOID_SPEED {
            self.velocity[0] = self.velocity[0] * MAX_BOID_SPEED / speed;
            self.velocity[1] = self.velocity[1] * MAX_BOID_SPEED / speed;
        }
    }

    fn update_distance_from_edges(&mut self) {
        //calculate the distance from the edges of the screen
        // 0 : top
        // 1 : right
        // 2 : bottom
        // 3 : left
        self.distance_from_edges[0] = self.y;
        self.distance_from_edges[1] = WIDTH - self.x;
        self.distance_from_edges[2] = HEIGHT - self.y;
        self.distance_from_edges[3] = self.x;

        // get |dx| and |dy| to the edges
        self.distance_from_edges[0] = self.distance_from_edges[0].abs();
        self.distance_from_edges[1] = self.distance_from_edges[1].abs();
        self.distance_from_edges[2] = self.distance_from_edges[2].abs();
        self.distance_from_edges[3] = self.distance_from_edges[3].abs();
        // println!("distances: {:?}", self.distance_from_edges);
    }


    /**
     * Change dx and dy to avoid the edges of the screen
     */
    fn avoid_edges(&mut self) {
        for i in 0..4 {
            // const DEFAULT_OMEGA : f64 = 0.1;
            if self.distance_from_edges[i] < EDGE_DETECTION_DISTANCE {
                match i {
                    0 => {
                        let N : [f64;2] = [0.,1.];
                        let mut omega : f64 = 1.;
                        if (self.velocity[0] * N[1] - self.velocity[1] * N[0]) < 0.0 {
                            omega = -0.2;
                        } else {
                            omega = 0.2;
                        }
                        self.velocity[0] += -omega * self.velocity[1];
                        self.velocity[1] += omega * self.velocity[0];
                    },
                    1 => {
                        let N = [-1.,0.];
                        let mut omega : f64 = 1.;
                        if (self.velocity[0] * N[1] - self.velocity[1] * N[0]) < 0.0 {
                            omega = -0.2;
                        } else {
                            omega = 0.2;
                        }
                        self.velocity[0] += -omega * self.velocity[1];
                        self.velocity[1] += omega * self.velocity[0];
                    },
                    2 => {
                        let N = [0.,-1.];
                        let mut omega : f64 = 1.;
                        if (self.velocity[0] * N[1] - self.velocity[1] * N[0]) < 0.0 {
                            omega = -0.2;
                        } else {
                            omega = 0.2;
                        }
                        self.velocity[0] += -omega * self.velocity[1];
                        self.velocity[1] += omega * self.velocity[0];
                    },
                    3 => {
                        let N = [1.,0.];
                        let mut omega : f64 = 1.;
                        if (self.velocity[0] * N[1] - self.velocity[1] * N[0]) < 0.0 {
                            omega = -0.2;
                        } else {
                            omega = 0.2;
                        }
                        self.velocity[0] += -omega * self.velocity[1];
                        self.velocity[1] += omega * self.velocity[0];
                    },
                    _ => (),
                }
            }
        }
    }


    fn render(&self, gl: &mut GlGraphics, args: &RenderArgs) {
        use graphics::*;

        gl.draw(args.viewport(), |c, gl| {
            let transform = c.transform.trans(self.x, self.y);
            let white = [1.0, 1.0, 1.0, 1.0];
            let triangle = polygon::Polygon::new(white);
            // Define the vertices of the triangle relative to the boid's position
            let vertices = [
                [-5.0, -5.0],
                [5.0, 0.0],
                [-5.0, 5.0],
            ];
            // Apply rotation to the vertices based on the boid's angle
            let rotated_vertices = vertices.iter().map(|v| {
                let x = v[0] * self.angle.cos() - v[1] * self.angle.sin();
                let y = v[0] * self.angle.sin() + v[1] * self.angle.cos();
                [x, y]
            }).collect::<Vec<[f64; 2]>>();

            triangle.draw(&rotated_vertices, &c.draw_state, transform, gl);

            // println!("x: {}, y: {}, angle : {}", self.x, self.y, self.angle);

            //draw a red dot at the center of the boid
            let center = rectangle::centered_square(0.0, 0.0, 2.0);
            rectangle(color::RED, center, transform, gl);
            
        });
    }   

}


fn main() {
    let opengl = OpenGL::V3_2;

    let mut rng = rand::thread_rng();
    let mut window: GlutinWindow = WindowSettings::new(
        "Boids",
        [WIDTH, HEIGHT]
        )
        .exit_on_esc(true)
        .build()
        .unwrap();

    let mut app = App {
        gl: GlGraphics::new(opengl),
        boids: LinkedList::new(),
        markers: LinkedList::new(),
    };

    for _ in 0..NB_BOIDS {
        let mut rng = rand::thread_rng();
        let x = rng.gen_range(0..WIDTH as i32);
        let y = rng.gen_range(0..HEIGHT as i32);
        let angle : f64 = rng.gen_range(0.0..2.0 * std::f64::consts::PI); 

        app.boids.push_back(Boid::new(x as f64, y as f64, angle));
    }

    let mut events = Events::new(EventSettings::new()).ups(FRAME_RATE);

    while let Some(e) = events.next(&mut window) {
        if let Some(r) = e.render_args() {
            app.render(&r);
        }

        if let Some(u) = e.update_args() {
            app.update(&u);
        }
    }
}