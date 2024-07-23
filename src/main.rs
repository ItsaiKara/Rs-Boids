extern crate piston;
extern crate graphics;
extern crate glutin_window;
extern crate opengl_graphics;
extern crate rand;

use opengl_graphics::TextureSettings;
use rand::Rng;
use std::collections::LinkedList;
use piston::window::WindowSettings;
use piston::event_loop::*;
use piston::input::*;
use glutin_window::GlutinWindow;
use opengl_graphics::{GlGraphics, OpenGL, GlyphCache, Filter};
use graphics::{color};

const WIDTH: f64 = 1700.0;
const HEIGHT: f64 = 800.0;
const STEERING_FACTOR: f64 = 5.;

const BOID_SIZE: f64 = 10.0;
const MAX_BOID_SPEED: f64 = 10.0;
const NB_BOIDS: i32 = 100;


/**
 * The size of the flock (the radius around the boid in which other boids are considered neighbors)
 */
const FLOCK_SIZE: f64 = 170.;
const WEIGHT_COHESION: f64 = 0.3;

/**
 * The distance at which the boid will start to avoid colliding with other boids
 */
const SEPARATION_RADIUS: f64 = 20.0;
const WEIGHT_SEPARATION: f64 = 0.9;

/**
 * The distance at which the boid will start to align with other boids
 */
const ALIGNMENT_RADIUS: f64 = 200.;
const WEIGHT_ALIGNMENT: f64 = 0.7;

const FRAME_RATE: u64 = 24;
const VELCIRAPTOR_SPEED: f64 = 50.;

const MAX_OMEGA : f64 = 0.0;

const EDGE_DETECTION_DISTANCE: f64 = 50.0;

const FONT_DATA: &[u8] = include_bytes!("assets/FiraSans-Regular.ttf");

struct App {
    gl: GlGraphics,
    boids: LinkedList<Boid>,
    glyph_cache: GlyphCache<'static>,
    debug : bool,
    weight_cohesion : f64,
    weight_separation : f64,
    weight_alignment : f64,

    radius_cohesion : f64,
    radius_separation : f64,
    radius_alignment : f64,

    // 0 : cohesion
    // 1 : separation
    // 2 : alignment
    // True if edit weight mode is active
    // False if edit radius mode is active
    modes : [bool; 3]
}

impl App {
    fn render(&mut self, args: &RenderArgs, debug: bool) {
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

            let sim_args = [self.weight_cohesion, self.weight_separation, self.weight_alignment, self.radius_cohesion, self.radius_separation, self.radius_alignment];
            boid.render(&mut self.gl, args, debug, sim_args, self.modes);
        }

        self.gl.draw(args.viewport(), |c, gl| {
            let transform = c.transform.trans(10., 10.);
            let mut text = graphics::Text::new_color([1., 1.0, 1.0 ,1.], 11);
            let str = format!("Cohesion Weight :{} | Separation Weight :{} | Alignment Weight :{}  ", self.weight_cohesion, self.weight_separation, self.weight_alignment);

            text.draw(
                &str,
                &mut self.glyph_cache,
                &c.draw_state,
                transform,
                gl,
            ).unwrap();
        });

        self.gl.draw(args.viewport(), |c, gl| {
            let transform = c.transform.trans(10., 50.);
            let mut text = graphics::Text::new_color([1., 1.0, 1.0 ,1.], 11);
            let str = format!("Cohesion Radius :{} | Separation Radius :{} | Alignment Radius :{}  ", self.radius_cohesion, self.radius_separation, self.radius_alignment);

            text.draw(
                &str,
                &mut self.glyph_cache,
                &c.draw_state,
                transform,
                gl,
            ).unwrap();
        });

    }

    fn update(&mut self, _args: &UpdateArgs) {
        // C'est dégeulasse mais je ne sais pas comment faire autrement
        let dup_boids = self.boids.clone();
        
        // args for simulation update
        let sim_args = [self.weight_cohesion, self.weight_separation, self.weight_alignment, self.radius_cohesion, self.radius_separation, self.radius_alignment];

        for boid in &mut self.boids {
            boid.update(_args, dup_boids.clone(), sim_args, self.modes);
        }
    }


    fn print_boids(&self) {
        for boid in &self.boids {
            println!("x: {}, y: {}", boid.x, boid.y);
        }
    }

    fn reset(&mut self) {
        self.boids.clear();
        self.load_boids();
    }

    fn load_boids(&mut self) {
        for ct in 0..NB_BOIDS {
            let mut rng = rand::thread_rng();
            let x = rng.gen_range(0..WIDTH as i32);
            let y = rng.gen_range(0..HEIGHT as i32);
            let angle : f64 = rng.gen_range(0.0..2.0 * std::f64::consts::PI); 
    
            self.boids.push_back(Boid::new(ct, x as f64, y as f64, angle));
        }
    }

    fn init (&mut self) {
        self.load_boids();
    }

    fn print(&self) {
        println!("weight_cohesion : {}", self.weight_cohesion);
        println!("weight_separation : {}", self.weight_separation);
        println!("weight_alignment : {}", self.weight_alignment);
        println!("radius_cohesion : {}", self.radius_cohesion);
        println!("radius_separation : {}", self.radius_separation);
        println!("radius_alignment : {}", self.radius_alignment);
    }

    fn increase_value(&mut self, str : String, value : f64) {
        match str.as_str() {
            "cohesion" => {
                if self.modes[0] == true {
                    self.weight_cohesion += value;
                } else {
                    self.radius_cohesion += value*10.;
                }
            },
            "separation" => {
                if self.modes[1] == true {
                    self.weight_separation += value;
                } else {
                    self.radius_separation += value*10.;
                }
            },
            "alignment" => {
                if self.modes[2] == true {
                    self.weight_alignment += value;
                } else {
                    self.radius_alignment += value*10.;
                }
            },
            _ => (),
        }
    }

    fn decrease_value(&mut self, str : String, value : f64) {
        match str.as_str() {
            "cohesion" => {
                if self.modes[0] == true {
                    self.weight_cohesion -= value;
                } else {
                    self.radius_cohesion -= value*10.;
                }
            },
            "separation" => {
                if self.modes[1] == true {
                    self.weight_separation -= value;
                } else {
                    self.radius_separation -= value*10.;
                }
            },
            "alignment" => {
                if self.modes[2] == true {
                    self.weight_alignment -= value;
                } else {
                    self.radius_alignment -= value*10.;
                }
            },
            _ => (),
        }
    }

    fn toggle_mode(&mut self, str : String) {
        match str.as_str() {
            "cohesion" => {
                self.modes[0] = !self.modes[0];
            },
            "separation" => {
                self.modes[1] = !self.modes[1];
            },
            "alignment" => {
                self.modes[2] = !self.modes[2];
            },
            _ => (),
        }
    }
}


#[derive(Clone, Copy)] 
struct Boid {
    id : i32,
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
    com: [f64; 2],
    steer_away: [f64; 2],
    alignement_vector: [f64; 2],

    previous_angle: f64,
    unchanged_angle: i32,
}

impl Boid {
    fn new(i : i32 , x: f64, y: f64, angle : f64) -> Boid { 
        Boid {
            id : i ,
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
            //Center Of Mass
            com: [0.0, 0.0],
            //Point to steer away from
            steer_away: [0.0, 0.0],
            //Vector to align with
            alignement_vector: [0.0, 0.0],

            previous_angle: 0.0,
            unchanged_angle: 0,
        }
    }

    /**
     * Update the boid's position at each frame
     */
    fn update(&mut self, _args: &UpdateArgs, boids : LinkedList<Boid>, sim_args : [f64; 6], modes : [bool; 3]) {
        //update the boid's position
        self.update_distance_from_edges();
        self.wrap_around();

        let weight_cohesion = sim_args[0];
        let weight_separation = sim_args[1];
        let weight_alignment = sim_args[2];

        let flock_size = sim_args[3];
        let separation_radius = sim_args[4];    
        let alignment_radius = sim_args[5];

        
        // self.com  = self.get_center_of_mass(boids);
        // self.steer_away = self.get_steer_away(boids);
        
        self.com = self.build_rule_vectors(boids.clone(), "cohesion".to_string(), flock_size);
        self.steer_away = self.build_rule_vectors(boids.clone(), "separation".to_string(), separation_radius);
        self.alignement_vector = self.build_rule_vectors(boids.clone(), "alignment".to_string(), alignment_radius);
        
        let nb_neighbors_cohesion = self.get_nb_neighbors(boids.clone(), flock_size);
        let nb_neighbors_separation = self.get_nb_neighbors(boids.clone(), separation_radius);
        let nb_neighbors_alignement = self.get_nb_neighbors(boids.clone(), alignment_radius);

        if nb_neighbors_cohesion > 1 {
            self.update_cohesion();
        } else {
            self.cohesion = [0.0, 0.0];
        }

        if nb_neighbors_separation > 1 {
            self.update_separation();
        } else {
            self.separation = [0.0, 0.0];
        }

        if nb_neighbors_alignement > 1 {
            self.update_alignment();
        } else {
            self.alignment = [0.0, 0.0];
        }
        
        //update the boid's velocity based on the cohesion vector
        // self.velocity[0] += self.cohesion[0] * WEIGHT_COHESION;
        // self.velocity[1] += self.cohesion[1] * WEIGHT_COHESION;

        self.velocity[0] += self.cohesion[0] * weight_cohesion;
        
        // self.update_separation();
        
        self.velocity[0] -= self.separation[0] * weight_separation;
        self.velocity[1] -= self.separation[1] * weight_alignment;
        


        let mut omega_cohesion = self.smooth_angle(self.cohesion) * weight_cohesion;
        
        if omega_cohesion > MAX_OMEGA {
            omega_cohesion = MAX_OMEGA;
        } else if omega_cohesion < -MAX_OMEGA {
            omega_cohesion = -MAX_OMEGA;
        }
        {
            let tmp = self.velocity[0];
            self.velocity[0] += omega_cohesion * self.velocity[1];
            self.velocity[1] -= omega_cohesion * tmp;
        }

        let mut omega_separation = self.smooth_angle(self.separation) * weight_separation;

        if omega_separation > MAX_OMEGA {
            omega_separation = MAX_OMEGA;
        } else if omega_separation < -MAX_OMEGA {
            omega_separation = -MAX_OMEGA;
        }
        {
            let tmp = self.velocity[0];
            self.velocity[0] += omega_separation * self.velocity[1];
            self.velocity[1] -= omega_separation * tmp;
        }


        let mut  omega_alignement = self.smooth_angle(self.alignment) * weight_alignment;
        
        if omega_alignement > MAX_OMEGA {
            omega_alignement = MAX_OMEGA;
        } else if omega_alignement < -MAX_OMEGA {
            omega_alignement = -MAX_OMEGA;
        }
        
        let tmp = self.velocity[0];
        
        self.velocity[0] += omega_alignement * self.velocity[1];

        self.velocity[1] -= omega_alignement * tmp;
        


        self.enforce_max_speed();
    
        // update boid's position
        self.x += self.velocity[0];
        self.y += self.velocity[1];
        //update the boid's angle
        self.angle = self.velocity[1].atan2(self.velocity[0]);

        if self.velocity[0] == 0.0 && self.velocity[1] == 0.0 {
            self.unchanged_angle += 1;
        } else {
            self.unchanged_angle = 0;
        }
        self.dont_be_idiot();

    }   

    fn build_rule_vectors(&self, boids : LinkedList<Boid>, radius_name: String , radius: f64) -> [f64; 2] {
        let mut rule_vector = [0.0, 0.0];
        let mut nb_neighbors = 0;
        if radius_name == "separation" || radius_name == "cohesion" || radius_name == "alignment" {
            for boid in boids {
                if boid.id == self.id {
                    continue;
                }
                if (radius_name == "separation" || radius_name == "cohesion") && (boid.x - self.x).powi(2) + (boid.y - self.y).powi(2) < radius.powi(2) as f64 {
                    rule_vector[0] += boid.x;
                    rule_vector[1] += boid.y;
                    nb_neighbors += 1;
                } else if radius_name == "alignment" && (boid.x - self.x).powi(2) + (boid.y - self.y).powi(2) < radius.powi(2) as f64 {
                    rule_vector[0] += boid.velocity[0];
                    rule_vector[1] += boid.velocity[1];
                    nb_neighbors += 1;
                } else {
                    // panic!("Oops")
                }
            }
            if nb_neighbors > 0 {
                rule_vector[0] /= nb_neighbors as f64;
                rule_vector[1] /= nb_neighbors as f64;
            }
        }
        rule_vector
    }


    fn smooth_angle(&self, vector : [f64;2] ) -> f64 {
        const  LIMIT_THETA : f64=  std::f64::consts::PI / 2.;
        let mut omega = self.velocity[0] * vector[1] - self.velocity[1] * vector[0];
        let norm = (self.velocity[0].powi(2) + self.velocity[1].powi(2)).sqrt();
        omega /= norm;
        //chech if norm is 0 or omega is NaN
        if omega < 0.0 {
            omega = -1.0;
        } else {
            omega = 1.0;
        }
        
        // omega *= omega.min(LIMIT_THETA, (self.velocity[0] * self.alignement_vector[0] + self.velocity[1] * self.alignement_vector[1]).acos());
        let mut tmp = self.velocity[0] * self.alignement_vector[0] + self.velocity[1] * self.alignement_vector[1];
        tmp /= norm;
        if (tmp).acos() > LIMIT_THETA {
            tmp = LIMIT_THETA;
        }
        omega *= tmp.acos();
        // omega *= WEIGHT_ALIGNMENT / LIMIT_THETA;
        if norm == 0.0 || omega.is_nan() {
            return 0.0;
        } else {
            return omega
        }
    }

    fn get_nb_neighbors(&self, boids : LinkedList<Boid>, radius: f64) -> i32 {
        let mut nb_neighbors = 0;
        for boid in boids {
            if boid.id == self.id {
                continue;
            }
            if (boid.x - self.x).powi(2) + (boid.y - self.y).powi(2) < radius.powi(2) as f64 {
                nb_neighbors += 1;
            }
        }
        nb_neighbors
    }

    /**
     * Change the cohesion vector to point towards the center of mass of the flock
     */
    fn update_cohesion(&mut self) {
        //Update the cohesion vector to point towards the center of mass of the flock
        self.cohesion[0] = self.com[0] - self.x;
        self.cohesion[1] = self.com[1] - self.y;
        //normalize the cohesion vector
        let norm = (self.cohesion[0].powi(2) + self.cohesion[1].powi(2)).sqrt();
        self.cohesion[0] /= norm;
        self.cohesion[1] /= norm;    
    }

    fn update_separation(&mut self) {
        self.separation[0] = self.steer_away[0] - self.x;
        self.separation[1] = self.steer_away[1] - self.y;
        //normalize the cohesion vector
        let norm = (self.separation[0].powi(2) + self.separation[1].powi(2)).sqrt();
        self.separation[0] /= norm;
        self.separation[1] /= norm;
    }

    fn update_alignment(&mut self) {
        self.alignment[0] = self.alignement_vector[0] - self.velocity[0];
        self.alignment[1] = self.alignement_vector[1] - self.velocity[1];
        //normalize the cohesion vector
        let norm = (self.alignment[0].powi(2) + self.alignment[1].powi(2)).sqrt();
        self.alignment[0] /= norm;
        self.alignment[1] /= norm;
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

    fn wrap_around(&mut self) {
        if self.x > WIDTH {
            self.x = 0.0;
        } else if self.x < 0.0 {
            self.x = WIDTH;
        }

        if self.y > HEIGHT {
            self.y = 0.0;
        } else if self.y < 0.0 {
            self.y = HEIGHT;
        }
    }


    /**
     * 
     */
    fn avoid_edges(&mut self) -> f64 {
        let mut omega : f64 = 0.;
        for i in 0..4 {
            // const DEFAULT_OMEGA : f64 = 0.1;
            if self.distance_from_edges[i] < EDGE_DETECTION_DISTANCE {
                let mut N : [f64;2] ;
                match i {
                    0 => {
                        N = [0.,1.];
                    },
                    1 => {
                        N = [-1.,0.];
                        
                    },
                    2 => {
                        N = [0.,-1.];
                    },
                    3 => {
                        N = [1.,0.];
                    },
                    _ => (panic!("Merde")),
                }
                if (self.velocity[0] * N[1] - self.velocity[1] * N[0]) < 0.0 {
                    omega = -1.;
                } else {
                    omega = 1.;
                }
                omega *= 1.-(self.distance_from_edges[i] / EDGE_DETECTION_DISTANCE);
                omega *= STEERING_FACTOR;
                omega = omega / MAX_BOID_SPEED;
                // self.velocity[0] += -omega * self.velocity[1];
                // self.velocity[1] += omega * self.velocity[0];
            }
        }
        if omega > MAX_OMEGA {
            omega = MAX_OMEGA;
        } else if omega < -MAX_OMEGA {
            omega = -MAX_OMEGA;
        }
        omega
    }

    fn dont_be_idiot(&mut self) {
        if self.unchanged_angle > 2 {
            //slowlly bring velocity to 0
            self.velocity[0] *= 0.99;
            self.velocity[1] *= 0.99;
            println!("IAM AN IDIOT {} {} {} {} ", self.velocity[0], self.velocity[1], self.unchanged_angle, self.angle);
        }
    }


    fn render(&self, gl: &mut GlGraphics, args: &RenderArgs, debug: bool, sim_args : [f64; 6], modes : [bool; 3]) {
        use graphics::*; 

        let weight_cohesion = sim_args[0];
        let weight_separation = sim_args[1];
        let weight_alignment = sim_args[2];

        let flock_size = sim_args[3];
        let separation_radius = sim_args[4];
        let alignment_radius = sim_args[5];

        if debug {
            //draw the radius of the flock around the boid
            gl.draw(args.viewport(), |c, gl| {
                use graphics::ellipse;
                let transform = c.transform;
                let pink = [0.5, 0.0, 0.5, 0.3];
                let circle = ellipse::Ellipse::new(pink);
                let center = [self.x - flock_size/2., self.y- flock_size/2.];
                let radius = flock_size;
                //ofset the radius to make the circle fit the boid's position
                circle.draw([center[0], center[1], radius, radius], &c.draw_state, transform, gl);
            });
    
            //draw the radius of the separation around the boid
            gl.draw(args.viewport(), |c, gl| {
                use graphics::ellipse;
                let transform = c.transform;
                let red = [1.0, 0.0, 0.0, 0.3];
                let circle = ellipse::Ellipse::new(red);
                let center = [self.x - separation_radius/2., self.y- separation_radius/2.];
                let radius = separation_radius;
                //ofset the radius to make the circle fit the boid's position
                circle.draw([center[0], center[1], radius, radius], &c.draw_state, transform, gl);
            });
    
            //draw the radius of the alignment around the boid
            gl.draw(args.viewport(), |c, gl| {
                use graphics::ellipse;
                let transform = c.transform;
                let green = [0.1, 0.2, 0.0, 0.3];
                let circle = ellipse::Ellipse::new(green);
                let center = [self.x - alignment_radius/2., self.y- alignment_radius/2.];
                let radius = alignment_radius;
                //ofset the radius to make the circle fit the boid's position
                circle.draw([center[0], center[1], radius, radius], &c.draw_state, transform, gl);
            });
        }

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

        if (debug) {
            //draw a blue square on each boid.com
            gl.draw(args.viewport(), |c, gl| {
                let transform = c.transform;
                let blue = [0.0, 0.0, 1.0, 1.0];
                let square = rectangle::Rectangle::new(blue);
                let center = rectangle::centered_square(self.com[0], self.com[1], 5.0);
                square.draw(center, &c.draw_state, transform, gl);
            });

            //draw a green square on each boid.steer_away
            gl.draw(args.viewport(), |c, gl| {
                let transform = c.transform;
                let green = [0.0, 1.0, 0.0, 1.0];
                let square = rectangle::Rectangle::new(green);
                let center = rectangle::centered_square(self.steer_away[0], self.steer_away[1], 5.0);
                square.draw(center, &c.draw_state, transform, gl);
            });
        }

        
        

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

    let font_data: &[u8] = FONT_DATA;
    let glyph_cache = GlyphCache::from_bytes(font_data, (), TextureSettings::new().filter(Filter::Nearest)).unwrap();

    let mut app = App {
        gl: GlGraphics::new(opengl),
        boids: LinkedList::new(),
        glyph_cache: glyph_cache,
        debug : false,

        weight_cohesion : WEIGHT_COHESION,
        weight_separation : WEIGHT_SEPARATION,
        weight_alignment : WEIGHT_ALIGNMENT,

        radius_cohesion : FLOCK_SIZE,
        radius_separation : SEPARATION_RADIUS,
        radius_alignment : ALIGNMENT_RADIUS,

        modes : [false, false, false],
    };

    app.init();

    let mut events = Events::new(EventSettings::new()).ups(FRAME_RATE);

    while let Some(e) = events.next(&mut window) {
        if let Some(r) = e.render_args() {
            app.render(&r, app.debug);
        }

        if let Some(u) = e.update_args() {
            app.update(&u);
        }

        if let Some(Button::Keyboard(key)) = e.press_args() {
            match key {
                Key::Space => {
                    app.reset();
                },
                Key::D => {
                    app.debug = !app.debug;
                },
                Key::NumPad1 => {
                    app.decrease_value("cohesion".to_string(), 0.1);
                },
                Key::NumPad2 => {
                    app.toggle_mode("cohesion".to_string());
                },
                Key::NumPad3 => {
                    app.increase_value("cohesion".to_string(), 0.1);
                },
                Key::NumPad4 => {
                    app.decrease_value("separation".to_string(), 0.1);
                },
                Key::NumPad5 => {
                    app.toggle_mode("separation".to_string());
                },
                Key::NumPad6 => {
                    app.increase_value("separation".to_string(), 0.1);
                },
                Key::NumPad7 => {
                    app.decrease_value("alignment".to_string(), 0.1);
                },
                Key::NumPad8 => {
                    app.toggle_mode("alignment".to_string());
                },
                Key::NumPad9 => {
                    app.increase_value("alignment".to_string(), 0.1);
                },
                Key::I => {
                    app.print_boids();
                    app.print()
                },
                Key::R => {
                    app.weight_cohesion = WEIGHT_COHESION;
                    app.weight_separation = WEIGHT_SEPARATION;
                    app.weight_alignment = WEIGHT_ALIGNMENT;
                    app.radius_cohesion = FLOCK_SIZE;
                    app.radius_separation = SEPARATION_RADIUS;
                    app.radius_alignment = ALIGNMENT_RADIUS;
                },

                _ => (),
            }
        }
    }
}