mod utils;

use wasm_bindgen::prelude::*;
use cgmath::{Vector2};
use js_sys::Math::random;

// When the `wee_alloc` feature is enabled, use `wee_alloc` as the global
// allocator.
#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

#[wasm_bindgen]
extern {
    fn alert(s: &str);
}

#[wasm_bindgen]
pub fn greet() {
    alert("Hello, wasm-boids!");
}

#[derive(Debug, PartialEq, Clone)]
pub struct Boid {
    pub position: Vector2<f32>,
    pub velocity: Vector2<f32>,
    pub id: u32,
}

#[wasm_bindgen]
pub struct BoidOrchestrator {
    boids: Vec<Boids>,
    worldSize: Vector2<u32>,
    transferArray: Vec<f32>,
}


#[wasm_bindgen]
impl BoidOrchestrator {
    pub fn new(worldWidth: u32, worldHeight: u32, numBoids: u32) -> BoidOrchestrator {
        let worldSize = Vector2 {
            x: worldWidth,
            y: worldHeight,
        }
        for i in 0..numBoids {
            
            let boid = Boid {
                position: Vector2 {x: random()* worldSize.x, y: random()* worldSize.y}
                velocity: Vector2 {x: 0, y: 0};
                id: i;
            }
            transferArray.push(boid.position.x);
            transferArray.push(boid.position.y);
            transferArray.push(0); // TODO get angle calcualtion.  
            boids.insert(boid);
        }
    }

    pub fn tick(&self, dt: f32) {
        
        for (i, boid in boids.enumerate() {
            let newBoid = self.applyRules(boid, dt);
        }
    }
}




impl Universe {
    fn applyRules(&self,boid: &Boid, dt: f32) {
        let v1: Vector2<f32> = getVelToPercivedCenter(boid)
        //DO stuff
        boid
    }

    fn getVelToPercivedCenter(&self, boid: &Boid) {
        self.boids
    }
}