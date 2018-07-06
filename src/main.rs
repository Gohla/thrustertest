extern crate lpsolve;
extern crate nalgebra;

use std::str;
use lpsolve::*;
use nalgebra::Vector2;

struct Thruster {
    location: Vector2<f32>,
    direction: Vector2<f32>,
    max_thrust: f32
}

fn main() {
    // Current situation
    let t0 = Thruster {
        location: Vector2::new(2.0, -2.0),
        direction: Vector2::new(0.0, 1.0),
        max_thrust: 100.0,
    };
    let t1 = Thruster {
        location: Vector2::new(-2.0, -2.0),
        direction: Vector2::new(0.0, 1.0),
        max_thrust: 100.0,
    };
    let t2 = Thruster {
        location: Vector2::new(2.0, -1.0),
        direction: Vector2::new(f32::cos(-45f32.to_radians()), f32::sin(-45f32.to_radians())),
        max_thrust: 10.0,
    };
    let t3 = Thruster {
        location: Vector2::new(-2.0, -1.0),
        direction: Vector2::new(f32::cos(225f32.to_radians()), f32::sin(225f32.to_radians())),
        max_thrust: 10.0,
    };
    let thrusters = vec!(t0, t1, t2, t3);

    // Desired direction and rotation
//    let target_direction = Vector2::new(0, 1);
//    let target_rotation = 20.0;

    // We calculate six situations:
    // - thrust in +X direction
    // - thrust in +Y direction
    // - thrust in -X direction
    // - thrust in -Y direction
    // - rotate clockwise
    // - rotate counter-clockwise
    // After that, any rotation or translation
    // is a weighted combination of these.

    let mut p = Problem::new(0, thrusters.len() as i32).unwrap();
    p.set_verbose(Verbosity::Critical);
    // p.set_verbose(Verbosity::Full);

    // t1 + t2 + t3 + ... >= 0
    let r0 = row(thrusters.len(), vec![1.0; thrusters.len()]);
    p.add_constraint(&r0, 0.0, ConstraintType::Ge);

    for (i, thruster) in thrusters.iter().enumerate() {
        // ti >= 0
        let mut vmin = vec![0.0; thrusters.len()];
        vmin[i] = 1.0;
        let rmin = row(thrusters.len(), vmin).into_boxed_slice();
        p.add_constraint(&rmin, 0.0, ConstraintType::Ge);

        // ti <= MAX
        let mut vmax = vec![0.0; thrusters.len()];
        vmax[i] = 1.0;
        let rmax = row(thrusters.len(), vmax).into_boxed_slice();
        p.add_constraint(&rmax, thruster.max_thrust as f64, ConstraintType::Le);
    }

    // t1 * d1.x + t2 * d2.x + t3 * d3.x + ... == 0
    let rx = row(thrusters.len(), thrusters.iter().map(|t| t.direction.x as f64));
    p.add_constraint(&rx, 0.0, ConstraintType::Eq);
//    p.set_objective_function(&rx);
//    p.set_maxim();

    // t1 * d1.y + t2 * d2.y + t3 * d3.y + ... == 0
    let ry = row(thrusters.len(), thrusters.iter().map(|t| t.direction.y as f64));
    p.add_constraint(&ry, 0.0, ConstraintType::Eq);
//    p.set_objective_function(&ry);
//    p.set_maxim();

    // ri is the vector from the center of mass to the center of the thruster.
    // Simply by having the center of mass at 0,0, we can take only the thruster location for ri.
    // t1 * c1 + t2 * c2 + t3 * c3 + ... == 0
    let rr = row(thrusters.len(), thrusters.iter().map(|t| t.direction.perp(&t.location) as f64));
    println!("{:?}", rr);
//    p.add_constraint(&rr, 0.0, ConstraintType::Eq);
    p.set_objective_function(&rr);
    p.set_maxim();
//    p.set_minim();

    println!("Solving:");
    let status = p.solve();
    match status {
        SolveStatus::Optimal | SolveStatus::Suboptimal => {
            let mut result = vec![0.0; thrusters.len()];
            p.get_solution_variables(&mut result);
            let obj = p.get_objective();
            println!("Result: {:?} {:?} => {}", status, result, obj);
        },
        _ => {
            println!("Could not find a solution, or an error occurred: {:?}", status);
        }
    }

    // Print MPS file:
    // println!("-------------------");
    // let mut buf = vec![0u8; 1024];
    // p.write_freemps(&mut buf);
    // println!("{}", str::from_utf8(&buf).unwrap());
    println!("Done");
}

fn row<I>(columns: usize, values: I) -> Vec<f64>
  where I: IntoIterator<Item = f64> {
    // NOTE: Element 0 of each list of coefficients is ignored.
    // NOTE: Not putting the lists in separate variables or boxing them causes an unknown error,
    // probably because the data is not longer in memory after it has been borrowed.
    let mut r: Vec<f64> = vec![0.0f64; columns + 1];
    for (i, v) in values.into_iter().enumerate() {
        r[i + 1] = v;
    }
    return r;
}
