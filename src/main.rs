#![allow(dead_code)]

extern crate lpsolve;
extern crate nalgebra;

use lpsolve::*;
use nalgebra::Vector2;

type Num = f64;
type V2 = Vector2<Num>;

struct Thruster {
  location: V2,
  direction: V2,
  max_thrust: Num,
}

enum Target {
  Nothing,
  //Stop(f32), // TODO: implement stopping on an axis.
  Zero,
  Positive,
  Negative,
}

fn main() {
  // Current situation
  let t0 = Thruster {
    location: V2::new(2.0, -2.0),
    direction: V2::new(0.0, 1.0),
    max_thrust: 100.0,
  };
  let t1 = Thruster {
    location: V2::new(-2.0, -2.0),
    direction: V2::new(0.0, 1.0),
    max_thrust: 100.0,
  };
  let t2 = Thruster {
    location: V2::new(2.0, -1.0),
    direction: V2::new(Num::cos(-45f64.to_radians()), Num::sin(-45f64.to_radians())),
    max_thrust: 10.0,
  };
  let t3 = Thruster {
    location: V2::new(-2.0, -1.0),
    direction: V2::new(Num::cos(225f64.to_radians()), Num::sin(225f64.to_radians())),
    max_thrust: 10.0,
  };
  let thrusters = vec![t0, t1, t2, t3];
  let num_thrusters = thrusters.len();

  // Target
  let x_target = Target::Zero;
  let y_target = Target::Positive;
  let angular_target = Target::Zero;

  let mut p = Problem::new(0, num_thrusters as i32).unwrap();
  p.set_verbose(Verbosity::Critical);

  let mut objective_function = vec![0.0; thrusters.len()];
  match x_target {
    Target::Nothing => {}
    Target::Zero => {
      // Add row indicating that thrusters may have no effect on movement in X.
      let x_effect: Vec<Num> = thrusters.iter().map(|t| t.direction.x).collect();
      let row = row(num_thrusters, x_effect).into_boxed_slice();
      p.add_constraint(&row, 0.0, ConstraintType::Eq);
    }
    Target::Positive => {
      // For each thruster, add effect of movement in X to objective function.
      for (i, thruster) in thrusters.iter().enumerate() {
        objective_function[i] += thruster.direction.x
      }
    }
    Target::Negative => {
      // For each thruster, add inverse of effect of movement in X to objective function.
      for (i, thruster) in thrusters.iter().enumerate() {
        objective_function[i] -= thruster.direction.x
      }
    }
  }
  match y_target {
    Target::Nothing => {}
    Target::Zero => {
      // Add row indicating that thrusters may have no effect on movement in Y.
      let y_effect: Vec<Num> = thrusters.iter().map(|t| t.direction.y).collect();
      let row = row(num_thrusters, y_effect).into_boxed_slice();
      p.add_constraint(&row, 0.0, ConstraintType::Eq);
    }
    Target::Positive => {
      // For each thruster, add effect of movement in Y to objective function.
      for (i, thruster) in thrusters.iter().enumerate() {
        objective_function[i] += thruster.direction.y
      }
    }
    Target::Negative => {
      // For each thruster, add inverse of effect of movement in Y to objective function.
      for (i, thruster) in thrusters.iter().enumerate() {
        objective_function[i] -= thruster.direction.y
      }
    }
  }
  match angular_target {
    Target::Nothing => {}
    Target::Zero => {
      // Add row indicating that it may have no effect on angular movement.
      let angular_effect: Vec<Num> = thrusters.iter().map(|t| t.direction.perp(&t.location)).collect();
      let row = row(num_thrusters, angular_effect).into_boxed_slice();
      p.add_constraint(&row, 0.0, ConstraintType::Eq);
    }
    Target::Positive => {
      // For each thruster, add effect of angular movement to objective function.
      for (i, thruster) in thrusters.iter().enumerate() {
        objective_function[i] += thruster.direction.perp(&thruster.location);
      }
    }
    Target::Negative => {
      // For each thruster, add inverse of angular movement to objective function.
      for (i, thruster) in thrusters.iter().enumerate() {
        objective_function[i] -= thruster.direction.perp(&thruster.location);
      }
    }
  }
  let objective_function_row = row(num_thrusters, objective_function);
  p.set_objective_function(&objective_function_row);
  p.set_maxim();

  // For each thruster, constrain thrust >= 0 and <= max_thrust.
  for (i, thruster) in thrusters.iter().enumerate() {
    // ti >= 0
    let mut vmin = vec![0.0; num_thrusters];
    vmin[i] = 1.0;
    let rmin = row(thrusters.len(), vmin).into_boxed_slice();
    p.add_constraint(&rmin, 0.0, ConstraintType::Ge);

    // ti <= MAX
    let mut vmax = vec![0.0; thrusters.len()];
    vmax[i] = 1.0;
    let rmax = row(thrusters.len(), vmax).into_boxed_slice();
    p.add_constraint(&rmax, thruster.max_thrust, ConstraintType::Le);
  }

  println!("Solving:");
  let status = p.solve();
  match status {
    SolveStatus::Optimal | SolveStatus::Suboptimal => {
      let mut result = vec![0.0; thrusters.len()];
      p.get_solution_variables(&mut result);
      let obj = p.get_objective();
      println!("Result: {:?} {:?} => {}", status, result, obj);
    }
    _ => {
      println!("Could not find a solution, or an error occurred: {:?}", status);
    }
  }
  println!("Done");
}

fn row<I>(columns: usize, values: I) -> Vec<f64>
  where I: IntoIterator<Item=f64> {
  // NOTE: Element 0 of each list of coefficients is ignored.
  // NOTE: Not putting the lists in separate variables or boxing them causes an unknown error,
  // probably because the data is not longer in memory after it has been borrowed.
  let mut r: Vec<f64> = vec![0.0f64; columns + 1];
  for (i, v) in values.into_iter().enumerate() {
    r[i + 1] = v;
  }
  return r;
}
