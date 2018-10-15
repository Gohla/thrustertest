#![allow(dead_code)]

//! A linear programming solver for thruster dynamics acting on a rigid body in 2D space.
//! We use the following units:
//!
//! * Distance : meters (m)
//! * Linear:
//!   * Mass         : kilogram (kg)
//!   * Force        : Newton (N)
//!   * Velocity     : meter per second (m/s)
//!   * Acceleration : meter per second squared (m/s^2)
//! * Angular:
//!   * Angular mass (moment of inertia, rotational inertia) : kilograms meter squared (kg m^2)
//!   * Angular force (torque, moment, moment of force)      : Newton meter (N m)
//!   * Angular velocity                                     : radian per second (rad/s)
//!   * Angular acceleration                                 : radian per second squared (rad/s^2)

extern crate lpsolve;
extern crate nalgebra;

use lpsolve::*;
use nalgebra::{Point2, Vector2};

type Num = f64;
type V2 = Vector2<Num>;
type P2 = Point2<Num>;

/// A rigid body in 2D space.
struct Body {
  /// The mass of the body, in kg.
  mass: f64,
  /// The center of mass of the body, in m.
  center_of_mass: P2,
  /// The thrusters of the body.
  thrusters: Vec<Thruster>,
}

/// A thruster on a rigid body in 2D space.
#[derive(Clone)]
struct Thruster {
  /// Name of the thruster.
  name: String,
  /// Position of the thruster on the body, in m.
  position: P2,
  /// Normalized direction in which acceleration is applied.
  direction: V2,
  /// Maximum force the thruster can produce, in N.
  max_thrust: Num,
}

impl Thruster {
  fn new(name: impl Into<String>, position: P2, direction: V2, max_thrust: Num) -> Self {
    Thruster { name: name.into(), position, direction: direction.normalize(), max_thrust }
  }
}

/// The calculated effects of a thruster on a rigid body in 2D space.
struct ThrusterEffect {
  /// The thruster this effect calculation is for.
  thruster: Thruster,
  /// How much linear acceleration in m/s^2 is applied to the x axis, when the thruster provides 1N of thrust.
  x_acceleration: Num,
  /// How much linear acceleration in m/s^2 is applied to the y axis, when the thruster provides 1N of thrust.
  y_acceleration: Num,
  /// How much angular acceleration in rad/s^2 is applied, when the thruster provides 1N of thrust.
  angular_acceleration: Num,
}

/// The command to execute for a certain axis, which accelerates on that axis. Linear acceleration
/// for linear axes, and angular acceleration for the angular axis. Negative acceleration is
/// deceleration.
#[derive(Debug)]
enum AccelerationCommand {
  /// Require nothing: do not care about acceleration on the axis.
  Nothing,
  /// Require zero acceleration on the axis.
  Zero,
  /// Maximize acceleration on the axis.
  Positive,
  /// Minimize (or maximize negative) acceleration on the axis.
  Negative,
// TODO: implement stopping.
//  /// Maximize deceleration on the axis to stop the body.
//  Stop {
//    /// Maximum deceleration that may be applied to stop the body.
//    maximum_deceleration: f64
//  }
}

fn main() {
  // Body
  let body = Body {
    mass: 10.0,
    center_of_mass: P2::new(0.0, 0.0),
    thrusters: vec![
      Thruster::new("T0", P2::new(2.0, -2.0), V2::new(0.0, 1.0), 100.0),
      Thruster::new("T1", P2::new(-2.0, -2.0), V2::new(0.0, 1.0), 100.0),
      Thruster::new("T2", P2::new(2.0, -1.0), V2::new(-0.5, -0.5), 25.0),
      Thruster::new("T3", P2::new(-2.0, -1.0), V2::new(0.5, -0.5), 50.0),
    ],
  };

  println!("Body setup");
  println!("Mass: {}kg", body.mass);
  println!("Center of mass: ({}, {})", body.center_of_mass.x, body.center_of_mass.y);
  for thruster in &body.thrusters {
    println!("Thruster {}:", thruster.name);
    println!("  location  : ({}, {})", thruster.position.x, thruster.position.y);
    println!("  direction : ({}, {})", thruster.direction.x, thruster.direction.y);
    println!("  max thrust: {}N", thruster.max_thrust);
  }

  // Calculated data
  let thruster_effects: Vec<_> = {
    let mass = body.mass;
    body.thrusters.iter().map(|t| {
      let x_acceleration = t.direction.x / mass;
      let y_acceleration = t.direction.y / mass;
      ThrusterEffect {
        x_acceleration,
        y_acceleration,
        // TODO: how to calculate the angular acceleration? should at least take mass and arm into account.
        angular_acceleration: t.direction.perp(&t.position.coords),
        thruster: t.clone(),
      }
    }).collect()
  };

  println!();
  println!("Thruster effects");
  for te in &thruster_effects {
    println!("Thruster {}:", te.thruster.name);
    println!("  x acceleration      : {}", te.x_acceleration);
    println!("  y acceleration      : {}", te.y_acceleration);
    println!("  angular acceleration: {}", te.angular_acceleration);
  }

  // Solve for Y movement
  solve(&thruster_effects, AccelerationCommand::Zero, AccelerationCommand::Positive, AccelerationCommand::Zero);
  solve(&thruster_effects, AccelerationCommand::Zero, AccelerationCommand::Negative, AccelerationCommand::Zero);
  // Solve for angular movement
  solve(&thruster_effects, AccelerationCommand::Zero, AccelerationCommand::Zero, AccelerationCommand::Positive);
  solve(&thruster_effects, AccelerationCommand::Zero, AccelerationCommand::Zero, AccelerationCommand::Negative);
  // Solve for X movement
  solve(&thruster_effects, AccelerationCommand::Positive, AccelerationCommand::Zero, AccelerationCommand::Zero);
  solve(&thruster_effects, AccelerationCommand::Negative, AccelerationCommand::Zero, AccelerationCommand::Zero);
  // Nothing
  solve(&thruster_effects, AccelerationCommand::Nothing, AccelerationCommand::Nothing, AccelerationCommand::Nothing);
  // Zero
  solve(&thruster_effects, AccelerationCommand::Zero, AccelerationCommand::Zero, AccelerationCommand::Zero);
}

fn solve(
  thruster_effects: &[ThrusterEffect],
  x_target: AccelerationCommand,
  y_target: AccelerationCommand,
  angular_target: AccelerationCommand,
) {
  println!();
  println!("Solving for x: {:?}, y: {:?}, angular: {:?}", x_target, y_target, angular_target);

  let num_thrusters = thruster_effects.len();

  let mut p = Problem::new(0, num_thrusters as i32).unwrap();
  p.set_verbose(Verbosity::Critical);

  let mut objective_function = vec![0.0; num_thrusters];
  match x_target {
    AccelerationCommand::Nothing => {}
    AccelerationCommand::Zero => {
      // Add row indicating that thrusters may have no effect on movement in X.
      let x_effects: Vec<_> = thruster_effects.iter().map(|td| td.x_acceleration).collect();
      let row = row(x_effects, num_thrusters).into_boxed_slice();
      p.add_constraint(&row, 0.0, ConstraintType::Eq);
    }
    AccelerationCommand::Positive => {
      // For each thruster, add effect of movement in X to objective function.
      for (i, te) in thruster_effects.iter().enumerate() {
        objective_function[i] += te.x_acceleration
      }
    }
    AccelerationCommand::Negative => {
      // For each thruster, add inverse of effect of movement in X to objective function.
      for (i, te) in thruster_effects.iter().enumerate() {
        objective_function[i] -= te.x_acceleration
      }
    }
  }
  match y_target {
    AccelerationCommand::Nothing => {}
    AccelerationCommand::Zero => {
      // Add row indicating that thrusters may have no effect on movement in Y.
      let y_effects: Vec<_> = thruster_effects.iter().map(|td| td.y_acceleration).collect();
      let row = row(y_effects, num_thrusters).into_boxed_slice();
      p.add_constraint(&row, 0.0, ConstraintType::Eq);
    }
    AccelerationCommand::Positive => {
      // For each thruster, add effect of movement in Y to objective function.
      for (i, te) in thruster_effects.iter().enumerate() {
        objective_function[i] += te.y_acceleration
      }
    }
    AccelerationCommand::Negative => {
      // For each thruster, add inverse of effect of movement in Y to objective function.
      for (i, te) in thruster_effects.iter().enumerate() {
        objective_function[i] -= te.y_acceleration
      }
    }
  }
  match angular_target {
    AccelerationCommand::Nothing => {}
    AccelerationCommand::Zero => {
      // Add row indicating that it may have no effect on angular movement.
      let angular_effects: Vec<_> = thruster_effects.iter().map(|td| td.angular_acceleration).collect();
      let row = row(angular_effects, num_thrusters).into_boxed_slice();
      p.add_constraint(&row, 0.0, ConstraintType::Eq);
    }
    AccelerationCommand::Positive => {
      // For each thruster, add effect of angular movement to objective function.
      for (i, te) in thruster_effects.iter().enumerate() {
        objective_function[i] += te.angular_acceleration;
      }
    }
    AccelerationCommand::Negative => {
      // For each thruster, add inverse of angular movement to objective function.
      for (i, te) in thruster_effects.iter().enumerate() {
        objective_function[i] -= te.angular_acceleration;
      }
    }
  }
  let objective_function_row = row(objective_function, num_thrusters);
  p.set_objective_function(&objective_function_row);
  p.set_maxim();

  // For each thruster, constrain thrust >= 0 and <= max_thrust.
  for (i, te) in thruster_effects.iter().enumerate() {
    // ti >= 0
    let mut vmin = vec![0.0; num_thrusters];
    vmin[i] = 1.0;
    let rmin = row(vmin, num_thrusters).into_boxed_slice();
    p.add_constraint(&rmin, 0.0, ConstraintType::Ge);

    // ti <= MAX
    let mut vmax = vec![0.0; num_thrusters];
    vmax[i] = 1.0;
    let rmax = row(vmax, num_thrusters).into_boxed_slice();
    p.add_constraint(&rmax, te.thruster.max_thrust, ConstraintType::Le);
  }

  let status = p.solve();
  match status {
    SolveStatus::Optimal | SolveStatus::Suboptimal => {
      let mut result = vec![0.0; num_thrusters];
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

fn row<I>(values: I, columns: usize) -> Vec<f64>
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
