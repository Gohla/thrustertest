#![allow(dead_code)]

//! A linear programming solver for thruster dynamics acting on a rigid body in 2D space.
//! We use the following units:
//!
//! * Distance : meters (m)
//! * Linear:
//!   * Force        : Newton (N)
//!   * Mass         : kilogram (kg)
//!   * Acceleration : meter per second squared (m/s^2)
//! * Angular:
//!   * Angular force (torque, moment, moment of force)      : Newton meter (N m)
//!   * Angular mass (moment of inertia, rotational inertia) : kilograms meter squared (kg m^2)
//!   * Angular acceleration                                 : radian per second squared (rad/s^2)

extern crate lpsolve;
extern crate nalgebra;

use lpsolve::*;
use nalgebra::{Point2, Vector2};
use std::fmt;

pub type Num = f64;
pub type V2 = Vector2<Num>;
pub type P2 = Point2<Num>;

/// A rigid body in 2D space.
pub struct Body {
  /// The mass of the body, in kg.
  mass: f64,
  /// The center of mass of the body, in m.
  center_of_mass: P2,
  /// The thrusters of the body.
  thrusters: Vec<Thruster>,
  /// The effects of thrusters on the body.
  effects: Vec<Effect>,
}

impl Body {
  pub fn new(mass: f64, center_of_mass: P2, thrusters: impl Into<Vec<Thruster>>) -> Self {
    let thrusters = thrusters.into();
    let effects = thrusters.iter().map(|t| {
      let x_acceleration = t.force_vector.x / mass;
      let y_acceleration = t.force_vector.y / mass;
      let angular_acceleration = {
        let arm = t.position - center_of_mass;
        let torque = arm.perp(&t.force_vector); // Angular force.
        let arm_distance_squared = nalgebra::distance_squared(&t.position, &center_of_mass);
        let moment_of_inertia = mass * arm_distance_squared; // Angular mass.
        torque / moment_of_inertia
      };
      Effect {
        x_acceleration,
        y_acceleration,
        angular_acceleration,
        thruster: t.clone(),
      }
    }).collect();
    Body { mass, center_of_mass, thrusters, effects }
  }
}

/// A thruster on a rigid body in 2D space.
#[derive(Clone)]
pub struct Thruster {
  /// Name of the thruster.
  name: String,
  /// Position of the thruster on the body, in m.
  position: P2,
  /// Force vector (normalized) in which acceleration is applied.
  force_vector: V2,
  /// Minimum force the thruster can produce, in N. Must be >= 0.
  min_thrust: Num,
  /// Maximum force the thruster can produce, in N. Must be >= 1.
  max_thrust: Num,
}

impl Thruster {
  pub fn new(name: impl Into<String>, position: P2, force_vector: V2, min_thrust: Num, max_thrust: Num) -> Self {
    assert!(min_thrust >= 0.0, "Minimum thrust {} < 0.0", min_thrust);
    assert!(max_thrust >= 1.0, "Maximum thrust {} < 1.0", max_thrust);
    Thruster { name: name.into(), position, force_vector: force_vector.normalize(), min_thrust, max_thrust }
  }
}

/// The calculated effects of a thruster on a rigid body in 2D space.
pub struct Effect {
  /// The thruster this effect calculation is for.
  thruster: Thruster,
  /// How much linear acceleration in m/s^2 is applied to the x axis, when the thruster provides 1N of thrust.
  x_acceleration: Num,
  /// How much linear acceleration in m/s^2 is applied to the y axis, when the thruster provides 1N of thrust.
  y_acceleration: Num,
  /// How much angular acceleration in rad/s^2 is applied, when the thruster provides 1N of thrust.
  angular_acceleration: Num,
}

/// The target acceleration for a certain axis.
#[derive(Debug)]
pub enum Target {
  /// Require zero acceleration on the axis.
  Zero,
  /// Maximize acceleration on the axis, optionally limiting acceleration.
  Maximize(Option<f64>),
  /// Minimize (or maximize negative) acceleration on the axis, optionally limiting acceleration
  Minimize(Option<f64>),
}

impl Body {
  pub fn solve(
    &self,
    x_target: Option<Target>,
    y_target: Option<Target>,
    angular_target: Option<Target>,
  ) {
    println!();
    println!("Solving for x: {:?}, y: {:?}, angular: {:?}", x_target, y_target, angular_target);

    let num_thrusters = self.effects.len();

    let mut p = Problem::new(0, num_thrusters as i32).unwrap();
    p.set_verbose(Verbosity::Critical);

    // The objective function (row), with a column for each thruster indicating the sum of desired
    // accelerations on each axis, when the thruster provides 1N of thrust.
    // This function will be maximized (possibly constrained by several constraints).
    let mut objective_function = vec![0.0; num_thrusters];

    // Process x linear axis.
    if let Some(x_target) = x_target {
      match x_target {
        Target::Zero => {
          // Add constraint row: thrusters provide no linear acceleration on the x-axis.
          let x_effects: Vec<_> = self.effects.iter().map(|e| e.x_acceleration).collect();
          let row = row(x_effects, num_thrusters);
          p.add_constraint(&row, 0.0, ConstraintType::Eq);
        }
        Target::Maximize(max_accel) => {
          // For each thruster: add its x-acceleration to objective function.
          for (i, e) in self.effects.iter().enumerate() {
            objective_function[i] += e.x_acceleration
          }
          if let Some(max_accel) = max_accel {
            // Add constraint row: thrusters provide linear acceleration on the x-axis <= max_accel.
            let x_effects: Vec<_> = self.effects.iter().map(|e| e.x_acceleration).collect();
            let row = row(x_effects, num_thrusters);
            p.add_constraint(&row, max_accel, ConstraintType::Le);
          }
        }
        Target::Minimize(max_accel) => {
          // For each thruster: add inverse of its x-acceleration to objective function.
          for (i, e) in self.effects.iter().enumerate() {
            objective_function[i] -= e.x_acceleration
          }
          if let Some(max_accel) = max_accel {
            // Add constraint row: thrusters provide linear acceleration on the x-axis <= max_accel.
            let x_effects: Vec<_> = self.effects.iter().map(|e| -e.x_acceleration).collect();
            let row = row(x_effects, num_thrusters);
            p.add_constraint(&row, max_accel, ConstraintType::Le);
          }
        }
      }
    }

    // Process y linear axis
    if let Some(y_target) = y_target {
      match y_target {
        Target::Zero => {
          // Add constraint row: thrusters provide no linear acceleration on the y-axis.
          let y_effects: Vec<_> = self.effects.iter().map(|e| e.y_acceleration).collect();
          let row = row(y_effects, num_thrusters);
          p.add_constraint(&row, 0.0, ConstraintType::Eq);
        }
        Target::Maximize(max_accel) => {
          // For each thruster: add effect of movement in Y to objective function.
          for (i, e) in self.effects.iter().enumerate() {
            objective_function[i] += e.y_acceleration
          }
          if let Some(max_accel) = max_accel {
            // Add constraint row: thrusters provide linear acceleration on the y-axis <= max_accel.
            let y_effects: Vec<_> = self.effects.iter().map(|e| e.y_acceleration).collect();
            let row = row(y_effects, num_thrusters);
            p.add_constraint(&row, max_accel, ConstraintType::Le);
          }
        }
        Target::Minimize(max_accel) => {
          // For each thruster: add inverse of effect of movement in Y to objective function.
          for (i, e) in self.effects.iter().enumerate() {
            objective_function[i] -= e.y_acceleration
          }
          if let Some(max_accel) = max_accel {
            // Add constraint row: thrusters provide linear acceleration on the y-axis <= max_accel.
            let y_effects: Vec<_> = self.effects.iter().map(|e| -e.y_acceleration).collect();
            let row = row(y_effects, num_thrusters);
            p.add_constraint(&row, max_accel, ConstraintType::Le);
          }
        }
      }
    }

    // Process angular axis
    if let Some(angular_target) = angular_target {
      match angular_target {
        Target::Zero => {
          // Add constraint row: thrusters provide no angular acceleration.
          let angular_effects: Vec<_> = self.effects.iter().map(|e| e.angular_acceleration).collect();
          let row = row(angular_effects, num_thrusters);
          p.add_constraint(&row, 0.0, ConstraintType::Eq);
        }
        Target::Maximize(max_accel) => {
          // For each thruster: add effect of angular movement to objective function.
          for (i, e) in self.effects.iter().enumerate() {
            objective_function[i] += e.angular_acceleration;
          }
          if let Some(max_accel) = max_accel {
            // Add constraint row: thrusters provide angular acceleration <= max_accel.
            let angular_effects: Vec<_> = self.effects.iter().map(|e| e.angular_acceleration).collect();
            let row = row(angular_effects, num_thrusters);
            p.add_constraint(&row, max_accel, ConstraintType::Le);
          }
        }
        Target::Minimize(max_accel) => {
          // For each thruster: add inverse of angular movement to objective function.
          for (i, e) in self.effects.iter().enumerate() {
            objective_function[i] -= e.angular_acceleration;
          }
          if let Some(max_accel) = max_accel {
            // Add constraint row: thrusters provide angular acceleration <= max_accel.
            let angular_effects: Vec<_> = self.effects.iter().map(|e| -e.angular_acceleration).collect();
            let row = row(angular_effects, num_thrusters);
            p.add_constraint(&row, max_accel, ConstraintType::Le);
          }
        }
      }
    }

    // Move the objective function row to the solver, and tell it that we want to maximize.
    let objective_function_row = row(objective_function, num_thrusters);
    p.set_objective_function(&objective_function_row);
    p.set_maxim();

    // For each thruster, add two rows that constrains thrust >= min_thrust and <= max_thrust.
    for (i, e) in self.effects.iter().enumerate() {
      let mut vmin = vec![0.0; num_thrusters];
      vmin[i] = 1.0;
      let r_min = row(vmin, num_thrusters);
      p.add_constraint(&r_min, e.thruster.min_thrust, ConstraintType::Ge);

      let mut vmax = vec![0.0; num_thrusters];
      vmax[i] = 1.0;
      let r_max = row(vmax, num_thrusters);
      p.add_constraint(&r_max, e.thruster.max_thrust, ConstraintType::Le);
    }

    // Run the solver and print result.
    let status = p.solve();
    match status {
      SolveStatus::Optimal | SolveStatus::Suboptimal => {
        let mut result = vec![0.0; num_thrusters];
        p.get_solution_variables(&mut result);
        let obj = p.get_objective();
        println!("Result: {:?} {:?}N => {}(m and rad)/s^2", status, result, obj);
      }
      _ => {
        println!("Could not find a solution, or an error occurred: {:?}", status);
      }
    }
    println!("Done");
  }
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

impl fmt::Debug for Body {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    writeln!(f, "Body setup");
    writeln!(f, "Mass           : {}kg", self.mass);
    writeln!(f, "Center of mass : ({}m, {}m)", self.center_of_mass.x, self.center_of_mass.y);
    writeln!(f);
    for thruster in &self.thrusters {
      fmt::Debug::fmt(thruster, f)?;
    }
    writeln!(f);
    for effect in &self.effects {
      fmt::Debug::fmt(effect, f)?;
    }
    Ok(())
  }
}

impl fmt::Debug for Thruster {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    writeln!(f, "Thruster {}:", self.name);
    writeln!(f, "  location   : ({}m, {}m)", self.position.x, self.position.y);
    writeln!(f, "  direction  : ({}, {})", self.force_vector.x, self.force_vector.y);
    writeln!(f, "  min thrust : {}N", self.min_thrust);
    writeln!(f, "  max thrust : {}N", self.max_thrust);
    Ok(())
  }
}

impl fmt::Debug for Effect {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    writeln!(f, "Thruster {}:", self.thruster.name);
    writeln!(f, "  x acceleration       : {}m/s^2", self.x_acceleration);
    writeln!(f, "  y acceleration       : {}m/s^2", self.y_acceleration);
    writeln!(f, "  angular acceleration : {}rad/s^2", self.angular_acceleration);
    Ok(())
  }
}
