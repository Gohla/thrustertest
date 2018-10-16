extern crate thrustertest;

use thrustertest::{Body, P2, Target, Thruster, V2};

fn main() {
  // Body
  let body = Body::new(
    10.0,
    P2::new(0.0, 0.0),
    vec![
      Thruster::new("T0", P2::new(2.0, -2.0), V2::new(0.0, 1.0), 0.0, 100.0),
      Thruster::new("T1", P2::new(-2.0, -2.0), V2::new(0.0, 1.0), 0.0, 100.0),
      Thruster::new("T2", P2::new(2.0, -1.0), V2::new(-0.5, -0.5), 0.0, 25.0),
      Thruster::new("T3", P2::new(-2.0, -1.0), V2::new(0.5, -0.5), 0.0, 50.0),
    ],
  );
  println!("{:?}", body);

  // Solve for Y movement
  body.solve(Target::Zero, Target::Positive, Target::Zero);
  body.solve(Target::Zero, Target::Negative, Target::Zero);
  body.solve(Target::Zero, Target::Stop { maximum_deceleration: 1.0 }, Target::Zero);
  body.solve(Target::Zero, Target::Stop { maximum_deceleration: -1.0 }, Target::Zero);
  // Solve for angular movement
  body.solve(Target::Zero, Target::Zero, Target::Positive);
  body.solve(Target::Zero, Target::Zero, Target::Negative);
  // Solve for X movement
  body.solve(Target::Positive, Target::Zero, Target::Zero);
  body.solve(Target::Negative, Target::Zero, Target::Zero);
  // Nothing
  body.solve(Target::Nothing, Target::Nothing, Target::Nothing);
  // Zero
  body.solve(Target::Zero, Target::Zero, Target::Zero);
}
