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

  println!("Solve for linear x axis");
  body.solve(Some(Target::Maximize(None)), Some(Target::Zero), Some(Target::Zero));
  body.solve(Some(Target::Minimize(None)), Some(Target::Zero), Some(Target::Zero));
  body.solve(Some(Target::Maximize(Some(1.0))), Some(Target::Zero), Some(Target::Zero));
  body.solve(Some(Target::Minimize(Some(1.0))), Some(Target::Zero), Some(Target::Zero));
  println!();
  println!("Solve for linear y axis");
  body.solve(Some(Target::Zero), Some(Target::Maximize(None)), Some(Target::Zero));
  body.solve(Some(Target::Zero), Some(Target::Minimize(None)), Some(Target::Zero));
  body.solve(Some(Target::Zero), Some(Target::Maximize(Some(1.0))), Some(Target::Zero));
  body.solve(Some(Target::Zero), Some(Target::Minimize(Some(1.0))), Some(Target::Zero));
  println!();
  println!("Solve for angular axis");
  body.solve(Some(Target::Zero), Some(Target::Zero), Some(Target::Maximize(None)));
  body.solve(Some(Target::Zero), Some(Target::Zero), Some(Target::Minimize(None)));
  body.solve(Some(Target::Zero), Some(Target::Zero), Some(Target::Maximize(Some(0.5))));
  body.solve(Some(Target::Zero), Some(Target::Zero), Some(Target::Minimize(Some(0.5))));
}
