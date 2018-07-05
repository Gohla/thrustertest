extern crate lpsolve;

use lpsolve::*;
//use std::ffi::CString;

fn main() {
    println!("Calculating:");
//    let mut p = Problem::read_freemps(&CString::new("model.mps").unwrap(), NORMAL).unwrap();
    let mut p = Problem::new(0, 2).unwrap();
    // NOTE: Element 0 of each list of coefficients is ignored.
    // NOTE: Not putting the lists in separate variables causes an unknown error,
    // probably because the data is not longer in memory after it has been borrowed.
    let r0 = [0.0, 143.0, 60.0];
    p.set_objective_function(&r0);
    let r1 = [0.0, 120.0, 210.0];
    p.add_constraint(&r1,150000.0, ConstraintType::Le);
    let r2 = [0.0, 110.0,  30.0];
    p.add_constraint(&r2,4000.0, ConstraintType::Le);
    let r3 = [0.0,  1.0,   1.0];
    p.add_constraint(&r3,75.0, ConstraintType::Le);
    p.set_maxim();
    p.set_verbose(Verbosity::Important);

    println!("Solving:");
    let status = p.solve();
    match status {
        SolveStatus::Optimal | SolveStatus::Suboptimal => {
            let mut result = [0.0; 2];
            p.get_solution_variables(&mut result);
            let obj = p.get_objective();
            let x = result[0];
            let y = result[1];
            println!("Result: x= {}, y= {} = {}", x, y, obj);
        },
        _ => {
            println!("Could not find a solution, or an error occurred: {:?}", status)
        }
    }
    println!("Done");
}
