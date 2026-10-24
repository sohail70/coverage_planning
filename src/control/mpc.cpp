#include "cleaner/control/mpc.hpp"
#include <casadi/core/calculus.hpp>
#include <casadi/core/mx.hpp>
#include <casadi/core/optistack.hpp>

ControlCommand MPC::compute(const RobotState& current_state, const ReferencePath2D& reference_path) {

}

void MPC::reset() {

}


void MPC::test1() {
    casadi::Opti opti; // Optimization Poroblem
    casadi::MX w = opti.variable(); // Decision Variable
    casadi::MX J = w*w - 6*w + 13; // Objective Function
    opti.minimize(J); // Tell the optimizer to minimize J
    opti.solver("ipopt"); // choose ipopt
    casadi::OptiSol solution = opti.solve(); // solve
    std::cout<<"Optimal w: "<<solution.value(w)<<"\n";
    std::cout<<"Minimum J: "<<solution.value(J)<<"\n";
}

/*
    min e^(0.2w) sin(w)
     w
    s.t. w>=0 and w<=4pi
    set_initial would be important because this problem has 3 minimum 2 of which are local and another is global! so if you choose initial w of 1.0 would get zero
*/
void MPC::test2() {
    casadi::Opti opti;
    casadi::MX w = opti.variable();
    casadi::MX J = casadi::MX::exp(0.2*w) * casadi::MX::sin(w); // you can also ommit casadi::MX for exp and sin! because they are exposed in the casadi namespace!
    opti.minimize(J);
    opti.solver("ipopt");
    opti.subject_to(w>=0);
    opti.subject_to(w<=4*casadi::pi);
    opti.set_initial(w,10.0);
    casadi::OptiSol solution = opti.solve();
    std::cout<<"Optimal w: "<<solution.value(w)<<"\n";
    std::cout<<"Optimal J: "<<solution.value(J)<<"\n";
}

/*
    linear regression example
    y = ax+b 
    we have data point we want to fit a function to those data points
    least sqaure --> minimize the output of the model wrt to the data point
    J(a,b) = sum (yi - (a*xi+b))^2
              i

*/

void MPC::test3() {
    // # x_data = [0,45,90,135,180]
    // # y_data = [667,661,757,871,1210]
    std::vector<int> x_data {0,45,90,135,180};
    std::vector<int> y_data {667,661,757,871,1210};
    casadi::Opti opti;
    casadi::MX a = opti.variable();
    casadi::MX b = opti.variable();

    casadi::MX J = 0;
    for (int i = 0 ; i < x_data.size() ; i++) {
        auto y_hat = a * x_data[i] + b;
        auto y = y_data[i];
        auto error = y - y_hat;
        J += error * error;
    }
    opti.minimize(J);
    opti.solver("ipopt");
    casadi::OptiSol solution = opti.solve();

    std::cout<<"Optimal a and b : " << solution.value(a) <<", "<<solution.value(b)<<"\n";
    std::cout<<"Optimal J : "<<solution.value(J)<<"\n";

    
}