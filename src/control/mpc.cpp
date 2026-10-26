#include "cleaner/control/mpc.hpp"
#include "cleaner/control/reference_path.hpp"
#include <casadi/core/calculus.hpp>
#include <casadi/core/function.hpp>
#include <casadi/core/mx.hpp>
#include <casadi/core/optistack.hpp>
#include <casadi/core/runtime/casadi_runtime.hpp>
#include <casadi/core/slice.hpp>
#include <casadi/core/sparsity_interface.hpp>

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

casadi::Function MPC::dynamics() {
    const double Ts = 0.1;
    // State Variables
    casadi::MX x = casadi::MX::sym("x");
    casadi::MX y = casadi::MX::sym("y");
    casadi::MX theta = casadi::MX::sym("theta");
    casadi::MX state = casadi::MX::vertcat({x,y,theta});

    // Control Variables
    casadi::MX v = casadi::MX::sym("v");
    casadi::MX w = casadi::MX::sym("w");

    casadi::MX control = casadi::MX::vertcat({v, w});

    // Discrete Time Robot Dynamic
    casadi::MX next_state =  state + Ts * casadi::MX::vertcat({
        v * casadi::MX::cos(theta),
        v * casadi::MX::sin(theta),
        theta + Ts * w
    });

    casadi::Function f("f",{state, control}, {next_state});
    return f; 
}


void MPC::test4() {
    // how to contruct J for MPC?
    int N = 3;
    casadi::Opti opti;
    casadi::MX x0 = opti.parameter(3);
    casadi::MX X = x0;

    // Reference state
    casadi::DM xref = casadi::DM::zeros(3, 1);
    xref(0) = 1.5;
    xref(1) = 1.5;
    xref(2) = -std::acos(-1.0);


    casadi::MX U = opti.variable(2,N);

    casadi::Function f = dynamics();

    casadi::DM Q = casadi::DM::zeros(3,3);
    Q(0,0) = 100;
    Q(1,1) = 100;
    Q(2,2) = 1;

    casadi::DM R = casadi::DM::zeros(2,2);
    R(0,0) = 1.0;
    R(1,1) = 0.5;

    casadi::DM Qf = casadi::DM::zeros(3, 3);
    Qf(0, 0) = 100;
    Qf(1, 1) = 100;
    Qf(2, 2) = 10;


    casadi::MX J = 0;
    // Lets build J!
    for (int i = 0 ; i < N ; i++) {
        casadi::MX U_k = U(casadi::Slice(), i);
        std::vector<casadi::MX> input{X,U_k};
        X = f(input)[0];
        opti.subject_to(X(0) >= -2);
        opti.subject_to(X(0) <= 2);
        opti.subject_to(X(1) >= -2);
        opti.subject_to(X(1) <= 2);

        casadi::MX error = X - xref;
        J += casadi::MX::mtimes(error.T(), casadi::MX::mtimes(Q, error));
        J += casadi::MX::mtimes(U_k.T(), casadi::MX::mtimes(R, U_k));
    }
    casadi::MX last_prediction_error = X - xref;
    J += casadi::MX::mtimes(last_prediction_error.T() , casadi::MX::mtimes(Qf, last_prediction_error) );


    


}