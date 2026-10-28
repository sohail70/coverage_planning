#include "cleaner/control/mpc.hpp"
#include "cleaner/control/reference_path.hpp"
#include "cleaner/geometry.hpp"
#include "cleaner/geometry_types.hpp"
#include <casadi/core/calculus.hpp>
#include <casadi/core/dm_fwd.hpp>
#include <casadi/core/function.hpp>
#include <casadi/core/mx.hpp>
#include <casadi/core/optistack.hpp>
#include <casadi/core/runtime/casadi_runtime.hpp>
#include <casadi/core/slice.hpp>
#include <casadi/core/sparsity_interface.hpp>
#include <cmath>

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
        w
    });

    casadi::Function f("f",{state, control}, {next_state});
    return f; 
}


void MPC::singleShooting() {
    // Define the prediction horizon and create the optimization problem
    int N = 20;
    casadi::Opti opti;

    // Current state parameter and initial predicted state
    casadi::MX x0 = opti.parameter(3);
    casadi::MX X = x0;

    // Reference state the robot should reach
    casadi::DM xref = casadi::DM::zeros(3, 1);
    xref(0) = 1.5;
    xref(1) = 1.5;
    xref(2) = -std::acos(-1.0);

    // Decision variables: linear and angular velocities over N steps
    casadi::MX U = opti.variable(2,N);

    // Create the robot's discrete-time dynamics function
    casadi::Function f = dynamics();

    // State tracking weights: penalize position and heading errors
    casadi::DM Q = casadi::DM::zeros(3,3);
    Q(0,0) = 100;
    Q(1,1) = 100;
    Q(2,2) = 1;

    // Control effort weights: penalize large control inputs
    casadi::DM R = casadi::DM::zeros(2,2);
    R(0,0) = 1.0;
    R(1,1) = 0.5;

    // Terminal state weights: give extra importance to the final prediction
    casadi::DM Qf = casadi::DM::zeros(3, 3);
    Qf(0, 0) = 100;
    Qf(1, 1) = 100;
    Qf(2, 2) = 10;

    // Initialize the objective function
    casadi::MX J = 0;

    // Build the predicted trajectory and accumulate the running cost
    for (int i = 0 ; i < N ; i++) {
        // Extract the control input for prediction step i
        casadi::MX U_k = U(casadi::Slice(), i);

        // Predict the next state using the current state and control
        std::vector<casadi::MX> input{X,U_k};
        X = f(input)[0];

        // Constrain the predicted position to remain inside the map
        opti.subject_to(X(0) >= -2);
        opti.subject_to(X(0) <= 2);
        opti.subject_to(X(1) >= -2);
        opti.subject_to(X(1) <= 2);

        // Penalize the predicted state error and control effort
        casadi::MX error = X - xref;
        J += casadi::MX::mtimes(error.T(), casadi::MX::mtimes(Q, error));
        J += casadi::MX::mtimes(U_k.T(), casadi::MX::mtimes(R, U_k));
    }

    // Add the terminal cost for the final predicted state
    casadi::MX last_prediction_error = X - xref;
    J += casadi::MX::mtimes(last_prediction_error.T() , casadi::MX::mtimes(Qf, last_prediction_error) );

    // Constrain the linear and angular velocities at every prediction step
    for (int i = 0 ; i < N ; i++) {
        opti.subject_to(U(0,i) >= -0.6 );
        opti.subject_to(U(0,i) <= 0.6 );
        opti.subject_to(U(1,i) >= -casadi::pi/4);
        opti.subject_to(U(1,i) <= casadi::pi/4);
    }

    // Tell IPOPT to minimize the objective subject to the constraints
    opti.minimize(J);
    opti.solver("ipopt");

    // Initialize the robot's current state and set the initial parameter value
    casadi::DM x_current = casadi::DM::zeros(3, 1);
    opti.set_value(x0, x_current);

    // Initialize the control sequence used as the solver's initial guess
    casadi::DM U_guess = casadi::DM::zeros(2,N);
    opti.set_initial(U, U_guess);

    // Run the receding-horizon MPC loop
    double t = 0;
    double sim_time = 10.0;
    double Ts = 0.1;
    while (t < sim_time) {
        // Update the current state and warm-start the optimization
        opti.set_value(x0, x_current);
        opti.set_initial(U,U_guess);

        // Solve for the optimal sequence of control inputs
        casadi::OptiSol res = opti.solve();
        casadi::DM U_opt = res.value(U);

        // Apply only the first control input from the optimized sequence
        casadi::DM u_apply = U_opt(casadi::Slice(), 0);

        // Simulate one time step using the robot dynamics
        std::vector<casadi::DM> input{x_current, u_apply};
        x_current = f(input)[0];

        // Shift the solution by one and use it as a warm start
        for (int i = 0 ; i + 1 < N ; i++) {
            U_guess(0,i) = U_opt(0,i+1);
            U_guess(1,i) = U_opt(1,i+1);
        }

        // Initialize the last column because no previous prediction is available
        U_guess(0, N-1) = 0;
        U_guess(1, N-1) = 0;

        // Advance simulation time by one sampling period
        t += Ts;
    }

    // Print the final simulated robot state
    std::cout << "Final state:\n" << x_current << std::endl;
}


void MPC::multipleShooting() {
    casadi::Opti opti;
    int N = 20;
    casadi::MX U = opti.variable(2,N);
    casadi::MX X = opti.variable(3,N+1);
    casadi::DM xref = casadi::DM::zeros(3,1) ;
    xref(0) = 1.5;
    xref(1) = 1.5;
    xref(2) = casadi::pi/4;


    casadi::DM Q = casadi::DM::zeros(3,3);
    Q(0,0) = 100;
    Q(1,1) = 100;
    Q(2,2) = 1;
    casadi::DM Qf = casadi::DM::zeros(3,3);
    Qf(0,0) = 100;
    Qf(1,1) = 100;
    Qf(2,2) = 10;

    casadi::DM R = casadi::DM::zeros(2,2);
    R(0,0) = 1;
    R(1,1) = 0.5;


    casadi::MX J = 0;
    // construct J 
    for (int i = 0 ; i < N ; i++) {
        casadi::MX error = X(casadi::Slice(),i) - xref;
        J+= casadi::MX::mtimes(error.T(), casadi::MX::mtimes(Q, error));
        casadi::MX U_k = U(casadi::Slice(), i);
        J+= casadi::MX::mtimes(U_k.T(), casadi::MX::mtimes(R, U_k));
    }
    casadi::MX error = X(casadi::Slice(),N) - xref; // final state
    J+= casadi::MX::mtimes(error.T(), casadi::MX::mtimes(Qf, error));

    // inequality constraint
    for (int i = 0 ; i < N ; i++) {
        opti.subject_to(X(0,i) >= -2);
        opti.subject_to(X(0,i) <= 2);
        opti.subject_to(X(1,i) >= -2);
        opti.subject_to(X(1,i) <= 2);

        opti.subject_to(U(0,i) >= -0.6);
        opti.subject_to(U(0,i) <= 0.6);
        opti.subject_to(U(1,i) >= -casadi::pi/4);
        opti.subject_to(U(1,i) <= casadi::pi/4);
    }
    // terminal state bounds since X has N+1 dimension
    opti.subject_to(X(0,N) >= -2);
    opti.subject_to(X(0,N) <=  2);
    opti.subject_to(X(1,N) >= -2);
    opti.subject_to(X(1,N) <=  2);




    // eqaulity constraint --> dynamics!
    casadi::Function f = dynamics();
    casadi::MX x0 = opti.parameter(3,1);
    casadi::DM x_current = casadi::DM::zeros(3,1);
    x_current(0) = 0.3;
    x_current(1) = 0.2;
    x_current(2) = 0.3;
    opti.set_value(x0, x_current);
    opti.subject_to(X(casadi::Slice(),0) == x0);
    for (int i = 0 ; i < N; i++) {
        casadi::MX U_k = U(casadi::Slice(), i);
        std::vector<casadi::MX> input{X(casadi::Slice(), i), U_k};
        opti.subject_to(X(casadi::Slice(), i+1) == f(input)[0] );
    }

    opti.minimize(J);
    opti.solver("ipopt");

    double sim_time = 10.0;
    double t = 0.0;
    double Ts = 0.1;



    casadi::DM U_guess = casadi::DM::zeros(2,N);
    opti.set_initial(U, U_guess);

    while (t < sim_time) {
        opti.set_value(x0 , x_current);
        opti.set_initial(U, U_guess);

        casadi::OptiSol sol =  opti.solve();
        casadi::DM U_opt = sol.value(U);
        casadi::DM u_apply = U_opt(casadi::Slice() , 0) ;
        std::vector<casadi::DM> input{x_current, u_apply};
        x_current = f(input)[0];

        // using U_opt (except the first one which we used for u_apply, as a guess for the next optimization cycle)
        for (int i = 0 ; i < N-1 ; i++) {
            U_guess(0,i) = U_opt(0,i+1);
            U_guess(1,i) = U_opt(1,i+1);
        }
        U_guess(0, N-1) = 0;
        U_guess(1, N-1) = 0;

        t += Ts;
    }


    std::cout << "Final state:\n" << x_current << std::endl;

}




void MPC::multipleShootingTracking() {
    casadi::Opti opti;
    int N = 20;
    casadi::MX U = opti.variable(2,N);
    casadi::MX X = opti.variable(3,N+1);


    Segment<double> s1 {Point<double>{0.0,0.0}, Point<double>{5.0,1.0}};

    double theta_ref = atan2(s1.end_.y_ - s1.start_.y_ , s1.end_.x_ - s1.start_.x_);

    double vref = 0.3;
    double length = distance(s1.start_, s1.end_);
    double Ts = 0.1;
    int count = static_cast<int>( std::ceil(length / (vref * Ts)));
    
    casadi::DM xref = casadi::DM::zeros(3, count+1) ;
    xref(0,0) = s1.start_.x_;
    xref(1,0) = s1.start_.y_;
    xref(2,0) = theta_ref;
    for (int i = 0 ; i < count ; i++) {
        xref(0,i+1) = xref(0,i) + vref*cos(theta_ref)*Ts; 
        xref(1,i+1) = xref(1,i) + vref*sin(theta_ref)*Ts; 
        xref(2,i+1) = theta_ref;
    }
    // in case the above "count" variable creats any indexing problems due to int round up!
    xref(0,count) = s1.end_.x_;
    xref(1,count) = s1.end_.y_;





    casadi::DM Q = casadi::DM::zeros(3,3);
    Q(0,0) = 100;
    Q(1,1) = 100;
    Q(2,2) = 1;
    casadi::DM Qf = casadi::DM::zeros(3,3);
    Qf(0,0) = 100;
    Qf(1,1) = 100;
    Qf(2,2) = 10;

    casadi::DM R = casadi::DM::zeros(2,2);
    R(0,0) = 1;
    R(1,1) = 0.5;


    casadi::MX J = 0;
    casadi::MX xref_moving_window_symbolic = opti.parameter(3, N+1);
    // construct J 
    for (int i = 0 ; i < N ; i++) {
        // casadi::MX error = X(casadi::Slice(),i) - xref(casadi::Slice(), start_idx + i);
        casadi::MX error = X(casadi::Slice(),i) - xref_moving_window_symbolic(casadi::Slice(), i);
        J+= casadi::MX::mtimes(error.T(), casadi::MX::mtimes(Q, error));
        casadi::MX U_k = U(casadi::Slice(), i);
        J+= casadi::MX::mtimes(U_k.T(), casadi::MX::mtimes(R, U_k));
    }
    // casadi::MX error = X(casadi::Slice(),N) - xref(casadi::Slice(), start_idx + N); // final state
    casadi::MX error = X(casadi::Slice(),N) - xref_moving_window_symbolic(casadi::Slice(), N); // final state
    J+= casadi::MX::mtimes(error.T(), casadi::MX::mtimes(Qf, error));

    // inequality constraint
    for (int i = 0 ; i < N ; i++) {
        opti.subject_to(X(0,i) >= -20);
        opti.subject_to(X(0,i) <= 20);
        opti.subject_to(X(1,i) >= -20);
        opti.subject_to(X(1,i) <= 20);

        opti.subject_to(U(0,i) >= -0.6);
        opti.subject_to(U(0,i) <= 0.6);
        opti.subject_to(U(1,i) >= -casadi::pi/4);
        opti.subject_to(U(1,i) <= casadi::pi/4);
    }
    // terminal state bounds since X has N+1 dimension
    opti.subject_to(X(0,N) >= -20);
    opti.subject_to(X(0,N) <=  20);
    opti.subject_to(X(1,N) >= -20);
    opti.subject_to(X(1,N) <=  20);


    // eqaulity constraint --> dynamics!
    casadi::Function f = dynamics();
    casadi::MX x0 = opti.parameter(3,1);
    casadi::DM x_current = casadi::DM::zeros(3,1);
    opti.set_value(x0, x_current);
    opti.subject_to(X(casadi::Slice(),0) == x0);
    for (int i = 0 ; i < N; i++) {
        casadi::MX U_k = U(casadi::Slice(), i);
        std::vector<casadi::MX> input{X(casadi::Slice(), i), U_k};
        opti.subject_to(X(casadi::Slice(), i+1) == f(input)[0] );
    }


    opti.minimize(J);
    opti.solver("ipopt");

    double sim_time = 20.0;
    double t = 0.0;



    casadi::DM U_guess = casadi::DM::zeros(2,N);
    opti.set_initial(U, U_guess);

    int start_idx = 0;
    casadi::DM xref_window = casadi::DM::zeros(3, N+1); // for filling using the caluclated points of the trajectory which has count+1 columns

    while (t < sim_time) {
        opti.set_value(x0 , x_current);
        opti.set_initial(U, U_guess);
        for (int i = 0 ; i < N+1 ; i++) {
            if (start_idx+i <= count){
                xref_window(0,i) = xref(0,start_idx+i);
                xref_window(1,i) = xref(1, start_idx+i);
                xref_window(2,i) = xref(2, start_idx+i);
            }
            else {
                xref_window(0,i) = xref(0,count);
                xref_window(1,i) = xref(1, count);
                xref_window(2,i) = xref(2,count);
            }
        }
        opti.set_value(xref_moving_window_symbolic, xref_window);

        casadi::OptiSol sol =  opti.solve();
        casadi::DM U_opt = sol.value(U);
        casadi::DM u_apply = U_opt(casadi::Slice() , 0) ;
        std::vector<casadi::DM> input{x_current, u_apply};
        x_current = f(input)[0];

        // using U_opt (except the first one which we used for u_apply, as a guess for the next optimization cycle)
        for (int i = 0 ; i < N-1 ; i++) {
            U_guess(0,i) = U_opt(0,i+1);
            U_guess(1,i) = U_opt(1,i+1);
        }
        U_guess(0, N-1) = 0;
        U_guess(1, N-1) = 0;

        start_idx = std::min(start_idx + 1, count);
        t += Ts;
    }


    std::cout << "Final state:\n" << x_current << std::endl;

}