#include "cleaner/control/mpc.hpp"
#include "cleaner/control/control_types.hpp"
#include "cleaner/control/reference_path.hpp"
#include "cleaner/geometry.hpp"
#include "cleaner/geometry_types.hpp"
#include <casadi/core/calculus.hpp>
#include <casadi/core/dm_fwd.hpp>
#include <casadi/core/function.hpp>
#include <casadi/core/generic_expression.hpp>
#include <casadi/core/generic_type.hpp>
#include <casadi/core/mx.hpp>
#include <casadi/core/optistack.hpp>
#include <casadi/core/runtime/casadi_runtime.hpp>
#include <casadi/core/slice.hpp>
#include <casadi/core/sparsity_interface.hpp>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <system_error>



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



// Where should the robot be at time t?
void MPC::multipleShootingTrajectoryTracking() {
    casadi::Opti opti;
    int N = 20;
    casadi::MX U = opti.variable(2,N);
    casadi::MX X = opti.variable(3,N+1);


    Segment<double> s1 {Point<double>{0.0,0.0}, Point<double>{7.0,-1.0}};

    double theta_ref = atan2(s1.end_.y_ - s1.start_.y_ , s1.end_.x_ - s1.start_.x_);

    double vref = 0.9;
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

    double w_backward = 50.0;
    casadi::MX J = 0;
    casadi::MX xref_moving_window_symbolic = opti.parameter(3, N+1);
    // construct J 
    for (int i = 0 ; i < N ; i++) {
        // casadi::MX error = X(casadi::Slice(),i) - xref(casadi::Slice(), start_idx + i);
        casadi::MX error = X(casadi::Slice(),i) - xref_moving_window_symbolic(casadi::Slice(), i);
        J+= casadi::MX::mtimes(error.T(), casadi::MX::mtimes(Q, error));
        casadi::MX U_k = U(casadi::Slice(), i);
        J+= casadi::MX::mtimes(U_k.T(), casadi::MX::mtimes(R, U_k));

        // Optional : forward movement costs nothing extra; backward movement adds a cost to the objective.
        casadi::MX v = U(0,i);
        casadi::MX backward_penalty = casadi::MX::pow(v - casadi::MX::sqrt(v*v + 1e-6), 2);
        J += w_backward * backward_penalty;
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
    x_current(0) = 0;
    x_current(1) = -2;
    x_current(2) = casadi::pi;
    opti.set_value(x0, x_current);
    opti.subject_to(X(casadi::Slice(),0) == x0);
    for (int i = 0 ; i < N; i++) {
        casadi::MX U_k = U(casadi::Slice(), i);
        std::vector<casadi::MX> input{X(casadi::Slice(), i), U_k};
        opti.subject_to(X(casadi::Slice(), i+1) == f(input)[0] );
    }


    opti.minimize(J);
    opti.solver("ipopt");

    double sim_time = 20.0; // if the vref = 0.3 and the distance of the segment is 5 then we need at least 16.67 seconds
    double t = 0.0;



    casadi::DM U_guess = casadi::DM::zeros(2,N);
    opti.set_initial(U, U_guess);

    int start_idx = 0;
    casadi::DM xref_window = casadi::DM::zeros(3, N+1); // for filling using the caluclated points of the trajectory which has count+1 columns


    std::vector<cv::Point2d> robot_path; // PLOT

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

        /////////////PLOT/////////////////
        // Store robot position
        robot_path.emplace_back(
            static_cast<double>(x_current(0)),
            static_cast<double>(x_current(1))
        );

        // Visualize
        visualizeRobot(x_current, s1, robot_path);
        ///////////END PLOT/////////////

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
    cv::waitKey(0);
    cv::destroyAllWindows();

}

// Instead of asking: Where should the robot be at time t? Ask: How close is the robot to the path, and how far should it progress along that path?
// now for each moving xref you need to use the distancePointToSegment to find the closest point from the current position of the robot to the segment

/*
 * PATH-FOLLOWING MPC OBJECTIVE
 *
 * Goal:
 *   Keep the robot close to a geometric path, align its heading with the
 *   path direction, and regulate its velocity along the path.
 *
 *   Unlike trajectory tracking, we do not prescribe an (x,y) reference
 *   position for every time step. Instead, we penalize geometric path
 *   deviation and track a desired path-progress velocity.
 *
 * PATH GEOMETRY:
 *   direction = (B - A) / ||B - A||     Unit tangent of the path.
 *   normal    = (-ty, tx)               Unit normal to the path.
 *   c         = A . normal              Line equation: P . normal = c.
 *
 * STAGE COST (for each predicted state k = 0,...,N-1):
 *
 * 1. CROSS-TRACK ERROR:
 *      e_ct = X(k) . normal - c
 *
 *    Signed perpendicular distance from the predicted robot position
 *    to the infinite line containing the segment.
 *    Penalizing e_ct^2 keeps the robot close to the path.
 *
 * 2. HEADING ERROR:
 *      e_heading = wrap(theta(k) - theta_ref)
 *
 *    Shortest angular difference between the robot heading and the
 *    path tangent direction. Penalizing its square aligns the robot
 *    with the path.
 *
 * 3. CONTROL EFFORT:
 *      J_control = U(k)^T * R * U(k)
 *
 *    Penalizes excessive linear and angular control inputs.
 *
 * 4. ALONG-PATH POSITION:
 *      s(k) = direction . (position(k) - A)
 *
 *    Projection of the robot's displacement from the segment start
 *    onto the unit path tangent. Measured in metres.
 *
 * 5. REMAINING DISTANCE:
 *      remaining(k) = max(0, length - s(k))
 *
 *    Estimated distance from the predicted robot position to the
 *    segment endpoint, measured along the path.
 *
 * 6. DESIRED PATH SPEED:
 *      v_ref(k) = min(v_max, sqrt(2 * a_brake * remaining(k)))
 *
 *    Distance-dependent speed profile based on the stopping-distance
 *    equation v^2 = 2 * a * d.
 *    The desired speed decreases as the robot approaches the endpoint.
 *    a_brake is a positive deceleration magnitude.
 *
 * 7. ACTUAL PATH-PROGRESS RATE:
 *      s_dot(k) = direction . [v*cos(theta), v*sin(theta)]
 *
 *    Projection of the robot's world-frame velocity onto the path
 *    tangent. Measures how fast the robot advances along the path,
 *    in metres per second.
 *
 * 8. PROGRESS-RATE TRACKING:
 *      J_progress = q_progress * (s_dot(k) - v_ref(k))^2
 *
 *    Encourages the robot to achieve the desired speed along the path,
 *    rather than simply moving forward in its body-frame direction.
 *
 * 9. BACKWARD-MOTION PENALTY:
 *    Penalizes negative linear velocity while allowing forward motion.
 *
 * TERMINAL COST (at k = N):
 *    Penalizes terminal cross-track and heading errors so that the
 *    predicted trajectory ends close to the path and properly aligned.
 *
 * Complete objective:
 *    J = sum(stage costs) + terminal cost.
 *
 * IMPORTANT:
 *    This is path following with an assigned, distance-dependent speed
 *    profile. The optimizer tracks the desired path speed; it does not
 *    independently optimize a virtual path-progress variable.
 *
 *    Cross-track error describes the infinite supporting line.
 *    Finite-segment endpoint constraints must be handled separately.

 *    s_k and s_dot_k are different quantities. s_k is position along the path (metres); s_dot_k is velocity along the path (metres/second)
 */

void MPC::multipleShootingPathTracking() {
    casadi::Opti opti;
    int N = 20;
    casadi::MX U = opti.variable(2,N);
    casadi::MX X = opti.variable(3,N+1);


    Segment<double> s1 {Point<double>{1.0,1.0}, Point<double>{7.0,-1.0}};

    Point<double> direction{s1.end_- s1.start_};
    normalize(direction);
    auto normal = normalLeft(direction);
    normalize(normal); // unnecessary
    double theta_ref = atan2(s1.end_.y_ - s1.start_.y_ , s1.end_.x_ - s1.start_.x_);
    double length = distance(s1.start_, s1.end_);
    double c = dotProduct2D(s1.start_, normal); // P.n = c
    double Ts = 0.1;


    casadi::DM R = casadi::DM::zeros(2,2);
    R(0,0) = 1;
    R(1,1) = 0.5;

    double w_backward = 50.0;
    casadi::MX J = 0;

    double vmax = 0.3;
    double a_brake = 0.1;
    double q_ct = 100;
    double q_heading = 10;
    double q_progress = 100;
    // construct J 
    for (int i = 0 ; i < N ; i++) {
        // Cross track error! perpendicular distance of each predicted position to the segment
        casadi::MX error_ct = X(0,i)*normal.x_ + X(1,i)*normal.y_ - c;
        J+= q_ct * error_ct * error_ct;

        // // shortest angular difference instead of simple "X(2,i) - theta_ref"
        // casadi::MX angle_error = X(2,i) - theta_ref;
        // casadi::MX error_heading = casadi::MX::atan2( casadi::MX::sin(angle_error), casadi::MX::cos(angle_error));
        // J+= q_heading * error_heading * error_heading;
        casadi::MX angle_diff = X_(2,i) - theta_ref;
        J_ += 2.0 * config_.q_heading * (1.0 - casadi::MX::cos(angle_diff));


        casadi::MX U_k = U(casadi::Slice(), i);
        J+= casadi::MX::mtimes(U_k.T(), casadi::MX::mtimes(R, U_k));

        // Progress term for unicycle model
        // Along-path position [m]
        casadi::MX s_k = direction.x_ * (X(0,i) - s1.start_.x_) + direction.y_ * (X(1,i) - s1.start_.y_);

        // // Remaining distance [m]
        // casadi::MX remaining = casadi::MX::fmax(0.0, length - s_k);

        // This smooths the transition when the predicted robot position reaches the segment endpoint.
        casadi::MX raw_remaining = length - s_k;

        constexpr double smooth_eps = 1e-3;

        casadi::MX remaining = 0.5 * (
            raw_remaining +
            casadi::MX::sqrt(
                raw_remaining * raw_remaining + smooth_eps * smooth_eps
            )
        );



        // Desired speed based on stopping distance [m/s]
        double eps = 1e-6;

        casadi::MX vref_k = casadi::MX::fmin(
                vmax,
                casadi::MX::sqrt(2.0 * a_brake * remaining + eps)
                    - std::sqrt(eps)
            );

        // Actual progress rate [m/s]
        casadi::MX s_dot_k = direction.x_ * U_k(0) * casadi::MX::cos(X(2,i)) + direction.y_ * U_k(0) * casadi::MX::sin(X(2,i));
        // Progress-rate tracking cost
        casadi::MX error_progress = s_dot_k - vref_k;
        J += q_progress * error_progress * error_progress;

        // Optional : forward movement costs nothing extra; backward movement adds a cost to the objective.
        casadi::MX v = U(0,i);
        casadi::MX backward_penalty = casadi::MX::pow(v - casadi::MX::sqrt(v*v + 1e-6), 2);
        J += w_backward * backward_penalty;
    }
    

    casadi::MX error_ct = X(0,N) * normal.x_ + X(1,N) * normal.y_  - c;

    // casadi::MX angle_error = X(2,N) - theta_ref;
    // casadi::MX error_heading = casadi::MX::atan2( casadi::MX::sin(angle_error), casadi::MX::cos(angle_error));
    // J+= q_ct * error_ct * error_ct + q_heading * error_heading * error_heading;

    casadi::MX angle_error = X_(2,config_.N) - theta_ref;
    J_ += config_.q_ct * error_ct * error_ct + 2.0 * config_.q_heading * (1.0 - casadi::MX::cos(angle_error));
    

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
    x_current(0) = 0;
    x_current(1) = -2;
    x_current(2) = casadi::pi;
    opti.set_value(x0, x_current);
    opti.subject_to(X(casadi::Slice(),0) == x0);
    for (int i = 0 ; i < N; i++) {
        casadi::MX U_k = U(casadi::Slice(), i);
        std::vector<casadi::MX> input{X(casadi::Slice(), i), U_k};
        opti.subject_to(X(casadi::Slice(), i+1) == f(input)[0] );
    }


    opti.minimize(J);
    opti.solver("ipopt");

    double sim_time = 50.0; // if the vref = 0.3 and the distance of the segment is 5 then we need at least 16.67 seconds
    double t = 0.0;



    casadi::DM U_guess = casadi::DM::zeros(2,N);
    opti.set_initial(U, U_guess);

    casadi::DM X_guess = casadi::DM::repmat(x_current, 1, N+1);
    opti.set_initial(X, X_guess);


    std::vector<cv::Point2d> robot_path; // PLOT

    while (t < sim_time) {
        opti.set_value(x0 , x_current);
        opti.set_initial(U, U_guess);
        opti.set_initial(X, X_guess);
       
        casadi::OptiSol sol =  opti.solve();


        casadi::DM U_opt = sol.value(U);
        casadi::DM u_apply = U_opt(casadi::Slice() , 0) ;
        std::vector<casadi::DM> input{x_current, u_apply};
        x_current = f(input)[0];

        /////////////PLOT/////////////////
        // Store robot position
        robot_path.emplace_back(
            static_cast<double>(x_current(0)),
            static_cast<double>(x_current(1))
        );

        // Visualize
        visualizeRobot(x_current, s1, robot_path);
        ///////////END PLOT/////////////

        // using U_opt (except the first one which we used for u_apply, as a guess for the next optimization cycle)
        for (int i = 0 ; i < N-1 ; i++) {
            U_guess(0,i) = U_opt(0,i+1);
            U_guess(1,i) = U_opt(1,i+1);
        }
        U_guess(0, N-1) = 0;
        U_guess(1, N-1) = 0;

        // Same for X_opt
        casadi::DM X_opt = sol.value(X);
        // Shift predicted states by one timestep
        for (int i = 0; i < N; i++) {
            X_guess(casadi::Slice(), i) =
                X_opt(casadi::Slice(), i+1);
        }
        // Repeat the terminal state
        X_guess(casadi::Slice(), N) = X_opt(casadi::Slice(), N);

        t += Ts;
    }


    std::cout << "Final state:\n" << x_current << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();

}


void MPC::visualizeRobot(
    const casadi::DM& state,
    const Segment<double>& reference,
    const std::vector<cv::Point2d>& robot_path)
{
    const int width = 800;
    const int height = 600;
    const int padding = 50;

    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    // World boundaries with some margin
    double xmin = -2.0;
    double xmax =  9.0;
    double ymin = -4.0;
    double ymax =  2.0;

    // Preserve aspect ratio
    double scale = std::min(
        (width - 2.0 * padding) / (xmax - xmin),
        (height - 2.0 * padding) / (ymax - ymin)
    );

    double offset_x = (width - scale * (xmax - xmin)) / 2.0;
    double offset_y = (height - scale * (ymax - ymin)) / 2.0;

    // Convert world coordinates to image coordinates
    auto worldToPixel = [&](double x, double y) {
        int px = static_cast<int>(offset_x + (x - xmin) * scale);
        int py = static_cast<int>(height - offset_y - (y - ymin) * scale);

        return cv::Point(px, py);
    };

    // Draw reference segment
    cv::Point start = worldToPixel(reference.start_.x_, reference.start_.y_);
    cv::Point goal = worldToPixel(reference.end_.x_, reference.end_.y_);

    cv::line(image, start, goal, cv::Scalar(200, 200, 200), 3);
    cv::circle(image, start, 6, cv::Scalar(0, 180, 0), -1);
    cv::circle(image, goal, 6, cv::Scalar(0, 0, 255), -1);

    // Draw robot trajectory
    for (size_t i = 1; i < robot_path.size(); i++) {
        cv::line(
            image,
            worldToPixel(robot_path[i-1].x, robot_path[i-1].y),
            worldToPixel(robot_path[i].x, robot_path[i].y),
            cv::Scalar(255, 150, 0),
            2
        );
    }

    // Extract robot state
    double x = static_cast<double>(state(0));
    double y = static_cast<double>(state(1));
    double theta = static_cast<double>(state(2));

    cv::Point center = worldToPixel(x, y);

    // Robot body
    int robot_radius = static_cast<int>(0.15 * scale);

    cv::circle(
        image,
        center,
        robot_radius,
        cv::Scalar(255, 0, 0),
        2
    );

    // Robot heading
    double heading_length = 0.4;

    double hx = x + heading_length * std::cos(theta);
    double hy = y + heading_length * std::sin(theta);

    cv::line(
        image,
        center,
        worldToPixel(hx, hy),
        cv::Scalar(0, 0, 255),
        3
    );

    cv::circle(image, center, 4, cv::Scalar(255, 0, 0), -1);

    // Display
    cv::imshow("MPC Robot Tracking", image);
    cv::waitKey(1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
    construct the CasADi optimization problem once, then update its parameters whenever the robot 
    state or reference changes.
*/

MPC::MPC() {
    buildProblem();
    buildObjective();
    buildConstraints();

    casadi::Dict options;
    options["expand"] = true; //Converts the problem from MX to SX, which evaluates faster
    options["print_time"] = false; //Stops CasADi printing timing tables
    options["ipopt.print_level"] = 0; //Stops IPOPT printing each iteration
    options["ipopt.sb"] = "yes"; //Hides the IPOPT startup banner
    opti_.solver("ipopt", options);

    reset();
}

// Call Every 50 ms
ControlResult MPC::compute(const RobotState& current_state, const ReferencePath2D& local_reference_path) {
    try {
        updateParameters(current_state, local_reference_path);

        // Initialize the warm start from the measured robot state
        // only if no previous successful solution is available.
        if (!warm_start_valid_) {
            casadi::DM x0 = casadi::DM::zeros(3,1);
            x0(0) = current_state.pose_.position_.x_;
            x0(1) = current_state.pose_.position_.y_;
            x0(2) = current_state.pose_.heading_;

            X_guess_ = casadi::DM::repmat(x0, 1, config_.N+1);
            U_guess_ = casadi::DM::zeros(2, config_.N);
        }

        ControlResult result = solve();

        warm_start_valid_ = (result.status_ == ControlStatus::SUCCESS);

        return result;
    }
    catch (const std::exception& e) {
        warm_start_valid_ = false;

        ControlResult result;
        result.status_ = ControlStatus::FAILURE;
        result.command_.header_.frame_id_ = "base_link";
        result.command_.header_.stamp_ = std::chrono::system_clock::now();
        result.command_.twist_.linear_x_ = 0.0;
        result.command_.twist_.linear_y_ = 0.0;
        result.command_.twist_.angular_z_ = 0.0;

        return result;
    }
}


casadi::Function MPC::motionModel() {
    const double Ts = config_.Ts;
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





// creates X_, U_ and all symbolic parameters.
void MPC::buildProblem() {
    // Assuming for now the states are [x,y,theta]
    X_ = opti_.variable(3, config_.N + 1);

    // Control inputs [v,w]
    U_ = opti_.variable(2, config_.N);

    // Runtime parameters
    params_.x0        = opti_.parameter(3,1);
    params_.start     = opti_.parameter(2,1);
    params_.direction = opti_.parameter(2,1);
    params_.vref    = opti_.parameter(1,config_.N);
    params_.theta_ref      = opti_.parameter(1,config_.N+1);

}



/*
    WHY THE SPEED AND HEADING TARGETS ARE COMPUTED OUTSIDE THE SOLVER
    (see multipleShootingPathTracking() above for the first version of this cost)

    The first version computed the target speed inside the cost:

        v_ref = min(vmax, sqrt(2 * a_brake * max(0, length - s)))

    That made the robot freeze near the end of a segment:
        - Close to the endpoint the sqrt curve is almost vertical (slope = a_brake / sqrt(eps),
          which is 100 for a_brake = 0.1 and eps = 1e-6), and past the endpoint fmax makes it
          completely flat. The solver follows slopes, so it failed with Restoration_Failed as
          soon as the prediction reached that point, about a_brake * T^2 / 2 metres before the
          end (T = N * Ts is the horizon).
        - A failed solve commands v = 0, so the state does not change, and the next solve fails
          in exactly the same way. The robot never recovers.
    A larger eps only softens the curve. Here the solver receives the targets as plain numbers,
    filled in by updateParameters(), so the steep curve is not part of the problem at all.

    WHY THE ROBOT STOPPED TRACKING THE LINE AFTER A TURN
    The robot used to switch segment 0.15 m before the corner, so it started turning early and
    joined the next line about 0.13 m to the side. The fix is in three places:
        - ExecutionManager: arrive at 0.03 m, measured along the segment only.
        - updateParameters(): keep a small creep speed until arrival, otherwise the target
          speed fades to zero and the robot never gets there.
        - updateParameters(): while the robot is not facing the segment, the target speed is 0
          and the target heading turns gradually (stop and rotate).

    OTHER CHANGES TO THE COST
        - Heading uses 2 * (1 - cos(error)) instead of atan2(sin, cos). Lanes alternate between
          0 and 180 degrees, which is exactly where the atan2 form jumps.
        - q_speed tracks v directly. At a 90 degree heading error s_dot does not depend on v,
          so the progress term alone cannot control it.
        - r_v is gone because it pulled the robot below its target speed.
        - The backward penalty is replaced by the hard limit v >= 0 in buildConstraints().
        - r_du penalizes changes in control between stages to smooth the commands.

    WHY THE HORIZON DOES NOT SEE THE NEXT SEGMENT
    Tested on the first 12 segments of map.png (max cross-track error):
        tolerance 0.15, next segment visible : 0.128 m
        tolerance 0.03, next segment visible : 0.026 m
        tolerance 0.03, one segment only     : 0.025 m
    A robot that stops at every corner has nothing to prepare for, so a window of segments does
    not help. It is worth adding if the robot ever drives through corners without stopping.
*/
void MPC::buildObjective() {
    J_ = 0;

    // Unit normal vector: perpendicular to the segment direction.
    casadi::MX normal_x = -params_.direction(1);
    casadi::MX normal_y = params_.direction(0);

    // Line equation: normal.x * x + normal.y * y = c
    casadi::MX c = params_.start(0) * normal_x + params_.start(1) * normal_y;

    for (int i = 0 ; i < config_.N ; i++) {
        // Cross-track error: signed perpendicular distance from the predicted position to the infinite line supporting the segment.
        casadi::MX e_ct = X_(0,i)*normal_x + X_(1,i)*normal_y - c;
        J_ += config_.q_ct * e_ct * e_ct;

        // OLD (wrong): one fixed heading target for the whole horizon, and an atan2 error.
        //     The robot was asked to face the segment at every stage, with no gradual turn.
        //     atan2(sin, cos) also jumps at 180 degrees, and our lanes alternate between 0 and 180.
        //
        //     casadi::MX theta_ref = casadi::MX::atan2(params_.direction(1), params_.direction(0));
        //     casadi::MX angle_diff = X_(2,i) - theta_ref;
        //     casadi::MX e_heading = casadi::MX::atan2(casadi::MX::sin(angle_diff), casadi::MX::cos(angle_diff));
        //     J_ += config_.q_heading * e_heading * e_heading;

        // Heading error: smooth around the whole circle, approximately error^2 for small errors.
        J_ += config_.q_heading * 2.0 * (1.0 - casadi::MX::cos(X_(2,i) - params_.theta_ref(i)));

        // OLD (wrong): r_v pulled the robot below its target speed, because any speed had a cost.
        //
        //     J_ += config_.r_v * U_k(0) * U_k(0) + config_.r_w * U_k(1) * U_k(1);

        // Control effort: penalize angular velocity only, the speed is handled by the tracking terms below.
        casadi::MX U_k = U_(casadi::Slice(), i);
        J_ += config_.r_w * U_k(1) * U_k(1);

        // OLD (wrong): this is what froze the robot near the end of a segment.
        //     Inside the solver, sqrt is almost vertical near the endpoint and fmax is flat past it.
        //     IPOPT failed with Restoration_Failed there, and v = 0 meant every later solve failed too.
        //     The same formula now runs in plain C++ in updateParameters(), and the solver only sees a number.
        //
        //     casadi::MX s_k = params_.direction(0) * (X_(0,i) - params_.start(0)) + params_.direction(1) * (X_(1,i) - params_.start(1));
        //     casadi::MX remaining = casadi::MX::fmax(0.0, params_.length - s_k);
        //     constexpr double eps = 1e-6;
        //     casadi::MX v_ref_k = casadi::MX::fmin(
        //         params_.vmax,
        //         casadi::MX::sqrt(2.0 * config_.a_brake * remaining + eps) - std::sqrt(eps)
        //     );
        //     casadi::MX e_progress = s_dot_k - v_ref_k;

        // Along-path progress rate: project the predicted world-frame translational velocity onto the segment direction.
        // For a unicycle: world velocity = [v*cos(theta), v*sin(theta)].
        casadi::MX s_dot_k = params_.direction(0) * U_k(0) * casadi::MX::cos(X_(2,i)) + params_.direction(1) * U_k(0) * casadi::MX::sin(X_(2,i));

        // Progress-speed tracking: encourage the actual along-path velocity to match the desired speed.
        casadi::MX e_progress = s_dot_k - params_.vref(i);
        J_ += config_.q_progress * e_progress * e_progress;

        // Speed tracking: keeps control of v when the heading is far from the segment direction.
        casadi::MX e_speed = U_k(0) - params_.vref(i);
        J_ += config_.q_speed * e_speed * e_speed;

        // Control smoothing: penalize the change in [v,w] between consecutive stages.
        if (i > 0) {
            casadi::MX delta_u = U_k - U_(casadi::Slice(), i-1);
            J_ += config_.r_du * casadi::MX::dot(delta_u, delta_u);
        }

        // OLD (replaced): a soft penalty still allows some reverse, and it adds another sqrt to the cost.
        //     The hard limit v >= 0 in buildConstraints() is simpler and cannot be traded off.
        //
        //     casadi::MX v = U_k(0);
        //     casadi::MX backward_penalty = casadi::MX::pow(v - casadi::MX::sqrt(v*v + 1e-6), 2);
        //     J_ += config_.w_backward * backward_penalty;
    }

    // OLD (wrong): same problems as the stage heading cost above, a fixed target and an atan2 error.
    //
    //     casadi::MX angle_error = X_(2,config_.N) - theta_ref;
    //     casadi::MX error_heading = casadi::MX::atan2(casadi::MX::sin(angle_error), casadi::MX::cos(angle_error));
    //     J_ += config_.q_ct * error_ct * error_ct + config_.q_heading * error_heading * error_heading;

    // Terminal cost: penalize cross-track and heading errors at the final predicted state.
    casadi::MX error_ct = X_(0,config_.N) * normal_x + X_(1,config_.N) * normal_y - c;

    J_ += config_.q_ct * error_ct * error_ct
        + config_.q_heading * 2.0 * (1.0 - casadi::MX::cos(X_(2,config_.N) - params_.theta_ref(config_.N)));

    // Register the complete objective function with the optimizer.
    opti_.minimize(J_);
}





void MPC::buildConstraints() {
    // Inequality constraints for the current state [x,y,theta] and control [v,w]
    for (int i = 0 ; i < config_.N ; i++) {
        opti_.subject_to(X_(0,i) <= 50); // 50 m
        opti_.subject_to(X_(0,i) >= -50);
        opti_.subject_to(X_(1,i) <= 50);
        opti_.subject_to(X_(1,i) >= -50);

        // Robot velocity limits
        opti_.subject_to(U_(0,i) <= config_.max_linear_speed);
        opti_.subject_to(U_(0,i) >= 0.0); // no reverse
        opti_.subject_to(U_(1,i) <= config_.max_angular_speed);
        opti_.subject_to(U_(1,i) >= -config_.max_angular_speed);

        // OLD (replaced): reverse was allowed here and only discouraged by the backward penalty,
        //     and the segment speed limit was a constraint. It now reaches the solver through vref.
        //
        //     opti_.subject_to(U_(0,i) >= -config_.max_linear_speed);
        //     opti_.subject_to(U_(0,i) <= params_.vmax);
        //     opti_.subject_to(U_(0,i) >= -params_.vmax);
    }

    // Terminal state bounds
    opti_.subject_to(X_(0,config_.N) <= 50);
    opti_.subject_to(X_(0,config_.N) >= -50);
    opti_.subject_to(X_(1,config_.N) <= 50);
    opti_.subject_to(X_(1,config_.N) >= -50);

    // Equality constraints --> initial state
    opti_.subject_to(X_(casadi::Slice(), 0) == params_.x0);

    // Equality constraints --> discrete-time dynamics
    casadi::Function f = motionModel();

    for (int i = 0 ; i < config_.N ; i++) {
        casadi::MX X_k = X_(casadi::Slice(), i);
        casadi::MX U_k = U_(casadi::Slice(), i);

        std::vector<casadi::MX> input{X_k, U_k};
        opti_.subject_to(X_(casadi::Slice(),i+1) == f(input)[0]);
    }
}


// 1. Convert the robot pose to CasADi DM [x, y, theta]

// 2. Get the active segment from reference.segments_.front()

// 3. Calculate the segment's:
//    - Start position [x, y]
//    - Normalized direction [dx, dy]

// 4. Walk forward through the horizon to get the target speed and heading of every stage

// 5. Update all CasADi parameters using opti_.set_value()
void MPC::updateParameters(const RobotState& current_state, const ReferencePath2D& reference) {
    if (reference.segments_.empty())
        throw std::invalid_argument("Empty reference path");

    const auto& pose = current_state.pose_;
    if (!std::isfinite(pose.position_.x_) ||
        !std::isfinite(pose.position_.y_) ||
        !std::isfinite(pose.heading_))
        throw std::invalid_argument("updateParameters : Invalid robot state");
    if (current_state.header_.frame_id_ != reference.frame_id_)
        throw std::runtime_error("updateParameters: Inconsistent frames");
    // Current robot state [x, y, theta]
    casadi::DM x0 = casadi::DM::zeros(3,1);
    x0(0) = current_state.pose_.position_.x_;
    x0(1) = current_state.pose_.position_.y_;
    x0(2) = current_state.pose_.heading_;

    opti_.set_value(params_.x0, x0);

    // Active reference segment (single-segment MPC for now)
    const ReferenceSegment2D& active_seg_ = reference.segments_.front();

    Point<double> A = active_seg_.segment_.start_;
    Point<double> B = active_seg_.segment_.end_;

    double length = distance(A, B);

    if (!std::isfinite(length) || length <= 1e-8)
        throw std::invalid_argument("Invalid reference segment length");

    if (!std::isfinite(active_seg_.max_speed_) || active_seg_.max_speed_ <= 0.0)
        throw std::invalid_argument("Invalid reference speed");

    // Segment start position
    casadi::DM seg_start_ = casadi::DM::zeros(2,1);
    seg_start_(0) = A.x_;
    seg_start_(1) = A.y_;
    opti_.set_value(params_.start, seg_start_);

    // Normalized segment direction
    Point<double> dir = B - A;
    normalize(dir);

    casadi::DM dir_dm = casadi::DM::zeros(2,1);
    dir_dm(0) = dir.x_;
    dir_dm(1) = dir.y_;
    opti_.set_value(params_.direction, dir_dm);

    // Target speed and heading of every stage, see the comment above buildObjective() for why.
    casadi::DM vref = casadi::DM::zeros(1, config_.N);
    casadi::DM theta_ref = casadi::DM::zeros(1, config_.N + 1);

    const double seg_heading = std::atan2(dir.y_, dir.x_);

    // Start from the robot itself. The heading is never wrapped, so starting from it keeps theta_ref close to the robot's angle.
    double s = std::max(0.0, dotProduct2D(pose.position_ - A, dir));
    double heading = pose.heading_;

    for (int k = 0 ; k <= config_.N ; k++) {
        double target_speed = 0.0;
        double heading_error = std::atan2(std::sin(seg_heading - heading), std::cos(seg_heading - heading));

        if (std::abs(heading_error) > config_.heading_aligned_tolerance) {
            // Rotate stage: stay in place and turn the target heading gradually toward the segment.
            double step = std::min(std::abs(heading_error), config_.rotate_speed * config_.Ts);
            heading += heading_error > 0.0 ? step : -step;
        }
        else {
            // Drive stage: brake toward the endpoint, but keep creeping until the robot arrives.
            heading += heading_error;
            double remaining = std::max(0.0, length - s);
            if (remaining > 1e-3) {
                target_speed = std::min(
                    active_seg_.max_speed_,
                    std::max(config_.min_creep_speed, std::sqrt(2.0 * config_.a_brake * remaining))
                );
            }
        }

        theta_ref(k) = heading;
        if (k < config_.N)
            vref(k) = target_speed;

        s += target_speed * config_.Ts;
    }

    opti_.set_value(params_.vref, vref);
    opti_.set_value(params_.theta_ref, theta_ref);
}

ControlResult MPC::solve() {
    ControlResult result;

    result.command_.header_.frame_id_ = "base_link";
    result.command_.header_.stamp_ = std::chrono::system_clock::now();

    try {
        // Apply warm-start guesses from the previous optimization cycle
        opti_.set_initial(X_, X_guess_);
        opti_.set_initial(U_, U_guess_);
        // Solve optimization problem
        casadi::OptiSol sol = opti_.solve();

        casadi::DM U_opt = sol.value(U_);
        casadi::DM u_apply = U_opt(casadi::Slice(), 0);

        // First optimal control input [v, w]
        result.command_.twist_.linear_x_ = static_cast<double>(u_apply(0));
        result.command_.twist_.angular_z_ = static_cast<double>(u_apply(1));
        result.status_ = ControlStatus::SUCCESS;


        // Shift optimal controls by one timestep for the next optimization cycle
        for (int i = 0 ; i < config_.N-1 ; i++) {
            U_guess_(0,i) = U_opt(0,i+1);
            U_guess_(1,i) = U_opt(1,i+1);
        }
        U_guess_(0, config_.N-1) = 0;
        U_guess_(1, config_.N-1) = 0;

        // Shift optimal states by one timestep
        casadi::DM X_opt = sol.value(X_);
        for (int i = 0; i < config_.N; i++) {
            X_guess_(casadi::Slice(), i) = X_opt(casadi::Slice(), i+1);
        }
        // Repeat the terminal state
        X_guess_(casadi::Slice(), config_.N) = X_opt(casadi::Slice(), config_.N);
    }
    catch (const std::exception& e) {
        std::cerr << "MPC solver failed: " << e.what() << '\n';
        // Solver failure: return zero velocity
        result.command_.twist_.linear_x_ = 0.0;
        result.command_.twist_.angular_z_ = 0.0;
        result.status_ = ControlStatus::FAILURE;
    }

    return result;
}

void MPC::reset() {
    warm_start_valid_ = false;

    X_guess_ = casadi::DM::zeros(3, config_.N+1);
    opti_.set_initial(X_, X_guess_);

    U_guess_ = casadi::DM::zeros(2, config_.N);
    opti_.set_initial(U_, U_guess_);
}