#include "cleaner/control/mpc.hpp"
#include <casadi/casadi.hpp>
#include <casadi/core/dm_fwd.hpp>
#include <casadi/core/function.hpp>

int main(int argc, char* argv[]) {
    std::cout<<"HELLO WORLD \n";
    MPC mpc_;
    mpc_.multipleShooting();

    // casadi::DM current_state_ =  casadi::DM::vertcat({0,0,0});
    // casadi::DM current_control_ =  casadi::DM::vertcat({1,0});

    // casadi::Function f = mpc_.dynamics();
    // std::vector<casadi::DM> input = {current_state_, current_control_};

    // std::vector<casadi::DM> result = f(input);
    // std::cout<<"Result : " << result[0]<<"\n";

    
}