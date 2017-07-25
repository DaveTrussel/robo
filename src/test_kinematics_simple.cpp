#include "kinematics.hpp"

#include <iostream>
#include <iomanip>


using namespace robo;
using namespace std;

constexpr auto pi = 3.141592653589793238462643383279502884L;

void run_inverse_kinematics(int method, VectorXd q_target, VectorXd q_init, Kinematics& kin){
    kin.joint_to_cartesian(q_target);
    Frame f_target = kin.f_end;
    Error_type error = Error_type::no_error;
    switch(method){
        case 0: error = kin.cartesian_to_joint_jacobian_transpose(f_target, q_init);
                std::cout << "=== Jacobian transpose ==="       << std::endl;
                break;
        case 1: error = kin.cartesian_to_joint_ccd(f_target, q_init);
                std::cout << "=== Cyclic Coordinate Descent ===" << std::endl;
                break;
        case 2: error = kin.cartesian_to_joint_levenberg(f_target, q_init);
                std::cout << "=== Levenberg-Marquardt ==="       << std::endl;
                break;
        case 3: error = kin.cartesian_to_joint_sugihara(f_target, q_init);
                std::cout << "=== Sugihara ==="                  << std::endl;
                break;
        case 4: error = kin.cartesian_to_joint_sugihara_joint_limits(f_target, q_init);
                std::cout << "=== Sugihara Joint Limits ==="     << std::endl;
                break;
    }
    std::cout << "Error code:               " << (error == Error_type::no_error)    << std::endl;
    std::cout << "Initial joint position:   " << q_init.transpose()                 << std::endl;
    std::cout << "Target joint position:    " << q_target.transpose()               << std::endl;
    std::cout << "Found joint position:     " << kin.q_out.transpose()              << 
                 " (Several valid solutions might exist)"                           << std::endl;
    kin.joint_to_cartesian(kin.q_out);
    Frame f_found = kin.f_end;
    std::cout << "Target frame:           \n" << f_target.origin.transpose()  << "\n" << f_target.orientation << std::endl;
    std::cout << "Found frame:            \n" << f_found.origin.transpose()   << "\n" << f_found.orientation  << std::endl;
}

int main () {
    std::cout << std::fixed << std::setw( 12 ) << std::setprecision(2);
    std::cout << "=== Tests of different inverse kinematic algorithms  ===\n" << 
                 "" << std::endl;

    Vector3d axis_z, axis_y;
    axis_y << 0.0, 1.0, 0.0;
    axis_z << 0.0, 0.0, 1.0;
    
    Frame ff = Frame();

    double q_min = -170.0/180*pi;
    double q_max = 170.0/180*pi;

    Joint joint_ellbow = Joint(0, ff, axis_y, Joint_type::Rotational, q_min, q_max);
    Joint joint_wrist = Joint(0, ff, axis_z, Joint_type::Rotational, q_min, q_max);
    Joint joint_none = Joint(0, ff, axis_y, Joint_type::None);


    Vector3d length;
    length << 0, 0, 1;
    Frame tip = Frame(length);

    Inertia inertia = Inertia(1.0, length/2);

    Link link_0 = Link(0, joint_wrist,  tip, inertia);
    Link link_1 = Link(0, joint_ellbow, tip, inertia);
    Link link_2 = Link(1, joint_ellbow, tip, inertia);

    Chain chain;

    chain.add_link(link_0);
    chain.add_link(link_1);
    chain.add_link(link_2);

    int max_iter = 500;
    Kinematics kin = Kinematics(chain, max_iter);

    VectorXd q_target(chain.nr_joints), q_init(chain.nr_joints);
    double q0 = q_min/3.01;
    double q1 = q_min/3.01;
    double q2 = q_min/3.01;
    q_target << q0*2, q1*2, q2*2;
    q_init << q0, q1, q2;

    kin.joint_to_cartesian(q_init);
    kin.calculate_jacobian(q_init);
    cout << "Jacobian calculated:\n" << kin.jacobian << endl;

    // Analytic solution for jacobian for above 2D case
    MatrixXd J(6,3);
    double q12 = q1 + q2;

    double ss = sin(q1) + sin(q12);

    double a = -sin(q0)*ss;
    double b = cos(q0)*ss;
    double c = 0;

    double d = cos(q0)*(cos(q1) + cos(q12));
    double e = sin(q0)*(cos(q1) + cos(q12));
    double f = -sin(q1) - sin(q12);

    double g = cos(q0)*cos(q12);
    double h = sin(q0)*cos(q12);
    double j = -sin(q12);

    cout << sin(q0) << "," << cos(q0) << endl;

    J <<  a, d, g,
          b, e, h,
          c, f, j,
          0, 0, 0,
          0, 1, 1,
          1, 0, 0;

    cout << "Jacobian analytical:\n" << J << endl;

    cout << "dx=J*dq: " << (kin.jacobian * (q_target-q_init)).transpose() << "\n" << endl;   

    for(int i=0; i<5; ++i){
        run_inverse_kinematics(i, q_target, q_init, kin);
        cout << endl;
    }
}

