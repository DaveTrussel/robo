#pragma once

#include "frame.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>


namespace robo {

    enum class Joint_type { None, Rotational, Translational };

    class Joint{
    public:
        // Members
        int id;
        Joint_type type;
        Vector3d axis;
        Frame frame;
        struct {
            double rotor_inertia            = 0.0;
            double linear_friction_coeff    = 0.0;
            double q_min                    = std::numeric_limits<double>::min();
            double q_max                    = std::numeric_limits<double>::max();
        } parameters;

        // Constructors
        Joint(const int id_in, const Frame frame_in, const Vector3d axis_in, 
              const Joint_type type_in=Joint_type::Rotational,
              double q_min_in=std::numeric_limits<double>::min(),
              double q_max_in= std::numeric_limits<double>::max()):
            id(id_in), type(type_in), axis(axis_in.normalized()), frame(frame_in){
                parameters.q_min = q_min_in;
                parameters.q_max = q_max_in;
            };

        // Public member functions
        Frame pose(const double& q)const{
        /*
         * Calculates the joint transformation from parent to successor represented in the joint root frame
         */
            if(type == Joint_type::Rotational){
                return Frame(frame.origin, rotate(q));
            }
            if(type == Joint_type::Translational){ 
                return Frame(frame.origin + q*axis, frame.orientation);
            }
            else{
                return Frame(frame);
            }
        };

        Twist twist(const double &dq)const{
        /*
         * Calculates the 6D velocity of the joint represented in the joint root frame
         */
            Vector3d speed_lin = Vector3d::Zero();
            Vector3d speed_rot = Vector3d::Zero();
            if(type == Joint_type::Rotational){
                speed_rot << dq*axis;
            }
            if(type == Joint_type::Translational){
                speed_lin << dq*axis;
            }
            return Twist(speed_lin, speed_rot);
        };

        Twist motion_subspace()const{
        /*
         * Returns the subspace of motion for the given joint type represented in the joint root frame
         */
            return twist(1.0);
        };

        void set_joint_limits(const double& q_min, const double& q_max){
            parameters.q_min = q_min;
            parameters.q_max = q_max;
        };

    private:
        Matrix3d rotate(const double& angle)const{
        /*
         * Calculates the rotation matrix based on the Euler-Rodrigues formula (rotation around axis by angle)
         */
            double theta = angle/2.0;
            double a = std::cos(theta);
            double sine = std::sin(theta);
            double b = -sine * axis[0];
            double c = -sine * axis[1];
            double d = -sine * axis[2];
            double aa = a*a;
            double bb = b*b;
            double cc = c*c;
            double dd = d*d;
            double bc = b*c;
            double ad = a*d;
            double ac = a*c;
            double ab = a*b;
            double bd = b*d;
            double cd = c*d;
            Matrix3d rot;
            rot << aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac),
                   2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab),
                   2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc;
            return rot; 
        };
    };

}
