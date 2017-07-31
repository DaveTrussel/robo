#pragma once

#include "twist.hpp"

#include <Eigen/Dense>

namespace robo {

    class Wrench{
    /*
     * 6D Force, Torque
     */
    public:
        // Members
        Vector3d force;
        Vector3d torque;

        // Constructors
        explicit Wrench(const Vector3d& force_, const Vector3d& torque_): force(force_), torque(torque_){};
        
        explicit Wrench(const Vector3d& force_): force(force_), torque(Vector3d(0.0, 0.0, 0.0)){};
        
        Wrench(): force(Vector3d(0.0, 0.0, 0.0)), torque(Vector3d(0.0, 0.0, 0.0)){};

        double dot(const Twist& twist){
            return twist.linear.dot(force) + twist.rotation.dot(torque);
        };

        // Operators
        Wrench& operator +=(const Wrench& other){
            force += other.force;
            torque += other.torque;
            return *this;
        }

    };

    inline Wrench operator +(const Wrench& lhs, const Wrench& rhs){
        return Wrench(lhs.force + rhs.force, lhs.torque + rhs.torque);
    };

    inline Wrench operator -(const Wrench& lhs, const Wrench& rhs){
        return Wrench(lhs.force - rhs.force, lhs.torque - rhs.torque);
    };

    inline Wrench operator *(const double& val, const Wrench& wrench){
        return Wrench(val*wrench.force, val*wrench.torque);
    };

    inline Wrench operator *(const Wrench& wrench, const double& val){
        return Wrench(val*wrench.force, val*wrench.torque);
    };

    inline Wrench operator *(const Twist& twist, const Wrench& wrench){
        return Wrench(twist.rotation.cross(wrench.force),
                      twist.rotation.cross(wrench.torque)+twist.linear.cross(wrench.force));
    };

}
