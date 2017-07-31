#pragma once

#include "typedef.hpp"

#include <Eigen/Dense>

namespace robo {

    class Twist{
    /*
     * 6D Spatial Motion Vector
     */
    public:

        // Members
        Vector3d linear;
        Vector3d rotation;

        // Constructors
        explicit Twist(const Vector3d& lin, const Vector3d& rot): linear(lin), rotation(rot){};
        
        explicit Twist(const Vector3d& lin): linear(lin), rotation(Vector3d(0.0, 0.0, 0.0)){};
        
        Twist(): linear(Vector3d(0.0, 0.0, 0.0)), rotation(Vector3d(0.0, 0.0, 0.0)){};

        Twist cross(const Twist& other){
            return Twist(rotation.cross(other.linear) + linear.cross(other.rotation),
                         rotation.cross(other.rotation));
        }

    };

    inline Twist change_twist_reference(const Twist& twist, const Vector3d& delta_ref){
        return Twist(twist.linear + twist.rotation.cross(delta_ref), twist.rotation);
    };

    inline Twist operator +(const Twist& lhs, const Twist& rhs){
        return Twist(lhs.linear+rhs.linear, lhs.rotation+rhs.rotation);
    };

    inline Twist operator *(const Matrix3d& rot, const Twist& twist){
        return Twist(rot * twist.linear, rot * twist.rotation);
    };

    inline Twist operator *(const double& val, const Twist& twist){
        return Twist(twist.linear*val, twist.rotation*val);
    };

    inline Twist operator *(const Twist& twist, const double& val){
        return Twist(twist.linear*val, twist.rotation*val);
    };

}
