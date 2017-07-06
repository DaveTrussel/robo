#pragma once

#include "frame.hpp"
#include "twist.hpp"
#include "wrench.hpp"
#include <Eigen/Dense>

namespace robo{

    class Inertia{
    public:
        /* 
        ** All values must be given with respect to the specified frame
        */
        explicit Inertia(const Frame& frame_, const double& mass_,const Vector3d& com_,const Matrix3d& rot_): 
            frame(frame_), mass(mass_), h(com_*mass_), rot_inertia(rot_){};

        explicit Inertia(const double& mass_,const Vector3d& com_,const Matrix3d& rot_): 
            frame(), mass(mass_), h(com_*mass_), rot_inertia(rot_){};

        explicit Inertia(const double& mass_,const Vector3d& com_): 
            frame(), mass(mass_), h(com_*mass_), rot_inertia(Matrix3d::Zero()){};

        Inertia(): frame(), mass(0.0), h(Vector3d::Zero()), rot_inertia(Matrix3d::Zero()){};

        Wrench operator *(const Twist& t) const {
            return Wrench(mass * t.linear - h.cross(t.rotation), rot_inertia * t.rotation + h.cross(t.linear));
        }

    private:
        Frame frame; // Inertia is always refers to a point / coordinate frame
        double mass;
        Vector3d h; // center of mass times mass TODO find out how this is called (with respect to reference point)
        Matrix3d rot_inertia; // rotational inertia (with respect to reference point)
    };

}

