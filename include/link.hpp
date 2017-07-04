#pragma once

#include "inertia.hpp"
#include "joint.hpp"
#include <Eigen/Dense>

namespace robo{

    class Link{
    public:
        // Members
        int id;
        Joint joint;
        Frame tip;
        Inertia inertia;

        // Constructors
        Link(int id_, const Joint& joint_, const Frame& tip_, const Inertia& inertia_): id(id_), joint(joint_), tip(tip_), inertia(inertia_){};

        // Member functions
        Frame pose(const double& q)const{
        return joint.pose(q)*tip;
        };
        
        Twist twist(const double& q, const double &dq)const{
            Vector3d ref_point = joint.pose(q).orientation * tip.origin;
            return change_twist_reference(joint.twist(dq), ref_point);
        };
        
        bool has_joint()const{
            return joint.type != JointType::None;
        };
    };

}   
