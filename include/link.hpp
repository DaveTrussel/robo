#pragma once

#include "inertia.hpp"
#include "joint.hpp"

#include <Eigen/Dense>

namespace robo{

    class Link{
    public:
        // Members
        int id;
        Joint joint;     // This is the "root" of each link
        Frame tip;       // This is the end of the link (where the next joint is connected)
        Inertia inertia; // Reference point is the root of the link and also expressed in the root frame

        // Constructors
        Link(int id_, const Joint& joint_, const Frame& tip_, const Inertia& inertia_): id(id_), joint(joint_), tip(joint_.pose(0).inverse()*tip_), inertia(inertia_){};

        // Member functions
        Frame pose(const double& q)const{
        /*
         * Returns the position of the tip of the link expressed in the link root frame (joint frame)
         */
            return joint.pose(q)*tip;
        };
        
        Twist twist(const double& q, const double &dq)const{
        /*
         * Returns the twist of the tip of the link expressed in the link root frame (joint frame)
         */
            Vector3d ref_point = joint.pose(q).orientation * tip.origin;
            return change_twist_reference(joint.twist(dq), ref_point);
        };
        
        bool has_joint()const{
            return joint.type != Joint_type::None;
        };
    };

}   
