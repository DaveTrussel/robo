#include "dynamics.hpp"

#include <Eigen/Dense>
#include <iostream>

namespace robo{

    // Constructors
    Dynamics::Dynamics(const Chain& chain_): joint_torques(chain_.nr_joints), chain(chain_), nr_links(chain_.nr_links), nr_joints(chain_.nr_joints),
        trans_from_parent(nr_links), motion_subspace(nr_links), velocities(nr_links), accelerations(nr_links), wrenches(nr_links){}

    int Dynamics::calculate_torques(const VectorXd& q_, const VectorXd& dq_, const VectorXd& ddq_,
                                    const std::vector<Wrench>& wrenches_extern,
                                    const Vector3d& gravity_){
        /*
         * Recursive newton euler algorithm (based on "Rigid body dynamics algorithms" by R.Featherstone, 2007, ISBN 978-0-387-74314-1)
         */
        double q, dq, ddq;
        Twist joint_twist;
        Matrix3d to_link_rot;
        Frame trans_to_parent;

        Twist gravity{gravity_, Vector3d(0.0, 0.0, 0.0)};

        // From root to tip (Forward Recursion)
        // Through the kinematic relationships this calculates the pos, vel, acc of each link
        int iter_joint = 0;
        for(int iter_link=0; iter_link<chain.nr_links; ++iter_link){
            if(chain.links[iter_link].has_joint()){
                q = q_[iter_joint];
                dq = dq_[iter_joint];
                ddq = ddq_[iter_joint];
                ++iter_joint;
            }
            else{
                q = dq = ddq = 0.0;
            }

            // calculate frame of current link
            Link& link = chain.links[iter_link];
            Joint& joint = link.joint;
            trans_from_parent[iter_link] = link.pose(q); // trans_to_parent from parent to child, gives the frame of the tip of current link expressed in current link root coordinates
            trans_to_parent = trans_from_parent[iter_link].inverse(); // trans_to_parent from child to parent expressed in the the current link root frame 

            // calculate twist and motion in current link frame
            joint_twist = joint.twist(dq);  // twist at end of link expressed in root frame
            motion_subspace[iter_link] = joint.motion_subspace(); // joint motion subspace expressed in root frame

            // calculate velocity and acceleration in current link frame
            if(iter_link==0){
                velocities[iter_link]    = joint_twist;
                accelerations[iter_link] = trans_to_parent * gravity +
                                           motion_subspace[iter_link] * ddq +
                                           multiply_twists(velocities[iter_link], joint_twist);
            }
            else{
                velocities[iter_link]    = trans_to_parent * velocities[iter_link-1] + joint_twist;
                accelerations[iter_link] = trans_to_parent * accelerations[iter_link-1] +
                                           motion_subspace[iter_link] * ddq +
                                           multiply_twists(velocities[iter_link], joint_twist);
            }

            // calculate wrench acting on current link
            Inertia& inertia = link.inertia;
            wrenches[iter_link] = inertia * accelerations[iter_link] +
                                  velocities[iter_link] * (inertia * velocities[iter_link]) -
                                  wrenches_extern[iter_link];
            // TODO remove:: std::cout << "Forward Prop: Link " << iter_link << ": \t" << wrenches[iter_link].force.transpose() << ", " << wrenches[iter_link].torque.transpose() <<  std::endl;
        }

        // Back from tip to root (Backward recursion)
        // Based on the pos, vel, acc this now calculates the wrench accting on each link
        iter_joint = chain.nr_joints;
        for(int iter_link=chain.nr_links-1; iter_link>=0; --iter_link){
            if(chain.links[iter_link].has_joint()){
                // calculate torque (projection of wrench onto joint axis)
                double result = wrenches[iter_link].dot(motion_subspace[iter_link]);
                // TODO remove: std::cout << "Joint torque: " << result << std::endl;
                joint_torques[--iter_joint] = result;
            }
            if(iter_link>0){
                // add wrench of this link to lower link's wrench (trans_to_parent into lower frame)
                wrenches[iter_link-1] += trans_from_parent[iter_link]*wrenches[iter_link];;
                // TODO remove std::cout << "Backward Prop: Link " << iter_link-1 << ": \t" << wrenches[iter_link-1].force.transpose() << ", " << wrenches[iter_link-1].torque.transpose() <<  std::endl;
            }
            

        }
        return 1;
    }
    
}

