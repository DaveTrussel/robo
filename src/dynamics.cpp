#include "../include/dynamics.hpp"
#include <Eigen/Dense>
#include <iostream>

namespace robo{

    // Constructors
    Dynamics::Dynamics(const Chain& chain_): joint_torques(chain_.nr_joints), chain(chain_), nr_links(chain_.nr_links), nr_joints(chain_.nr_joints),
        link_frames(nr_links), unit_twists(nr_links), velocities(nr_links), accelerations(nr_links), wrenches(nr_links){}

    int Dynamics::calculate_torques(const VectorXd& q_, const VectorXd& dq_, const VectorXd& ddq_,
                                    const std::vector<Wrench>& wrenches_extern,
                                    const Vector3d& gravity_){
        // Recursive newton euler algorithm
        double q, dq, ddq;
        Twist link_twist;
        Matrix3d to_link_rot;
        Inertia inertia;
        Frame transform;

        Twist gravity{gravity_, Vector3d(0.0, 0.0, 0.0)};

        // From root to tip
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
            link_frames[iter_link] = link.pose(q); // gives the frame of the tip of current link expressed in current link coordinates
            transform = link_frames[iter_link].inverse(); // Used to transform everything to the link root frame

            // calculate twist and unit twist in current link frame
            to_link_rot = transform.orientation;
            link_twist = rotate_twist(to_link_rot, link.twist(q, dq));
            unit_twists[iter_link] = rotate_twist(to_link_rot, link.twist(q, 1.0));

            // calculate velocity and acceleration in current link frame
            if(iter_link==0){
                velocities[iter_link] = link_twist;
                // TODO mistake here! as below
                accelerations[iter_link] = transform * gravity +
                                           unit_twists[iter_link] * ddq +
                                           multiply_twists(velocities[iter_link], link_twist);
            }
            else{
                // TODO mistake here! this should be frame.inverse() * twist instead of rotation * twist
                velocities[iter_link] = transform * velocities[iter_link-1] + link_twist;
                accelerations[iter_link] = transform * accelerations[iter_link-1] +
                                           accelerations[iter_link-1] * ddq +
                                           multiply_twists(velocities[iter_link], link_twist);
            }

            // calculate wrench acting on current link
            inertia = chain.links[iter_link].inertia;
            wrenches[iter_link] = inertia * accelerations[iter_link] +
                                  velocities[iter_link] * (inertia * velocities[iter_link]) -
                                  wrenches_extern[iter_link];
            std::cout << "Forward Prop: Link " << iter_link << ": \t" << wrenches[iter_link].force.transpose() << ", " << wrenches[iter_link].torque.transpose() <<  std::endl;
        }

        // Back from tip to root
        iter_joint = chain.nr_joints;
        for(int iter_link=chain.nr_links-1; iter_link>=0; --iter_link){
            if(chain.links[iter_link].has_joint()){
                // calculate torque (unit twist * wrench)
                std::cout << "Unit twist " << iter_link << ": " << unit_twists[iter_link].linear.transpose() << ", " << unit_twists[iter_link].rotation.transpose() << std::endl;
                double result = wrenches[iter_link].dot(unit_twists[iter_link]);
                std::cout << "Joint torque: " << result << std::endl;
                joint_torques[--iter_joint] = result;
            }
            if(iter_link>0){
                // add wrench of this link to lower link's wrench (transform into lower frame)
                std::cout << link_frames[iter_link].origin.transpose() << "\n" <<link_frames[iter_link].orientation << std::endl;
                Wrench delta_parent = link_frames[iter_link]*wrenches[iter_link];
                std::cout << "From parent " << iter_link << ": \t" << delta_parent.force.transpose() << ", " << delta_parent.torque.transpose() <<  std::endl;
                std::cout << "From self   " << iter_link-1 << ": \t" << wrenches[iter_link-1].force.transpose() << ", " << wrenches[iter_link-1].torque.transpose() <<  std::endl;
                std::cout << "Add up to   " << iter_link-1 << ": \t" << (wrenches[iter_link-1]+delta_parent).force.transpose() << (wrenches[iter_link-1]+delta_parent).torque.transpose() << std::endl;
                wrenches[iter_link-1] += delta_parent;
                std::cout << "Backward Prop: Link " << iter_link-1 << ": \t" << wrenches[iter_link-1].force.transpose() << ", " << wrenches[iter_link-1].torque.transpose() <<  std::endl;
            }
            

        }
        return 1;
    }
    
}

