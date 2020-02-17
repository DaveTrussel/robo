#include "robo/dynamics.hpp"

#include "robo/body.hpp"
#include "robo/inertia.hpp"
#include "robo/joint.hpp"

#include <ext/alloc_traits.h>
#include <memory>

namespace robo{

// Constructors
Dynamics::Dynamics(const Chain& chain):
  chain_(chain),
  num_bodies_(chain.num_bodies),
  num_joints_(chain.num_joints),
  generalized_forces(chain.num_joints),
  transform_base_to_body(chain.num_bodies),
  transform_to_child(chain.num_bodies),
  motion_subspace(chain.num_bodies),
  velocities(chain.num_bodies),
  accelerations(chain.num_bodies),
  wrenches(chain.num_bodies),
  friction_forces(chain.num_bodies),
  joint_inertia_forces(chain.num_bodies){}

// Methods
void Dynamics::calculate_inverse_dynamics(const VectorXd& q_in,
                                          const VectorXd& dq_in,
                                          const VectorXd& ddq_in,
                                          const std::vector<Force>& wrenches_extern,
                                          const Vector3d& gravity_in){
  std::vector<double> q(num_bodies()), dq(num_bodies()), ddq(num_bodies());
  Motion joint_twist;
  Motion gravity{Vector3d(0.0, 0.0, 0.0), gravity_in};

  // From root to tip (Forward Recursion)
  // Through the kinematic relationships this calculates the pos, vel, acc of each body
  int iter_joint = 0;
  for(int iter_body=0; iter_body<num_bodies(); ++iter_body){
    if(chain_.body(iter_body).has_joint()){
        q[iter_body] =   q_in[iter_joint];
       dq[iter_body] =  dq_in[iter_joint];
      ddq[iter_body] = ddq_in[iter_joint];
      ++iter_joint;
    }
    else{
      q[iter_body] = dq[iter_body] = ddq[iter_body] = 0.0;
    }

    // calculate frame of current body
    const Body& body = chain_.body(iter_body);
    transform_to_child.at(iter_body) = body.transform(q[iter_body]);
    // calculate velocity and acceleration in current body frame
    // (at the point where the previous joint is attached)
    if(iter_body==0){
      velocities.at(iter_body)      = Motion(); // first body is base and not moving
      accelerations.at(iter_body)   = -gravity; // common 'trick' to avoid further gravity terms
      friction_forces.at(iter_body) = 0.0;
      transform_base_to_body.at(iter_body) = Transform();
    }
    else{
      // the velocity of the current body is determined by the previous joint;
      const Joint& joint_parent = chain_.body(iter_body-1).joint;
      joint_twist = joint_parent.twist(dq[iter_body-1]);
      friction_forces.at(iter_body) = joint_parent.calculate_friction_force(dq[iter_body-1]);
      joint_inertia_forces.at(iter_body) = joint_parent.calculate_inertia_force(ddq[iter_body-1]);
      motion_subspace.at(iter_body-1) = joint_parent.motion_subspace();
      Transform& XX = transform_to_child.at(iter_body-1);
      transform_base_to_body.at(iter_body) = XX * transform_base_to_body.at(iter_body-1);
      velocities[iter_body]    = XX.apply(velocities.at(iter_body-1)) + joint_twist;
      accelerations[iter_body] = XX.apply(accelerations.at(iter_body-1)) +
                                 motion_subspace.at(iter_body-1) * ddq[iter_body-1] +
                                 velocities[iter_body].cross(joint_twist);
    }

    // calculate wrench acting on current body (this could also be part of the backward recursion)
    const Inertia& inertia = body.inertia;
    wrenches.at(iter_body) = inertia * accelerations.at(iter_body) +
                             velocities.at(iter_body).cross((inertia * velocities[iter_body])) -
                             wrenches_extern.at(iter_body);
    // TODO external wrenches now in body coordinates (better in world coordinates?)
  }

  // Back from tip to root (Backward recursion)
  // Based on the pos, vel, acc this calculates the wrench accting on each body
  iter_joint = num_joints();
  for(int iter_body=num_bodies()-1; iter_body>0; --iter_body){
    if(chain_.body(iter_body-1).has_joint()){
      // calculate generalized force (projection of wrench onto joint axis)
      double result = motion_subspace.at(iter_body-1).dot(wrenches.at(iter_body));
      generalized_forces[--iter_joint] =
        result + friction_forces.at(iter_body) + joint_inertia_forces[iter_body];
    }
    if(iter_body>0){
      // add wrench of this body to lower body's wrench
      wrenches.at(iter_body-1) += transform_to_child.at(iter_body-1).apply_inverse(wrenches.at(iter_body));
    }
  }
}

VectorXd Dynamics::calculate_friction_forces(const VectorXd& dq){
  VectorXd ret(num_bodies());
  int j = 0;
  for (auto i=0; i<num_bodies(); ++i) {
    if (chain_.body(i).has_joint()) {
      ret[j] = chain_.body(i).joint.calculate_friction_force(dq[j]);
      ++j;
    }
  }
  return ret;
}

} // namespace robo
