#! /usr/bin/python3

from mcmpy import kindyn

end_effector_configs = ["LINK_1", # 180 deg
                        "LINK_2", #  90 deg
                        "LINK_3"] # custom gripper

voltage = "24"
robot_version = "D"

for end_effector_config in end_effector_configs:
  # Create instance of dynamic model based on config files and the end effector
  # configuration

  dynamics = kindyn.Dynamics("./config/",
                             end_effector_config,
                             voltage,
                             robot_version)

  # Desired motion profile
  q   = [0.1]*6           # joint positions     [rad]
  dq  = [0.1]*6           # joint velocities    [rad/s]
  ddq = [0.1]*6           # joint accelerations [rad/s^2]
  g   = [0.0, 0.0, -9.81] # gravity             [m/s^2]

  # in SI-units (forces [N] / torques [Nm])
  generalized_forces = dynamics.calculate_inverse_dynamics(q, dq, ddq, g)

  print("Calculated generalized forces for {0} configuration are:".format(end_effector_config))
  print(generalized_forces, '\n')

