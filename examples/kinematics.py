#! /usr/bin/python3

from mcmpy import kindyn
from mcmpy import util

from math import pi

end_effector_config = "LINK_1" # 180 deg
voltage = "24"
robot_version = "D"

# Create instance of kinematics based on config files and the end effector
# configuration

kin = kindyn.Kinematics("./config/", # location of the config root folder
                        end_effector_config,
                        voltage,
                        robot_version)

print("FORWARD KINEMATICS")

q   = [0, 0, pi/4, 0, pi/4, 0] # joint positions [rad]
# calculate translation and rotation of tip of kinematic chain
# both translation and rotation are from base frame to tool frame expressed in
# the base frame
xyz, rot = kin.calculate_forward(q) # both return values are lists
print("Translation x, y, z [m]:\n", xyz)
print("Rotation matrix [-]:\n", util.list_to_m3x3(rot))

print("\nINVERSE KINEMATICS")

print("No inital guess provided (slower)")
q = kin.calculate_inverse(xyz, rot, None)
print("Joint angles [deg]:\n", util.rad_to_deg(q))

print("With inital guess provided (faster)")
q_init = [0]*6 # inital guess for solver e.g. for tracking
q = kin.calculate_inverse(xyz, rot, q_init)
print("Joint angles [deg]:\n", util.rad_to_deg(q))

# neither version of IK is guaranteed to find a solution
