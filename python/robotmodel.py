import numpy as np
import math

class Robotmodel:
	def __init__(self, origin, orientation):
		self.links
		self.joints


class RotationMatrix3d:
	def __init__(self, axis=np.asarray([0,0,1])):
		self.data = np.zeros(3)
		self.axis = axis/math.sqrt(np.dot(axis, axis))

	def rotation(theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    a = math.cos(theta/2.0)
    b, c, d = -self.axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    lst_of_lst = [[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
           		  [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
           		  [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]]
	for i in range(3):
		for j in range(3):
			self.data[i,j] = lst_of_lst[i][j]     
	return self.data

