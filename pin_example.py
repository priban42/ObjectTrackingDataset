import numpy as np
import pinocchio as pin
from sys import argv
from os.path import dirname, join, abspath

urdf_filename = 'panda.urdf'
model = pin.buildModelFromUrdf(urdf_filename)
data = model.createData()

#q = pin.randomConfiguration(model)
q = np.array([0.6727893948554993,
-1.7532793283462524,
0.6003391146659851,
-2.653550863265991,
-1.5394641160964966,
0.7872329354286194,
-0.08573910593986511]+[0]*2)
#pin.forwardKinematics()
print('q: %s' % q)
# print(pin.get)
pin.forwardKinematics(model, data, q)
base = pin.SE3(np.eye(4))
# f = self._geom_data.oMg
# self._objects[f'{self.name}/{g.name}'].pose = (base * f).homogeneous

for i in range(10):
    print(model.names[i], data.oMi[i])
print(model.getJointId("panda_hand"))
