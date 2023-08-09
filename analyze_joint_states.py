import csv
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R
import numpy as np

urdf_filename = 'panda.urdf'
model = pin.buildModelFromUrdf(urdf_filename)
data = model.createData()

min_joint_states = [100]*7
max_joint_states = [-100]*7

all_joint_states = []

min_eef_pos = [1000]*3
max_eef_pos = [-1000]*3

min_eef_pose = [1000]*4
max_eef_pose = [-1000]*4
with open('joint_states.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        if line_count == 0:
            #print(f'Column names are {", ".join(row)}')
            #print(row[11:15])
            line_count += 1
        else:
            time = float(row[0])
            joint_states = [float(i) for i in row[1:8]]
            eef_pos = [float(i) for i in row[8:11]]
            eef_pose = [float(i) for i in row[11:15]]
            line_count += 1
            q = np.array(joint_states + [0, 0])
            pin.forwardKinematics(model, data, q)
            all_joint_states.append(np.array(joint_states))
            # for i in range(9):
            #    print(model.names[i], np.array(data.oMi[i].rotation))
            # r = R.from_quat(eef_pose)
            # print(r.as_matrix())
            for state in range(7):
                if joint_states[state] > max_joint_states[state]:
                    max_joint_states[state] = joint_states[state]
                if joint_states[state] < min_joint_states[state]:
                    min_joint_states[state] = joint_states[state]
            for pos in range(3):
                if eef_pos[pos] > max_eef_pos[pos]:
                    max_eef_pos[pos] = eef_pos[pos]
                if eef_pos[pos] < min_eef_pos[pos]:
                    min_eef_pos[pos] = eef_pos[pos]

    print("joint_states:")
    print(np.array(max_joint_states) - np.array(min_joint_states))
    print("max:")
    print(np.array(max_joint_states))
    print("min:")
    print(np.array(min_joint_states))
    print("mean:")
    print(np.mean(all_joint_states, axis=0))
