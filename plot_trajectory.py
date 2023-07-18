import os
import pickle
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("tkAgg")
NUMBER = 0
PATH = "/home/bagr/PycharmProjects/ObjectTrackingDataset/dataset/trajectory_" + str(NUMBER)
def import_plan(name="robot_trajectory.p"):
    file_path = os.path.join(PATH, name)
    with open(file_path, 'rb') as file_open:
        loaded_plan = pickle.load(file_open)
        # loaded_plan = yaml.load(file_open)
    return loaded_plan

def export_plan(plan, name = "reduced_plan.p"):
    file_path = os.path.join(PATH, name)
    with open(file_path, 'wb') as file_save:
        pickle.dump(plan, file_save)
        #yaml.dump(plan, file_save, default_flow_style=True)

def main():
    plan = import_plan("semi_reduced_robot_trajectory.p")
    #reduced_plan = []
    #print(plan.joint_trajectory.points[0])
    #print(plan.joint_trajectory.points[0].positions)
    #print(plan.joint_trajectory.points[0].time_from_start.nsecs)
    #print(plan.joint_trajectory.points[0].positions)
    #print(type(plan.joint_trajectory.points[0].positions))
    pos0 = []
    times = []
    for point in plan:
        pos0.append(point["positions"][0])
        times.append(point["nsecs"])
    plt.plot(pos0, times)
    plt.show()

        #reduced_plan.append([point.positions, point.time_from_start.nsecs])
    #export_plan(reduced_plan)
        #print(point.positions)
        #print(point.time_from_start.nsecs)
        #print("---")
    #print(len(plan.joint_trajectory.points))
if __name__ == "__main__":
    main()