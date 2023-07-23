import os
import pickle
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("tkAgg")
NUMBER = 0
#PATH = "C:\\Users\\Vojta\\PycharmProjects\\ObjectTrackingDataset\\dataset\\trajectory_" + str(NUMBER)
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
    #reduced_plan = []m
    #print(plan.joint_trajectory.points[0])
    #print(plan.joint_trajectory.points[0].positions)
    #print(plan.joint_trajectory.points[0].time_from_start.nsecs)
    #print(plan.joint_trajectory.points[0].positions)
    #print(type(plan.joint_trajectory.points[0].positions))
    joint_pos = []
    joint_vel = []
    joint_acc = []
    times = []

    last_time = 0
    time_sum = 0
    for point in plan:

        if last_time > point["nsecs"] / 1000000000:
            time_sum += 1
        times.append(time_sum + point["nsecs"] / 1000000000)
        print(time_sum + point["nsecs"] / 1000000000)
        last_time = point["nsecs"] / 1000000000

    for i in range(7):
        single_joint = []
        for point in plan:
            print(point["positions"][i])
            single_joint.append(point["positions"][i])
        joint_pos.append(single_joint)

    for i in range(7):
        single_joint = []
        for point in plan:
            single_joint.append(point["velocities"][i])
        joint_vel.append(single_joint)

    for i in range(7):
        single_joint = []
        for point in plan:
            single_joint.append(point["accelerations"][i])
        joint_acc.append(single_joint)

    fig, axs = plt.subplots(7, 3)
    for i in range(7):
        axs[i, 0].plot(times, joint_pos[i])

    for i in range(7):
        axs[i, 1].plot(times, joint_vel[i])
    for i in range(7):
        axs[i, 2].plot(times, joint_acc[i])
    axs[0, 0].set_title("positions")
    axs[0, 1].set_title("velocities")
    axs[0, 2].set_title("accelerations")
    #plt.plot(times, pos0)
    plt.show()

        #reduced_plan.append([point.positions, point.time_from_start.nsecs])
    #export_plan(reduced_plan)
        #print(point.positions)
        #print(point.time_from_start.nsecs)
        #print("---")
    #print(len(plan.joint_trajectory.points))
if __name__ == "__main__":
    main()