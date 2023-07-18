import os
import pickle

def import_plan(name="tmp_plan.p"):
    file_path = os.path.join("/home/bagr/ws_moveit/src/my_tests/saved_paths", name)
    with open(file_path, 'rb') as file_open:
        loaded_plan = pickle.load(file_open)
        # loaded_plan = yaml.load(file_open)
    return loaded_plan

def export_plan(plan, name = "reduced_plan.p"):
    file_path = os.path.join("/home/bagr/ws_moveit/src/my_tests/saved_paths", name)
    with open(file_path, 'wb') as file_save:
        pickle.dump(plan, file_save)
        #yaml.dump(plan, file_save, default_flow_style=True)

def main():
    plan = import_plan()
    reduced_plan = []
    #print(plan.joint_trajectory.points[0])
    #print(plan.joint_trajectory.points[0].positions)
    #print(plan.joint_trajectory.points[0].time_from_start.nsecs)
    #print(plan.joint_trajectory.points[0].positions)
    #print(type(plan.joint_trajectory.points[0].positions))
    for point in plan.joint_trajectory.points:
        reduced_plan.append([point.positions, point.time_from_start.nsecs])
    export_plan(reduced_plan)
        #print(point.positions)
        #print(point.time_from_start.nsecs)
        #print("---")
    #print(len(plan.joint_trajectory.points))
if __name__ == "__main__":
    main()