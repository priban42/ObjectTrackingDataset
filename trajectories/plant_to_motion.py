import os
import pickle

def import_plan(name="reduced_plan.p"):
    #file_path = os.path.join("/home/bagr/ws_moveit/src/my_tests/saved_paths", name)
    file_path = os.path.join(os.path.dirname(__file__),"saved_paths", name)
    with open(file_path, 'rb') as file_open:
        loaded_plan = pickle.load(file_open)
    return loaded_plan

plan = import_plan()
for point in plan:
    print(point)
    #for joint in range(7):

        #robot[joint] = point[0][joint]