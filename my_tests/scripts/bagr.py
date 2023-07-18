import csv

NUMBER = 0
PATH = "/home/bagr/PycharmProjects/ObjectTrackingDataset/dataset/trajectory_" + str(NUMBER)



def import_blender_camera_trajectory():
    with open(PATH + "/blender_camera_trajectory.csv", 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            print(row)

def main():
    import_blender_camera_trajectory()

if __name__ == "__main__":
    main()