# general information
- All the files were created using blender 3.3.0
- for Python debugging a console can be opened in the "Window->Toggle System Console" menu

# camera_trajectory_generator.blend
This file is used for generating a camera trajectory. It exports a csv file containing the camera 6D pose for each frame.
For that, there is a script named "save_camera_trajectory.csv", (which can be accessed using the blender text editor viewport)

It is necessary to set an absolute path for the exportation.

## generating the trajectory
- in the Timeline viewport choose a desired frame
- move the camera toa desired location (using the G key or the "item->Transform" menu) and rotate it (using the R key or the "item->Transform" menu)
- make sure that "Quaternion (WXYZ)" is selected in the "item->Transform->Rotation mode" menu.
- press the I key.
- select "Location & Rotation".
- Repeat all previous steps until the desired trajectory is completed.
- (make sure the correct output directory is set. You can choose a different NUMBER value for different trajectories for convenience)
- press the "Run Script" button found in the Text editor viewport

![blender trajectory generator description](blender_trajectory_generator_description.PNG?raw=true)

# import_trajectory.blend
This file is used for importing a joint-space trajectory for the panda robot generated in moveit. It can be also used for rendering the trajectory.
It generates a file structure including pictures, joint states for each frame, object poses. (This part will be reworked to satisfy the [bop dataset format](https://github.com/thodan/bop_toolkit/blob/master/docs/bop_datasets_format.md) )
It is meant to render multiple combinations of scenes and trajectories at different speeds.

## replaying the trajectory
- in the replay_trajectory script choose the correct TRAJECTORY_PATH and RENDER_PATH.
- in the render_all() function set the desired ranges in the for loops.
- press the "Run Script" button found in the Text editor viewport