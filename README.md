# drive_gazebo_plugins
Gazebo-ROS plugins used by the drive team. Included plugins:

## Sign_label_plugin
Camera plugin used to generate labeled ground truth for the detection of traffic signs in images.
It is intended to be added to .sdf files generated by [drive_sim_road_generation](https://github.com/tum-phoenix/drive_sim_road_generation). 
The output format is folder of image files and a .csv file containing the bounding boxes of signs in each of the images.
Multiple signs in one image are handled by adding multiple rows for the same image in the .csv.

### Pipeline for generating ground truth data
1. Install Gazebo 8

   **NOTE TO ROS USERS:** Gazebo 7 is usually installed with the ros-*distribution*-full packages (including *kinetic*). 
   Use the following commands to remove it and switch to Gazebo 8 (replace *kinetic* by your distribution) after [adding the
   LCAS package repository to your sources](https://github.com/LCAS/rosdistro/wiki#using-the-l-cas-repository-if-you-just-want-to-use-our-software)
   ```bash
   sudo apt-get remove ros-kinetic-gazebo*
   sudo apt-get update
   sudo apt-get install ros-kinetic-gazebo8*
   ```
2. Clone [drive_sim_road_generation](https://github.com/tum-phoenix/drive_sim_road_generation) and follow the instructions there
   to generate a .sdf file that can be used in Gazebo 8. 
   
   **NOTE: the keyframe_plugin build location has to remain in your GAZEBO_PLUGIN_PATH at all times when generating labels.**
   
3. Replace the 'filename' string in scripts/insert_label_plugin.py by the location of the generated .sdf file and run it 
   
4. Replace the <output_folder> attribute in the .sdf file by the desired output folder of the frames and the label .csv file.

5. Launch the simulation with:
   ```bash
   roslaunch drive_gazebo_plugins sign_label_plugin.launch world:=*your_sdf_file_location*
   ```
   replacing *your_sdf_file_location* with the full path to the edited .sdf file.
   
   **NOTE: Currently, the plugin will not start before there is at least one subscriber to the camera topic. Therefore, it is advised
   to run a roscore in advance and subscribe anything to that topic (even a simple listener, rviz or image_view is sufficient)**
   
6. *Optional:* you can verify the generated images by replacing the 'source_folder' string in scripts/draw_bounding_boxes.py and running it.
   This will provide a simple visualization of the bounding boxes, press any button to skip and move forward.
