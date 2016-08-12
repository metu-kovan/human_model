ROS Package for Drawing Human Model in RViz 

#Installation
Copy the files under the ROS workspace

#Example Execution
-Run Singeleton Server
-roslaunch icub_arm_imitator imitator_online.launch
-roslaunch human_model human_model.launch (model:=female2)
You can change model with directory name under urdf directory(default female2)

#Help
For source code You can use the following
>python  
>import animator  
>help(animator) help(animator.JointTree) etc.  

#URDF Directory Structure
Every directory indicates a different model. General structure of directory is:  
__meshes directory:__ contains Collada files used in models.xacro  
__tree.json :__ contains tree structure needed for animator.py .If a value is omitted,default
value will be used. Default values can be found with help(animator.JointTree.\_\_init\_\_)  
__model.xacro:__ evaluated to create urdf file used by RViz by launch file.General macros used for
multiple models are:  
<xacro:link_to_world link="link name" mesh="name of the mesh" scale="scale of the mesh"/>  
<xacro:link_without_geo link="link name"/>  
<xacro:connector link="link name" mesh="name of the mesh" scale="scale of the mesh"/>  
<xacro:connector_without_geo link="link name"/>  
