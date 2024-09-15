# 3D-thermal-model
Development of the 3D thermal model, which given the three dimwnsional thermal data of the any 3D object.Here the TOF and thermal camera is used to get the point cloud and thermal data.The both data should be in sync and used to develop the 3d thermal model.
The things required for project:

Intel Real Sense camera
OpenMV based Thermal Camera
Knowledge of open3D, Opencv , Tkinter, PIL, VTK , numpy and threading.
Thermal model
The above image shows the methodology of the project:

The Intel RealSense camera collects RGB images and depth data.
From the depth data, the 3D coordinates of the points can be detected.
Using Open3D, 3D meshes can be developed.
While capturing the images, thermal images are also captured. The capture should be in sync.
The thermal images and 3D model are integrated.
VTK is used to visualize the 3D model.
OpenCV is used to process the thermal images and develop the video.
Threading is used to achieve parallelism.
