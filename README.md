# 3D-thermal-model
Development of the 3D thermal model, which given the three dimwnsional thermal data of the any 3D object.Here the TOF and thermal camera is used to get the point cloud and thermal data.The both data should be in sync and used to develop the 3d thermal model.



The things required for project:

1) Intel Real Sense camera
2) OpenMV based Thermal Camera
3) Knowledge of open3D, Opencv , Tkinter, PIL, VTK , numpy and threading.


![Thermal model](https://github.com/user-attachments/assets/35b82bc5-3efd-4047-ba5c-a1170cee1825)


The above image shows the methodology of the project:

1) The Intel RealSense camera collects RGB images and depth data.
2) From the depth data, the 3D coordinates of the points can be detected.
3) Using Open3D, 3D meshes can be developed.
4) While capturing the images, thermal images are also captured. The capture should be in sync.
5) The thermal images and 3D model are integrated.
6) VTK is used to visualize the 3D model.
7) OpenCV is used to process the thermal images and develop the video.
8) Threading is used to achieve parallelism.
