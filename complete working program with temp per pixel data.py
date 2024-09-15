import Demos.mmapfile_demo
import serial  # pip install pyserial
import keyboard  # pip install keyboard
import pyrealsense2 as rs
import open3d as o3d  # pip install open3d
import numpy as np  # pip install numpy
import time
from tkinter import font as tkfont
import tkinter.filedialog
import tkinter as tk
from PIL import ImageTk
from PIL import Image
import ast
from tkinter.ttk import Progressbar
import cv2
import time
import subprocess
import threading
import vtk


class Arduino():

    def __init__(self, comPort, baudRate, timeout):
        self.comPort = comPort
        self.baudRate = baudRate
        self.timeout = timeout
        self.totalAngle = 0
        self.s = serial.Serial(self.comPort, self.baudRate, timeout=self.timeout)
        time.sleep(2)
        self.currentstep = 0

    def rotate(self, steps):
        self.steps = steps
        self.negative = int(self.steps < 0)
        if self.negative:
            self.steps = -self.steps
        self.stepsAsString = str(self.steps)
        self.s.write([self.negative])
        for i in range(5 - len(self.stepsAsString)):
            self.s.write([0])
        for i in range(len(self.stepsAsString)):
            self.s.write([int(self.stepsAsString[i])])

    def waitForRotation(self):
        while True:
            self.data = str(self.s.readline())
            if self.data != "b''":
                self.currentstep += self.steps
                if int(str(self.data)[2:len(self.data) - 1]) != self.steps:
                    self.close()
                    raise Exception("The Arduino returned the wrong number of steps!")
                break

    def giveAngle(self):
        self.gearRatio = 6
        self.currentAngle = ((self.currentstep * 360) / 2048) / self.gearRatio
        return self.currentAngle

    def close(self):
        self.s.close()

class Scan():
    def __init__(self, width, height, framerate, autoexposureFrames, backDistance):

        self.width = width
        self.height = height
        self.framerate = framerate
        self.backDistance = backDistance
        # print("backdistance:", backDistance)
        self.autoexposureFrames = autoexposureFrames
        self.main_pcd = o3d.geometry.PointCloud()

        self.pipe = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.any, self.framerate)
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.any, self.framerate)

        # post-processing filters
        self.depth_to_disparity = rs.disparity_transform(True)
        self.disparity_to_depth = rs.disparity_transform(False)
        self.dec_filter = rs.decimation_filter()
        self.temp_filter = rs.temporal_filter()
        self.spat_filter = rs.spatial_filter()
        self.hole_filter = rs.hole_filling_filter()
        self.threshold = rs.threshold_filter(0.17, 0.4)

        self.dtr = np.pi / 180
        self.distance = 0.258 - self.backDistance
        self.bbox = o3d.geometry.AxisAlignedBoundingBox((-0.13, -0.13, 0), (0.13, 0.13, 0.2))

    def startPipeline(self):
        self.pipe.start(self.config)
        self.align = rs.align(rs.stream.color)
        print("pipeline gestart")

    def stopPipeline(self):
        self.pipe.stop()
        self.pipe = None
        self.config = None
        print("pipeline gestopt")

    def takeFoto(self):
        print("foto gemaakt!")
        for i in range(self.autoexposureFrames):
            self.frameset = self.pipe.wait_for_frames()

        # neem de foto
        self.frameset = self.pipe.wait_for_frames()
        self.frameset = self.align.process(self.frameset)
        self.profile = self.frameset.get_profile()
        self.depth_intrinsics = self.profile.as_video_stream_profile().get_intrinsics()
        self.w, self.h = self.depth_intrinsics.width, self.depth_intrinsics.height
        self.fx, self.fy = self.depth_intrinsics.fx, self.depth_intrinsics.fy
        self.px, self.py = self.depth_intrinsics.ppx, self.depth_intrinsics.ppy

        self.color_frame = self.frameset.get_color_frame()
        self.depth_frame = self.frameset.get_depth_frame()

        # self.depth_frame = self.threshold.process(self.depth_frame)
        # self.depth_frame = self.depth_to_disparity.process(self.depth_frame)
        # self.depth_frame = self.dec_filter.process(self.depth_frame)
        # self.depth_frame = self.temp_filter.process(self.depth_frame)
        # self.depth_frame = self.spat_filter.process(self.depth_frame)
        # self.depth_frame = self.disparity_to_depth.process(self.depth_frame)
        # self.depth_frame = self.hole_filter.process(self.depth_frame)
        # self.depth_frame = self.depth_frame.as_depth_frame()

        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(self.w, self.h, self.fx, self.fy, self.px, self.py)
        color = cv2.imread("D:\\thermal\\temp5.png")
        image_rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
        self.color_image = np.asanyarray(image_rgb)

    def processFoto(self, angle):
        print(angle)
        self.angle = angle
        self.depth_frame_open3d = o3d.geometry.Image(self.depth_image)
        self.color_frame_open3d = o3d.geometry.Image(self.color_image)

        self.rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(self.color_frame_open3d,
                                                                             self.depth_frame_open3d,
                                                                             convert_rgb_to_intensity=False)
        self.pcd = o3d.geometry.PointCloud.create_from_rgbd_image(self.rgbd_image, self.intrinsic)
        self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        self.pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
        self.getcameraLocation()
        self.rMatrix()
        self.pcd.rotate(self.R, (0, 0, 0))
        self.pcd.translate((self.x, self.y, self.z))
        self.pcd = self.pcd.crop(self.bbox)
        self.pcd, self.ind = self.pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=2)
        self.main_pcd = self.main_pcd + self.pcd

    def getPointcloud(self):
        return self.main_pcd

    def giveImageArray(self):
        return self.color_image

    def getcameraLocation(self):
        self.x = np.sin(self.angle * self.dtr) * self.distance - np.cos(self.angle * self.dtr) * 0.035
        self.y = -np.cos(self.angle * self.dtr) * self.distance - np.sin(self.angle * self.dtr) * 0.035
        self.z = 0.165
        self.o = self.angle
        self.a = 112.5
        self.t = 0


    def rMatrix(self):
        self.o = self.o * self.dtr
        self.a = (-self.a) * self.dtr
        self.t = self.t * self.dtr
        self.R = [[np.cos(self.o) * np.cos(self.t) - np.cos(self.a) * np.sin(self.o) * np.sin(self.t),
                   -np.cos(self.o) * np.sin(self.t) - np.cos(self.a) * np.cos(self.t) * np.sin(self.o),
                   np.sin(self.o) * np.sin(self.a)],
                  [np.cos(self.t) * np.sin(self.o) + np.cos(self.o) * np.cos(self.a) * np.sin(self.t),
                   np.cos(self.o) * np.cos(self.a) * np.cos(self.t) - np.sin(self.o) * np.sin(self.t),
                   -np.cos(self.o) * np.sin(self.a)],
                  [np.sin(self.a) * np.sin(self.t), np.cos(self.t) * np.sin(self.a), np.cos(self.a)]]

    def makeSTL(self, kpoints, stdRatio, depth, iterations):
        # print(self.main_pcd)
        self.stl_pcd = self.main_pcd
        self.stl_pcd = self.stl_pcd.uniform_down_sample(every_k_points=kpoints)
        self.stl_pcd, self.ind = self.stl_pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=stdRatio)
        self.bbox1 = o3d.geometry.AxisAlignedBoundingBox((-0.13, -0.13, 0), (0.13, 0.13, 0.01))
        self.bottom = self.stl_pcd.crop(self.bbox1)
        try:
            self.hull, self._ = self.bottom.compute_convex_hull()
            self.bottom = self.hull.sample_points_uniformly(number_of_points=10000)
            self.bottom.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            self.bottom.orient_normals_towards_camera_location(camera_location=np.array([0., 0., -10.]))
            self.bottom.paint_uniform_color([0, 0, 0])
            self._, self.pt_map = self.bottom.hidden_point_removal([0, 0, -1], 1)
            self.bottom = self.bottom.select_by_index(self.pt_map)
            self.stl_pcd = self.stl_pcd + self.bottom
        except:
            print("No bottom could be made")
            pass
        finally:
            self.mesh, self.densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(self.stl_pcd,
                                                                                                  depth=depth)
            self.mesh = self.mesh.filter_smooth_simple(number_of_iterations=iterations)
            self.mesh.scale(1000, center=(0, 0, 0))
            self.mesh.compute_vertex_normals()

        return self.mesh


class App(tk.Tk):

    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        self.title_font = tkfont.Font(family='Arial', size=18, weight="bold", slant="italic")
        self.title("3D Thermal Scanner")
        # self.wm_iconbitmap('icoontje3dscan_WDJ_icon.ico')
        self.iconbitmap(default="C:\\Users\\SMPM\\Downloads\\3D scanner\\Main program\\icoontje3dscan_WDJ_icon.ico")
        self.resizable(False, False)
        self.dictionary = self.readSettings()

        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (StartPage, SettingsPage):
            page_name = F.__name__
            frame = F(parent=container, controller=self)
            self.frames[page_name] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame("StartPage")

        self.frames["StartPage"].stlButton.configure(bg="#cccccc")
        self.frames["StartPage"].saveButton.configure(bg="#cccccc")
        self.frames["StartPage"].buttonShowPC.configure(bg="#cccccc")
        self.enablePC = False
        self.enableSaveSTL = False

    def show_frame(self, page_name):
        '''Show a frame for the given page name'''
        frame = self.frames[page_name]
        frame.tkraise()

    def startScan(self):
        # Initialize Arduino and Scan objects
        self.ard = Arduino(str(self.dictionary["COMport"]), int(self.dictionary["baudrate"]), 0.1)
        self.scan = Scan(int(self.dictionary["widthFrame"]), int(self.dictionary["heightFrame"]), 30, 10, 0)
        self.scan.startPipeline()
        self.frames["StartPage"].startProgress()

        try:
            # Define a function to run the external program
            def run_external():
                try:
                    self.run_external_program()
                except Exception as e:
                    print(f"Error running external program: {e}")

            # Start a thread to run the external program
            external_thread = threading.Thread(target=run_external)
            external_thread.start()

            while True:
                self.scan.takeFoto()
                self.frames["StartPage"].showImage(self.scan.giveImageArray())
                angle = float(self.ard.giveAngle())
                self.frames["StartPage"].Progress(angle)
                self.ard.rotate(int(self.dictionary["stepSize"]))
                self.scan.processFoto(angle)
                self.ard.waitForRotation()
                self.update()

                if angle >= 360:
                    print("The circle is complete!")
                    self.frames["StartPage"].endProgress()
                    break

                if keyboard.is_pressed('q'):
                    print("Scan stopped by user.")
                    break
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            self.scan.stopPipeline()
            self.ard.close()
            self.enablePC = True
            self.frames["StartPage"].stlButton.configure(bg="#f2f2f2")
            self.frames["StartPage"].buttonShowPC.configure(bg="#f2f2f2")
            # Wait for the x function thread to complete before exiting the startScan method


    def showPC(self):
        o3d.visualization.draw_geometries([self.scan.getPointcloud()])


    def makeSTL(self):
        if self.enablePC == True:
            self.STL = self.scan.makeSTL(int(self.dictionary["k_points"]), float(self.dictionary["std_ratio"]),
                                         int(self.dictionary["depth"]), int(self.dictionary["iterations"]))
            # o3d.visualization.draw_geometries([self.STL])
            self.enableSaveSTL = True
            self.frames["StartPage"].saveButton.configure(bg="#f2f2f2")
            print("makestl")

    def saveSTL(self):
        if self.enableSaveSTL == True:
            self.directory = tk.filedialog.asksaveasfilename(initialfile="MySTL.stl")
            o3d.io.write_triangle_mesh(self.directory, self.STL)
            def play():
                self.play_video()
            play_thread = threading.Thread(target=play)
            play_thread.start()

            reader = vtk.vtkSTLReader()
            reader.SetFileName(self.directory)  # Replace "example.stl" with your STL file path

            # Create a mapper
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(reader.GetOutputPort())

            # Create an actor
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)

            # Create a renderer
            renderer = vtk.vtkRenderer()
            renderer.AddActor(actor)
            renderer.SetBackground(0.2, 0.3, 0.4)  # Set background color (optional)

            # Create a render window
            render_window = vtk.vtkRenderWindow()
            render_window.SetSize(800, 600)  # Set window size

            # Create an interactor
            interactor = vtk.vtkRenderWindowInteractor()
            interactor.SetRenderWindow(render_window)

            # Add renderer to render window
            render_window.AddRenderer(renderer)

            # Initialize the interactor and start the rendering loop
            interactor.Initialize()
            render_window.Render()
            interactor.Start()

    def readSettings(self):
        file = open("C:\\Users\\SMPM\\Downloads\\3D scanner\\Main program\\settings.txt", "r")
        contents = file.read()
        dictionary = ast.literal_eval(contents)
        file.close()
        return dictionary

    def writeSettings(self, newdictionary):
        self.newdictionary = newdictionary
        with open("C:\\Users\\SMPM\\Downloads\\3D scanner\\Main program\\settings.txt", 'w') as self.data:
            self.data.write(str(self.newdictionary))

    def saveSettings(self):
        self.frames["SettingsPage"].enterSettings()
        self.writeSettings(self.dictionary)

    def close_windows(self):
        self.destroy()

    def play_video(self):
        # Open the video file
        cap = cv2.VideoCapture("D:\\output_video4.avi")

        # Check if the video file was opened successfully
        if not cap.isOpened():
            print("Error: Unable to open video file.")
            return

        # Read the frame rate of the video
        fps = cap.get(cv2.CAP_PROP_FPS)
        cv2.namedWindow('Thermal Video', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Thermal Video', 800,600)

        # Read and display frames from the video
        while True:
            # Read a frame from the video
            ret, frame = cap.read()

            # If the frame was not read successfully, end of the video
            if not ret:
                break

            # Display the frame
            cv2.imshow('Thermal Video', frame)

            # Wait for the duration of one frame before displaying the next frame
            # Use int(1000 / fps) to calculate the delay in milliseconds
            if cv2.waitKey(int(1000 / fps)) & 0xFF == ord('q'):
                break

        # Release the video capture object and close the OpenCV window
        cap.release()
        cv2.destroyAllWindows()


    def run_external_program(self):
        # Replace "python_script.py" with the path to the Python script you want to run
        script_path = "C:\\Users\\SMPM\\PycharmProjects\\solid_capture\\edited captureimage.py"

        try:
            # Run the external Python program
            subprocess.run(["python", script_path], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")
        except FileNotFoundError:
            print(f"Error: File '{script_path}' not found.")
        else:
            print("External program executed successfully.")


    def run_temp_function(self):
        script_path = "C:\\Users\\SMPM\\PycharmProjects\\solid_capture\\getting temperature of individual pixel.py"

        try:

            subprocess.run(["python", script_path], check=True)
        except subprocess.CalledProcessError as e:
            print(f"error:{e}")
        except FileNotFoundError:
            print(f"Error: File '{script_path}' is not found.")
        else:
            print("External program executed successfully")


class StartPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        self.controller = controller

        self.buttonFrame = tk.Frame(self)  # ,highlightbackground="black",highlightthickness=1
        self.buttonStart = tk.Button(self.buttonFrame, text='Start Scan', width=25, command=self.controller.startScan)
        self.buttonStart.grid(sticky="W", row=0, column=0, pady=5, padx=10)

        self.buttonSettings = tk.Button(self.buttonFrame, text='Settings', width=25,
                                        command=lambda: controller.show_frame("SettingsPage"))
        self.buttonSettings.grid(sticky="W", row=1, column=0, pady=5, padx=10)

        self.buttonShowPC = tk.Button(self.buttonFrame, text='Show Pointcloud', width=25,
                                      command=self.controller.showPC)
        self.buttonShowPC.grid(sticky="W", row=2, column=0, pady=5, padx=10)

        self.stlButton = tk.Button(self.buttonFrame, text='Make STL', width=25, command=self.controller.makeSTL)
        self.stlButton.grid(sticky="W", row=3, column=0, pady=5, padx=10)

        self.saveButton = tk.Button(self.buttonFrame, text='Save STL', width=25, command=self.controller.saveSTL)
        self.saveButton.grid(sticky="W", row=4, column=0, pady=5, padx=10)

        self.ThermalButton = tk.Button(self.buttonFrame, text="Thermal Data", width=25, command=self.controller.run_temp_function)
        self.ThermalButton.grid(sticky="W", row=5, column=0, pady=5, padx=10)

        self.quitButton = tk.Button(self.buttonFrame, text='Quit', width=25, command=self.controller.close_windows)
        self.quitButton.grid(sticky="W", row=6, column=0, pady=5, padx=10)

        self.buttonFrame.grid(sticky="W", row=0, column=0)

        self.load = Image.fromarray(np.zeros(shape=(480, 848, 3)), 'RGB')
        self.left, self.upper, self.right, self.lower = 0, 0, 848, 480

        # self.load = Image.open("testfoto.png")
        self.render = ImageTk.PhotoImage(self.load.crop((self.left, self.upper, self.right, self.lower)))
        self.canvas = tk.Canvas(self, width=self.right - self.left, height=self.lower - self.upper)
        self.canvas.grid(sticky="W", row=0, column=1)
        self.canvas.create_image(0, 0, anchor='nw', image=self.render)
        self.canvas.image = self.render

    def showImage(self, iArray):
        self.load = Image.fromarray(iArray, 'RGB')
        self.render = ImageTk.PhotoImage(self.load.crop((self.left, self.upper, self.right, self.lower)))
        self.canvas.create_image(0, 0, anchor='nw', image=self.render)
        self.canvas.image = self.render

    def startProgress(self):
        self.progress = Progressbar(self, orient="horizontal", length=self.right - self.left, mode='determinate',
                                    maximum=360)
        self.progress.grid(sticky="W", row=1, column=1, pady=5)

    def Progress(self, getal):
        self.progress['value'] = getal

    def endProgress(self):
        self.progress.grid_forget()


class SettingsEntry():

    def __init__(self, master, name, **kwargs):
        self.name = name
        self.master = master
        self.var = kwargs.get('var', None)
        self.frame = tk.Frame(self.master)
        self.label = tk.Label(self.frame, text=self.name, width=15, anchor='e')
        self.label.pack(side="left", fill="both")
        self.entry = tk.Entry(self.frame)
        self.entry.pack(side="left", fill="both")
        self.entry.delete(0)
        self.entry.insert(0, self.var)
        self.frame.pack()

    def get(self):
        return self.entry.get()

    def insert(self, newVar):
        self.entry.delete(0, 'end')
        self.entry.insert(0, newVar)


class SettingsPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        self.controller = controller

        self.buttonFrame = tk.Frame(self)
        self.buttonMp = tk.Button(self.buttonFrame, text='Main Page', width=25,
                                  command=lambda: controller.show_frame("StartPage"))
        self.buttonMp.grid(sticky="W", row=0, column=0, pady=5, padx=10)

        # self.buttonEntersettings = tk.Button(self.buttonFrame, text="Use Settings",width = 25,command=self.enterSettings)
        # self.buttonEntersettings.grid(sticky="W",row = 1, column = 0, pady = 5, padx = 10)

        self.buttonDefaultsettings = tk.Button(self.buttonFrame, text="Reset Default Settings", width=25,
                                               command=self.defaultSettings)
        self.buttonDefaultsettings.grid(sticky="W", row=1, column=0, pady=5, padx=10)

        self.buttonSavesettings = tk.Button(self.buttonFrame, text="Save and Use Settings", width=25,
                                            command=self.controller.saveSettings)
        self.buttonSavesettings.grid(sticky="W", row=2, column=0, pady=5, padx=10)

        self.quitButton = tk.Button(self.buttonFrame, text='Quit', width=25, command=self.controller.close_windows)
        self.quitButton.grid(sticky="W", row=3, column=0, pady=5, padx=10)

        self.buttonFrame.pack(side="left")

        self.BasicSettingsFrame = tk.LabelFrame(self, text="Basic Settings")

        self.e1 = SettingsEntry(self.BasicSettingsFrame, "Step size: ", var=self.controller.dictionary["stepSize"])
        self.e2 = SettingsEntry(self.BasicSettingsFrame, "Frame width: ", var=self.controller.dictionary["widthFrame"])
        self.e3 = SettingsEntry(self.BasicSettingsFrame, "Frame height: ",
                                var=self.controller.dictionary["heightFrame"])
        self.e4 = SettingsEntry(self.BasicSettingsFrame, "COM port Arduino: ",
                                var=self.controller.dictionary["COMport"])
        self.e5 = SettingsEntry(self.BasicSettingsFrame, "Baudrate Arduino: ",
                                var=self.controller.dictionary["baudrate"])

        self.BasicSettingsFrame.pack(side="left", fill="both")  # fill="both", expand="yes"

        self.MeshSettingsFrame = tk.LabelFrame(self, text="Mesh Settings")

        self.e6 = SettingsEntry(self.MeshSettingsFrame, "k points: ", var=self.controller.dictionary["k_points"])
        self.e7 = SettingsEntry(self.MeshSettingsFrame, "std ratio: ", var=self.controller.dictionary["std_ratio"])
        self.e8 = SettingsEntry(self.MeshSettingsFrame, "depth: ", var=self.controller.dictionary["depth"])
        self.e9 = SettingsEntry(self.MeshSettingsFrame, "iterations: ", var=self.controller.dictionary["iterations"])

        self.MeshSettingsFrame.pack(side="left", fill="both")

    def enterSettings(self):
        self.controller.dictionary["stepSize"] = self.e1.get()
        self.controller.dictionary["widthFrame"] = self.e2.get()
        self.controller.dictionary["heightFrame"] = self.e3.get()
        self.controller.dictionary["COMport"] = self.e4.get()
        self.controller.dictionary["baudrate"] = self.e5.get()
        self.controller.dictionary["k_points"] = self.e6.get()
        self.controller.dictionary["std_ratio"] = self.e7.get()
        self.controller.dictionary["depth"] = self.e8.get()
        self.controller.dictionary["iterations"] = self.e9.get()

    def defaultSettings(self):
        # het kan ook direct in de insert, maar zo (met de tussenstap) past ie ook gelijk de dictionary aan
        self.controller.dictionary["stepSize"] = 256
        self.controller.dictionary["widthFrame"] = 848
        self.controller.dictionary["heightFrame"] = 480
        self.controller.dictionary["COMport"] = "COM7"
        self.controller.dictionary["baudrate"] = 9600
        self.controller.dictionary["k_points"] = 10
        self.controller.dictionary["std_ratio"] = 0.5
        self.controller.dictionary["depth"] = 7
        self.controller.dictionary["iterations"] = 8
        self.e1.insert(self.controller.dictionary["stepSize"])
        self.e2.insert(self.controller.dictionary["widthFrame"])
        self.e3.insert(self.controller.dictionary["heightFrame"])
        self.e4.insert(self.controller.dictionary["COMport"])
        self.e5.insert(self.controller.dictionary["baudrate"])
        self.e6.insert(self.controller.dictionary["k_points"])
        self.e7.insert(self.controller.dictionary["std_ratio"])
        self.e8.insert(self.controller.dictionary["depth"])
        self.e9.insert(self.controller.dictionary["iterations"])


if __name__ == "__main__":
    app = App()
    app.mainloop()