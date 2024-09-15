import serial
import time
import cv2
import os

def send_command(command):
    ser.write(command.encode('utf-8') + b'\r\n')
    time.sleep(1)  # Adjust as needed

def read_response():
    response = ser.readline().decode('utf-8')
    return response.strip()

ser = serial.Serial('COM6', baudrate=9600, timeout=1)

try:
    send_command('import pyb')
    send_command('import sensor')
    send_command('sensor.reset()')
    time.sleep(2)  # Allow time for the sensor to reset
    print(read_response())  # Print response after sensor reset
    send_command('sensor.set_pixformat(sensor.RGB565)')
    send_command('sensor.set_framesize(sensor.QVGA)')
    # send_command('sensor.set_auto_gain(False)')
    send_command('sensor.skip_frames(time=2000)')

    for i in range(1):  # Capture four images
        send_command('img = sensor.snapshot()')
        print(read_response())  # Print response after image capture
        send_command('img.save("temp/temp{}.jpg")'.format(i))  # Save with different filenames
        print(read_response())  # Print response after saving the image
        print("Image {} captured and saved to temp{}.jpg".format(i+1, i))


finally:
    send_command('pyb.hard_reset()')
    ser.close()


time.sleep(4)

image = cv2.imread("E:\\temp\\temp0.jpg")
image_resize = cv2.resize(image, (848, 480))
# rimage = cv2.rotate(image_resize, cv2.ROTATE_90_COUNTERCLOCKWISE)
cv2.imwrite("D:\\thermal\\temp5.png", image_resize)
cv2.imshow("image_resize", image_resize)
cv2.waitKey(0)
# import numpy as np
# roi_i = cv2.imread("D:\\thermal\\temp5.png")
# roi_g = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# # roi_r = 255 - roi_g
# # cv2.imshow("Gray scale image", roi_g)
# roi = roi_g[150:180, 145:210]
# # roi = roi_r[280:370, 390:560]
# Min_temp = 30
# max_temp = 70
# min_pix = 200
# max_pix = 255
# average_pixel_value = np.mean(roi, axis=(0, 1))
# print("Red value:", average_pixel_value)
# clipped_pixel_value = np.clip(average_pixel_value, min_pix, max_pix)
# print("cliped pixel value:", clipped_pixel_value)
# t = (((clipped_pixel_value-200)*(70-30)/(max_pix - min_pix))+30)
# print(f"the temperature range will be {t * 1.05} - {t * 0.9}")
# print("temperature:", t)
# print("average pixel value:", average_pixel_value)
# roi_rec = cv2.rectangle(roi_g, (145, 150), (210, 180),(255, 255, 255), 1)
# # roi_rec = cv2.rectangle(roi_r, (390, 280), (560, 370),(255, 255, 255), 2)
# cv2.imshow("roi", roi_rec)
# cv2.waitKey(0)


# time.sleep(4)
#
# # Directory containing the images
# image_folder = 'E:\\temp'
# # Output video file name
# video_name = 'D:\\output_video4.avi'
#
# # Get the list of image filenames in the directory
# images = [img for img in os.listdir(image_folder) if img.endswith(".jpg")]
#
# # Sort the image filenames to ensure they are in the correct order
# images.sort()
#
# # Get the dimensions of the first image to use for video size
# frame = cv2.imread(os.path.join(image_folder, images[0]))
# height, width, layers = frame.shape
#
# # Define the codec and create VideoWriter object
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# video = cv2.VideoWriter(video_name, fourcc, 1, (width, height))
#
# # Iterate through the images and add them to the video
# for image in images:
#     video.write(cv2.imread(os.path.join(image_folder, image)))
#
# # Release the video writer object
# video.release()
#
# print(f"Video '{video_name}' created successfully.")

# time.sleep(2)
#
# def overlay_videos(background_video_path, overlay_video_path, output_video_path):
#     # Open background video
#     background_video = cv2.VideoCapture(background_video_path)
#
#     # Open overlay video
#     overlay_video = cv2.VideoCapture(overlay_video_path)
#
#     # Check if videos opened successfully
#     if not background_video.isOpened() or not overlay_video.isOpened():
#         print("Error: One or both input videos could not be opened.")
#         return
#
#     # Get properties of background video
#     width = int(background_video.get(cv2.CAP_PROP_FRAME_WIDTH))
#     height = int(background_video.get(cv2.CAP_PROP_FRAME_HEIGHT))
#     fps = background_video.get(cv2.CAP_PROP_FPS)
#
#     # Create VideoWriter object to write the output video
#     fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use appropriate codec based on file extension
#     out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))
#
#     # Check if VideoWriter initialized successfully
#     if not out.isOpened():
#         print("Error: Output video could not be initialized.")
#         return
#
#     # Read frames from both videos and overlay them
#     while True:
#         ret_bg, frame_bg = background_video.read()
#         ret_overlay, frame_overlay = overlay_video.read()
#
#         if not ret_bg or not ret_overlay:
#             break
#
#         # Resize overlay frame to match background frame dimensions
#         frame_overlay = cv2.resize(frame_overlay, (width, height))
#
#         # Overlay the frames
#         combined_frame = cv2.addWeighted(frame_bg, 1, frame_overlay, 0.5, 0)
#
#         # Write the combined frame to the output video
#         out.write(combined_frame)
#
#     # Release resources
#     background_video.release()
#     overlay_video.release()
#     out.release()
#     cv2.destroyAllWindows()
#

# # Example usage
# background_video_path = 'C:\\Users\\SMPM\\Desktop\\super depth.mp4.mp4'
# overlay_video_path = 'C:\\Users\\SMPM\\Desktop\\thermal video.mp4'
# output_video_path = 'C:\\Users\\SMPM\\Pictures\\Saved Pictures\\new2.mp4'
#
# overlay_videos(background_video_path, overlay_video_path, output_video_path)
