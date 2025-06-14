import os
import cv2
import numpy as np
import time
from ultralytics import YOLO
from libs.bbox3d_utils import *
from libs.Plotting import *
from model import model_mobilenet, orientation_loss
from utils import *

# Set environment variable to control TensorFlow logging
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"

# Select model
select_model = "mobilenetv2"

# Load the 3D model
bbox3d_model = tf.keras.models.load_model(
    "mobilenetv2/mobilenetv2_weights.h5",
    custom_objects={"orientation_loss": orientation_loss},
)

# Load a 2D model
bbox2d_model = YOLO("best.pt")  # Load an official model
# Set model parameters
bbox2d_model.overrides["conf"] = 0.9  # NMS confidence threshold
bbox2d_model.overrides["iou"] = 0.45  # NMS IoU threshold
bbox2d_model.overrides["agnostic_nms"] = False  # NMS class-agnostic
bbox2d_model.overrides["max_det"] = 1000  # Maximum number of detections per image

# Load the video
video = cv2.VideoCapture("car-stay.mp4")

# Get video information (frame width, height, frames per second)
frame_width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(video.get(cv2.CAP_PROP_FPS))

# Define the codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
out = cv2.VideoWriter(
    select_model + "_output_video.mp4", fourcc, 15, (frame_width, frame_height)
)

frameId = 0
start_time = time.time()
fps_display = ""
BEV_plot = False
TracK = False

# Resize factor
resize_factor = 0.5  # Adjust this value to reduce resolution

# Process each frame of the video
while True:
    frameId += 1
    success, frame = video.read()
    if not success:
        print("Error reading frame")
        break

    # Resize the frame to improve processing speed
    frame_resized = cv2.resize(
        frame, (int(frame_width * resize_factor), int(frame_height * resize_factor))
    )

    img2 = frame_resized.copy()
    img3 = frame_resized.copy()
    plot3dbev = Plot3DBoxBev(P2)

    # Process 2D and 3D boxes
    img2D, bboxes2d = process2D(bbox2d_model, img2, track=TracK)
    if len(bboxes2d) > 0:
        bboxes3D = process3D(bbox3d_model, frame, img2, bboxes2d)
        if len(bboxes3D) > 0:
            for (
                bbox_,
                dim,
                alpha,
                theta_ray,
                orient,
                conf,
                classes,
                location,
                objID,
            ) in bboxes3D:
                plot3d(img3, P2, bbox_, dim, alpha, theta_ray)

                if BEV_plot:
                    # Initialize object container
                    obj = KITTIObject()
                    obj.name = str(yolo_classes[int(classes.cpu().numpy())])
                    obj.truncation = float(0.00)
                    obj.occlusion = int(-1)
                    obj.xmin, obj.ymin, obj.xmax, obj.ymax = (
                        int(bbox_[0]),
                        int(bbox_[1]),
                        int(bbox_[2]),
                        int(bbox_[3]),
                    )

                    obj.alpha = recover_angle(orient, conf, bin_size)
                    obj.h, obj.w, obj.l = dim[0], dim[1], dim[2]
                    obj.rot_global, rot_local = compute_orientaion(P2, obj)
                    obj.tx, obj.ty, obj.tz = translation_constraints(P2, obj, rot_local)

                    # Plot 3D BEV bbox
                    rot_y = alpha + theta_ray
                    plot3dbev.plot(
                        img=img3,
                        class_object=obj.name.lower(),
                        bbox=[obj.xmin, obj.ymin, obj.xmax, obj.ymax],
                        dim=[obj.h, obj.w, obj.l],
                        loc=[obj.tx, obj.ty, obj.tz],
                        rot_y=rot_y,
                        objId=[objID] if not isinstance(objID, list) else objID,
                    )

    if BEV_plot:
        plot3dbev.plot(img=img3)
        img3 = plot3dbev.show_result()

    # Calculate the current time in seconds
    current_time = video.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
    if frameId % 20 == 0:  # Calculate FPS every 20 frames
        end_time = time.time()
        elapsed_time = end_time - start_time
        fps_current = frameId / elapsed_time
        fps_display = f"FPS: {fps_current:.2f}"
        print(f"Frame: {frameId}, FPS: {fps_current:.2f}")

    cv2.putText(
        img3,
        select_model + " " + fps_display,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 255),
        1,
        cv2.LINE_AA,
    )

    # Display the frame
    cv2.imshow("2D", img2)
    cv2.imshow("3D", img3)

    # Write the output video
    out.write(img3)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the video capture and writer
video.release()
out.release()
cv2.destroyAllWindows()
