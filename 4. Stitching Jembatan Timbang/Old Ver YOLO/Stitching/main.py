from ultralytics import YOLO
import cv2
import pyrealsense2 as rs
import numpy as np

# Load the YOLO model
model = YOLO("yolov8l-seg.pt")
bef_contour = None
saved_frames = []  # To store frames for stitching
stitching_window_name = "Stitched Image"
