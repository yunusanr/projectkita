import cv2
import numpy as np
from ultralytics import YOLO


class VideoToPanorama:
    def __init__(self, video_path, skip_frames=5):
        self.video_path = video_path
        self.skip_frames = skip_frames
        self.frames = []
        self.model = YOLO("yolov8m-seg.pt")  # Load your YOLOv8 segmentation model here

    def extract_frames(self):
        cap = cv2.VideoCapture(self.video_path)

        if not cap.isOpened():
            print("Error: Could not open video.")
            return

        frame_count = 0
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Skip frames based on the skip_frames parameter
            if frame_count % self.skip_frames == 0:
                # Detect and segment objects in the frame
                self.detect_and_segment_objects(frame)
                self.frames.append(frame)

            frame_count += 1

        cap.release()

    def detect_and_segment_objects(self, frame):
        # Run YOLOv8 segmentation model on the frame
        results = self.model(frame)

        # Draw segmentation masks and bounding boxes for detected objects
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()  # Get bounding box coordinates
            masks = result.masks.data.cpu().numpy()  # Get segmentation masks
            classes = result.boxes.cls.cpu().numpy()  # Get class indices

            for box, mask, cls in zip(boxes, masks, classes):
                x1, y1, x2, y2 = map(int, box[:4])  # Convert to integer
                # Draw bounding box
                cv2.rectangle(
                    frame, (x1, y1), (x2, y2), (0, 255, 0), 2
                )  # Draw rectangle

                # Create a mask for the current object
                mask = (mask * 255).astype(np.uint8)  # Convert mask to uint8
                mask_colored = cv2.cvtColor(
                    mask, cv2.COLOR_GRAY2BGR
                )  # Convert to 3 channels
                color = (0, 255, 255)  # Yellow color for the mask
                frame = cv2.addWeighted(
                    frame, 1, mask_colored, 0.5, 0
                )  # Overlay mask on the frame

    def stitch_images(self):
        # Create a stitcher object
        stitcher = cv2.Stitcher_create()

        # Stitch the frames together
        status, panorama = stitcher.stitch(self.frames)

        if status == cv2.Stitcher_OK:
            print("Panorama created successfully.")
            return panorama
        else:
            print("Error during stitching: ", status)
            return None

    def run(self):
        self.extract_frames()
        panorama = self.stitch_images()

        if panorama is not None:
            # Display the panorama
            cv2.imshow("Panorama", panorama)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            # Save the panorama
            cv2.imwrite("panorama.jpg", panorama)
            print("Panorama saved as 'panorama.jpg'.")
        else:
            print("No panorama to display or save.")


if __name__ == "__main__":
    video_path = r"resources\maju2.mp4"  # Use a raw string for the path
    panorama_creator = VideoToPanorama(video_path)
    panorama_creator.run()
