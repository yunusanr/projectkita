import cv2
import numpy as np
from ultralytics import YOLO
import concurrent.futures
import torch


class VehicleDimensionApp:
    def __init__(self, video_path, skip_frames=2, resize_factor=0.5):
        self.video_path = video_path
        self.skip_frames = skip_frames
        self.resize_factor = resize_factor
        self.model = YOLO("yolov8l-seg.pt")
        self.scanned_objects = []  # List to store scanned objects
        self.last_detected_masks = []  # Store last detected masks

        # Check if CUDA is available
        if torch.cuda.is_available():
            print("CUDA is available. Using GPU.")
        else:
            print("CUDA is not available. Using CPU.")

        # Start processing the video
        self.process_video()

    def predict_and_detect(self, frame):
        # Resize the frame
        frame = cv2.resize(frame, (0, 0), fx=self.resize_factor, fy=self.resize_factor)
        h, w = frame.shape[:2]
        imgsz = max(w, h)

        # Run YOLO model
        result = self.model(frame, imgsz=imgsz)
        boxes = result[0].boxes.xyxy.cpu().numpy().astype(int)
        masks = (
            (result[0].masks.data.cpu().numpy() * 255).astype("uint8")
            if result[0].masks is not None
            else None
        )
        clss = result[0].boxes.cls

        # Create a black background for contours
        contour_frame = np.zeros((h, w, 3), dtype=np.uint8)
        masked_frame = np.zeros((h, w, 3), dtype=np.uint8)

        current_masks = []  # Store current masks for this frame

        if boxes.size > 0 and clss is not None:
            for box, cls, mask in zip(boxes, clss, masks):
                name = result[0].names[int(cls)]
                if name in [
                    "car",
                    "truck",
                    "bus",
                    "suitcase",
                    "refrigerator",
                    "frisbee",
                    "boat",
                    "surfboard",
                    "cell phone",
                    "toilet",
                    "cake",
                    "cup",
                    "bird",
                    "motorcycle",
                    "person",
                ]:  # Add more classes as needed
                    mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
                    color_map = {name: [0, 255, 255]}  # Color for detected objects
                    mask[(mask == 255).all(-1)] = color_map[name]
                    masked_frame = cv2.add(masked_frame, mask)

                    # Store the current mask
                    current_masks.append(mask)

                    # Find contours in the mask
                    current_contours, _ = cv2.findContours(
                        mask[:, :, 0], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                    )
                    for contour in current_contours:
                        cv2.drawContours(contour_frame, [contour], -1, (0, 255, 0), 2)

                        # Check if the contour crosses the horizontal line
                        if self.check_crossing_line(contour, h):
                            self.scanned_objects.append(mask)  # Save the masked object

        # If no current masks were detected, keep the last known masks
        if not current_masks and self.last_detected_masks:
            for last_mask in self.last_detected_masks:
                masked_frame = cv2.add(masked_frame, last_mask)

        # Draw the center line on the masked frame
        center_line_y = masked_frame.shape[0] // 2
        cv2.line(
            masked_frame,
            (0, center_line_y),
            (masked_frame.shape[1], center_line_y),
            (255, 0, 0),
            1,
        )

        # Update the last detected masks
        self.last_detected_masks = current_masks

        return frame, masked_frame, contour_frame

    def check_crossing_line(self, contour, frame_height):
        # Define the center line position
        center_line_y = frame_height // 2

        # Get the bounding box of the contour
        x, y, w, h = cv2.boundingRect(contour)

        # Check if the contour crosses the center line
        return y < center_line_y < (y + h)

    def process_video(self):
        cap = cv2.VideoCapture(self.video_path)

        if not cap.isOpened():
            print("Error: Could not open video.")
            return

        frame_count = 0
        with concurrent.futures.ThreadPoolExecutor() as executor:
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    print("End of video.")
                    break

                # Skip frames based on the skip_frames parameter
                if frame_count % self.skip_frames != 0:
                    frame_count += 1
                    continue

                # Submit the frame for processing
                future = executor.submit(self.predict_and_detect, frame)
                result_frame, masked_frame, contour_frame = future.result()

                # Convert frames for display
                frame_rgb = cv2.cvtColor(result_frame, cv2.COLOR_BGR2RGB)
                masked_frame_rgb = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2RGB)
                contour_frame_rgb = cv2.cvtColor(contour_frame, cv2.COLOR_BGR2RGB)

                # Show the frames
                cv2.imshow("Video Input", frame_rgb)
                cv2.imshow("Masked Objects", masked_frame_rgb)
                cv2.imshow("Contours", contour_frame_rgb)

                # Exit on 'q' key
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                frame_count += 1  # Increment frame counter

        cap.release()
        cv2.destroyAllWindows()  # Close all OpenCV windows

        # Merge scanned objects into a final output frame
        self.merge_scanned_objects()

    def merge_scanned_objects(self):
        if self.scanned_objects:
            # Calculate total width and maximum height for the merged frame
            total_width = sum(mask.shape[1] for mask in self.scanned_objects)
            max_height = max(mask.shape[0] for mask in self.scanned_objects)

            # Create a blank frame to merge scanned objects
            merged_frame = np.zeros((max_height, total_width, 3), dtype=np.uint8)

            current_x = 0
            for obj in self.scanned_objects:
                obj_height, obj_width = obj.shape[:2]
                # Ensure the object fits within the merged frame
                if current_x + obj_width <= total_width:
                    merged_frame[0:obj_height, current_x : current_x + obj_width] = obj
                    current_x += obj_width  # Move the x position for the next object
                else:
                    print(
                        "Warning: Object exceeds merged frame width. Skipping object."
                    )

            # Show the merged result
            cv2.imshow("Merged Scanned Objects", merged_frame)
            cv2.waitKey(0)  # Wait for a key press to close the window
            cv2.destroyAllWindows()


if __name__ == "__main__":
    video_path = "resources/maju6.mp4"  # Replace with your video file path
    app = VehicleDimensionApp(video_path)
