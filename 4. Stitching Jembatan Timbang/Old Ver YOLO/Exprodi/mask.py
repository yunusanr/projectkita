import cv2
import numpy as np
from ultralytics import YOLO  # Import YOLO from ultralytics
import time
import torch  # Import PyTorch for CUDA support


class VehicleDimensionApp:
    def __init__(self, video_path, skip_frames=3, resize_factor=0.5):
        self.video_path = video_path
        self.skip_frames = skip_frames  # Number of frames to skip
        self.resize_factor = resize_factor  # Factor to resize frames
        self.pipeline = None
        self.running = False

        # Load YOLO model with CUDA support
        self.model = YOLO("yolov8l-seg.pt")  # Use 'cuda' for GPU

        # Check if CUDA is available
        if torch.cuda.is_available():
            print("CUDA is available. Using GPU.")
        else:
            print("CUDA is not available. Using CPU.")

        # Start processing the video
        self.process_video()

    def process_video(self):
        # Open the video file
        cap = cv2.VideoCapture(self.video_path)

        if not cap.isOpened():
            print("Error: Could not open video.")
            return

        frame_count = 0  # Initialize frame counter

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("End of video.")
                break

            # Skip frames based on the skip_frames parameter
            if frame_count % self.skip_frames != 0:
                frame_count += 1
                continue

            # Resize the frame
            frame = cv2.resize(
                frame, (0, 0), fx=self.resize_factor, fy=self.resize_factor
            )
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
            contour_frame = np.zeros((h, w, 3), dtype=np.uint8)  # Frame for contours
            masked_frame = np.zeros(
                (h, w, 3), dtype=np.uint8
            )  # New frame for masked objects

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
                        "airplane",
                        "person",
                    ]:
                        mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
                        color_map = {
                            "car": [0, 255, 255],
                            "bus": [0, 255, 255],
                            "truck": [0, 255, 255],
                            "suitcase": [0, 255, 255],
                            "refrigerator": [0, 255, 255],
                            "frisbee": [0, 255, 255],
                            "boat": [0, 255, 255],
                            "surfboard": [0, 255, 255],
                            "cell phone": [0, 255, 255],
                            "cake": [0, 255, 255],
                            "cup": [0, 255, 255],
                            "toilet": [0, 255, 255],
                            "bird": [0, 255, 255],
                            "motorcycle": [255, 0, 255],
                            "airplane": [255, 255, 0],
                            "person": [255, 255, 0],
                        }
                        mask[(mask == 255).all(-1)] = color_map[name]
                        frame = cv2.addWeighted(frame, 1, mask, 0.5, 0)

                        # Add the mask to the masked_frame
                        masked_frame = cv2.add(masked_frame, mask)

                        # Find contours in the mask
                        current_contours, _ = cv2.findContours(
                            mask[:, :, 0], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                        )

                        for contour in current_contours:
                            # Draw the contour
                            cv2.drawContours(
                                contour_frame, [contour], -1, (0, 255, 0), 2
                            )  # Green contours

            # Resize and convert the frame for display
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
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


if __name__ == "__main__":
    video_path = "resources\maju2.mp4"  # Replace with your video file path
    app = VehicleDimensionApp(video_path)
