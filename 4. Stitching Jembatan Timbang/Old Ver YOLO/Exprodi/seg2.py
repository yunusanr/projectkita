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
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2()

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

        # Add batch dimension if necessary
        if frame.ndim == 3:  # Single image case (h, w, c)
            frame = np.expand_dims(frame, axis=0)  # Add batch dimension (1, h, w, c)

        # Run YOLO model
        result = self.model(frame, imgsz=imgsz)

        # Process results
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

        if boxes.size > 0 and clss is not None:
            for box, cls, mask in zip(boxes, clss, masks):
                name = result[0].names[int(cls)]
                if name in ["car", "truck", "bus"]:  # Focus on vehicles
                    mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
                    masked_frame = cv2.add(masked_frame, mask)

                    # Find contours in the mask
                    current_contours, _ = cv2.findContours(
                        mask[:, :, 0], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                    )
                    for contour in current_contours:
                        cv2.drawContours(contour_frame, [contour], -1, (0, 255, 0), 2)

        return frame, masked_frame, contour_frame

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

                # Apply background subtraction
                fg_mask = self.bg_subtractor.apply(frame)
                _, fg_mask = cv2.threshold(fg_mask, 200, 255, cv2.THRESH_BINARY)

                # Submit the frame for processing
                future = executor.submit(self.predict_and_detect, fg_mask)
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


if __name__ == "__main__":
    video_path = "resources/maju6.mp4"  # Replace with your video file path
    app = VehicleDimensionApp(video_path)
