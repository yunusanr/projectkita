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

        # Define the horizontal line position
        center_y = h // 2  # Center line
        line_thickness = 1

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
                    "parking meter",
                    "toaster",
                    "toilet",
                    "cake",
                    "cup",
                    "bird",
                    "motorcycle",
                    "person",
                ]:  # Add more classes as needed
                    mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
                    color_map = {
                        "car": [0, 255, 255],
                        "bus": [0, 255, 255],
                        "truck": [0, 255, 255],
                        "suitcase": [0, 255, 255],
                        "refrigerator": [0, 255, 255],
                        "frisbee": [0, 255, 255],
                        "parking meter": [0, 255, 255],
                        "toaster": [0, 255, 255],
                        "boat": [0, 255, 255],
                        "surfboard": [0, 255, 255],
                        "cell phone": [0, 255, 255],
                        "cake": [0, 255, 255],
                        "cup": [0, 255, 255],
                        "toilet": [0, 255, 255],
                        "bird": [0, 255, 255],
                        "motorcycle": [0, 255, 255],
                        "person": [255, 255, 0],
                    }
                    mask[(mask == 255).all(-1)] = color_map[name]
                    frame = cv2.addWeighted(frame, 1, mask, 0.5, 0)
                    masked_frame = cv2.add(masked_frame, mask)

                    # Find contours in the mask
                    current_contours, _ = cv2.findContours(
                        mask[:, :, 0], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                    )
                    for contour in current_contours:
                        cv2.drawContours(contour_frame, [contour], -1, (0, 255, 0), 2)

                        # Get bounding box for the contour
                        x, y, w, h = cv2.boundingRect(contour)
                        # Draw the bounding box on the masked frame
                        cv2.rectangle(
                            masked_frame, (x, y), (x + w, y + h), (255, 0, 0), 2
                        )  # Blue box

                        # Check if the mask intersects with the horizontal line
                        if self.check_intersection_with_line(mask, center_y):
                            print(
                                f"Object intersects with the line at position: {np.argwhere(mask[:, :, 0] == 255)}"
                            )

        # Convert masked_frame to binary
        masked_frame_gray = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2GRAY)
        _, binary_mask = cv2.threshold(masked_frame_gray, 1, 255, cv2.THRESH_BINARY)

        # Draw a horizontal line at the center of the binary mask
        cv2.line(
            binary_mask,
            (0, center_y),
            (binary_mask.shape[1], center_y),
            (255),
            line_thickness,
        )

        return frame, masked_frame, contour_frame, binary_mask

    def check_intersection_with_line(self, mask, line_y):
        # Check if any part of the mask intersects with the horizontal line
        # Get the binary mask for the current object
        binary_mask = mask[:, :, 0]  # Use the first channel for binary mask

        # Check if there are any white pixels (255) in the row corresponding to the line_y
        if np.any(binary_mask[line_y, :] == 255):
            return True
        return False

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
                result_frame, masked_frame, contour_frame, binary_mask = future.result()

                # Convert frames for display
                frame_rgb = cv2.cvtColor(result_frame, cv2.COLOR_BGR2RGB)
                masked_frame_rgb = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2RGB)
                contour_frame_rgb = cv2.cvtColor(contour_frame, cv2.COLOR_BGR2RGB)
                binary_mask_rgb = cv2.cvtColor(
                    binary_mask, cv2.COLOR_GRAY2RGB
                )  # Convert binary mask to RGB for display

                # Show the frames
                cv2.imshow("Video Input", frame_rgb)
                cv2.imshow("Masked Objects", masked_frame_rgb)
                cv2.imshow("Contours", contour_frame_rgb)
                cv2.imshow("Binary Mask", binary_mask_rgb)  # Display the binary mask

                # Exit on 'q' key
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                frame_count += 1  # Increment frame counter

        cap.release()
        cv2.destroyAllWindows()  # Close all OpenCV windows


if __name__ == "__main__":
    video_path = "resources/maju6.mp4"  # Replace with your video file path
    app = VehicleDimensionApp(video_path)
