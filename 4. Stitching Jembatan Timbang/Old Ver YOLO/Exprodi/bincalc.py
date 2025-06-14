import cv2
import numpy as np
from ultralytics import YOLO
import concurrent.futures
import torch


class VehicleDimensionApp:
    def __init__(
        self,
        video_path,
        skip_frames=3,
        resize_factor=0.6,
        pixel_to_cm=0.1,
        confidence_threshold=0.5,
    ):
        self.video_path = video_path
        self.skip_frames = skip_frames
        self.resize_factor = resize_factor
        self.model = YOLO("yolov8l-seg.pt")
        self.total_length = 0
        self.total_w = 0
        self.detected_objects = set()
        self.pixel_to_cm = pixel_to_cm
        self.confidence_threshold = confidence_threshold
        self.is_vehicle_tracked = False
        self.previous_mask = None

        self.allowed_classes = [
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
        ]

        if torch.cuda.is_available():
            print("CUDA is available. Using GPU.")
        else:
            print("CUDA is not available. Using CPU.")

        self.process_video()

    def predict_and_detect(self, frame):
        frame = cv2.resize(frame, (0, 0), fx=self.resize_factor, fy=self.resize_factor)
        h, w = frame.shape[:2]
        imgsz = max(w, h)

        result = self.model(frame, imgsz=imgsz)
        masks = (
            (result[0].masks.data.cpu().numpy() * 255).astype("uint8")
            if result[0].masks is not None
            else None
        )
        clss = result[0].boxes.cls
        confs = result[0].boxes.conf.cpu().numpy()

        detected_this_frame = set()
        length = 0

        if masks is not None and clss is not None:
            largest_contour = None
            largest_area = 0

            for mask, cls, conf in zip(masks, clss, confs):
                if conf < self.confidence_threshold:
                    continue

                name = result[0].names[int(cls)]
                if name not in self.allowed_classes:
                    continue

                detected_this_frame.add(name)

                mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                contours, _ = cv2.findContours(
                    mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > largest_area:
                        largest_area = area
                        largest_contour = contour

                # Draw the mask on the frame
                frame[mask_resized > 0] = [255, 255, 255]  # Color the mask in blue

            if largest_contour is not None:
                x, y, w, h = cv2.boundingRect(largest_contour)
                length = w * self.pixel_to_cm
                self.total_w += w

                if not self.is_vehicle_tracked or self.has_significant_movement(
                    largest_contour
                ):
                    self.total_length += length
                    self.detected_objects.add(name)
                    self.is_vehicle_tracked = True
                    self.previous_mask = (
                        largest_contour  # Store the current mask for tracking
                    )

                cv2.drawContours(
                    frame, [largest_contour], -1, (0, 255, 0), 2
                )  # Draw the largest contour

        return frame, self.total_length

    def has_significant_movement(self, current_contour):
        if self.previous_mask is None:
            return True
        # Calculate the movement distance based on contour centroids
        M1 = cv2.moments(self.previous_mask)
        M2 = cv2.moments(current_contour)
        if M1["m00"] == 0 or M2["m00"] == 0:
            return True
        cX1 = int(M1["m10"] / M1["m00"])
        cY1 = int(M1["m01"] / M1["m00"])
        cX2 = int(M2["m10"] / M2["m00"])
        cY2 = int(M2["m01"] / M2["m00"])
        movement_threshold = 10  # Define a threshold for significant movement
        movement = np.linalg.norm(np.array([cX1, cY1]) - np.array([cX2, cY2]))
        return movement > movement_threshold

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

                if frame_count % self.skip_frames != 0:
                    frame_count += 1
                    continue

                future = executor.submit(self.predict_and_detect, frame)
                result_frame, total_length = future.result()

                cv2.putText(
                    result_frame,
                    f"Length: {total_length:.2f} cm",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (255, 0, 0),
                    2,
                )

                cv2.imshow("Video Input", result_frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                frame_count += 1

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    video_path = "resources/maju2.mp4"  # Replace with your video file path
    app = VehicleDimensionApp(video_path, pixel_to_cm=0.11, confidence_threshold=0.3)
