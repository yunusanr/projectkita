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
        resize_factor=0.5,
        pixel_to_cm=0.1,
        confidence_threshold=0.5,
    ):
        self.video_path = video_path
        self.skip_frames = skip_frames
        self.resize_factor = resize_factor
        self.model = YOLO("yolov8l-seg.pt")
        self.total_length = 0  # To accumulate lengths over frames
        self.total_w = 0  # To accumulate lengths over frames
        self.detected_objects = set()  # To track detected objects
        self.pixel_to_cm = pixel_to_cm  # Conversion factor from pixels to cm
        self.confidence_threshold = confidence_threshold  # Confidence threshold

        # Define the allowed classes
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
            "airplane",
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
        boxes = result[0].boxes.xyxy.cpu().numpy().astype(int)
        masks = (
            (result[0].masks.data.cpu().numpy() * 255).astype("uint8")
            if result[0].masks is not None
            else None
        )
        clss = result[0].boxes.cls
        confs = result[0].boxes.conf.cpu().numpy()  # Get confidence scores

        detected_this_frame = set()  # Track detected objects in this frame
        length = 0  # Initialize length variable

        if boxes.size > 0 and clss is not None:
            largest_contour = None
            largest_area = 0

            for box, cls, mask, conf in zip(boxes, clss, masks, confs):
                if conf < self.confidence_threshold:  # Check confidence threshold
                    continue  # Skip this detection

                name = result[0].names[int(cls)]
                if name not in self.allowed_classes:  # Check if the class is allowed
                    continue  # Skip this detection

                detected_this_frame.add(name)  # Mark this object as detected

                # Resize the mask to match the frame size
                mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))

                # Find contours in the mask
                contours, _ = cv2.findContours(
                    mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > largest_area:  # Check for the largest contour
                        largest_area = area
                        largest_contour = contour

                # Draw the bounding box
                cv2.rectangle(
                    frame,
                    (int(box[0]), int(box[1])),
                    (int(box[2]), int(box[3])),
                    (0, 255, 0),
                    2,
                )

            # If a largest contour was found, calculate its bounding box
            if largest_contour is not None:
                x, y, w, h = cv2.boundingRect(largest_contour)
                length = w * self.pixel_to_cm  # Convert width to cm
                self.total_w += w
                print(self.total_w)

                # Check if this object has been detected before
                if name not in self.detected_objects:
                    self.total_length += length  # Accumulate length
                    self.detected_objects.add(name)  # Mark this object as detected

                # Draw the largest contour
                cv2.drawContours(
                    frame, [largest_contour], -1, (255, 0, 0), 2
                )  # Blue contour

        return frame, self.total_length

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

                # Display the total length on the frame
                cv2.putText(
                    result_frame,
                    f"Length: {total_length:.2f} px",
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
    app = VehicleDimensionApp(
        video_path, pixel_to_cm=0.1, confidence_threshold=0.2
    )  # Set your desired threshold
