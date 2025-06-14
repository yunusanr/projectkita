import cv2
import numpy as np
from ultralytics import YOLO  # Import YOLO from ultralytics


class VehicleDimensionApp:
    def __init__(self, video_path):
        # Load YOLO model
        self.model = YOLO("yolov8l-seg.pt")  # Load your YOLOv8 model here

        # Initialize stitched parts tracking
        self.stitched_parts = []  # List to store stitched parts
        self.stitching_threshold = 50  # Threshold for overlapping detection

        # Open the video file
        self.video_path = video_path
        self.cap = cv2.VideoCapture(self.video_path)

    def process_video(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                print("End of video or cannot read the video file.")
                break

            # Get frame dimensions
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

            if boxes.size > 0 and clss is not None:
                for box, cls, mask in zip(boxes, clss, masks):
                    name = result[0].names[int(cls)]
                    if name in [
                        "car",
                        "motorcycle",
                        "truck",
                        "airplane",
                    ]:  # Process specific classes
                        mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

                        # Define color map for different classes
                        color_map = {
                            "car": [0, 255, 255],  # Yellow
                            "motorcycle": [255, 0, 255],  # Magenta
                            "truck": [255, 255, 0],  # Cyan
                            "airplane": [255, 0, 0],  # Blue
                        }
                        color = color_map.get(
                            name, [255, 255, 255]
                        )  # Default to white if not found
                        mask[(mask == 255).all(-1)] = color
                        frame = cv2.addWeighted(frame, 1, mask, 0.5, 0)

                        # Draw bounding box
                        x1, y1, x2, y2 = box
                        cv2.rectangle(
                            frame, (x1, y1), (x2, y2), (0, 0, 255), 2
                        )  # Red bounding box
                        cv2.putText(
                            frame,
                            name,
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 0, 255),
                            2,
                        )

                        # Find contours in the mask
                        current_contours, _ = cv2.findContours(
                            mask[:, :, 0], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                        )

                        for contour in current_contours:
                            # Draw the contour
                            cv2.drawContours(
                                contour_frame, [contour], -1, (0, 255, 0), 2
                            )  # Green contours

                            # Calculate the bounding box for the contour
                            x, y, w, h = cv2.boundingRect(contour)
                            new_part = (x, y, w, h)

                            # Check for overlap with stitched parts
                            if not self.is_overlapping(new_part):
                                self.stitched_parts.append(new_part)  # Add new part
                                # Calculate real-world dimensions
                                nearest_distance = self.calculate_nearest_distance(
                                    contour
                                )
                                self.update_dimensions(nearest_distance)

            # Resize and convert the frame for display
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Display the results
            self.display_results(frame_rgb)

            # Show the frame
            cv2.imshow("Vehicle Dimensions System", frame_rgb)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def is_overlapping(self, new_part):
        x, y, w, h = new_part
        for part in self.stitched_parts:
            px, py, pw, ph = part
            if x < px + pw and x + w > px and y < py + ph and y + h > py:
                return True  # Overlapping detected
        return False

    def calculate_nearest_distance(self, contour):
        # Placeholder for depth calculation; replace with actual depth data if available
        return 1000  # Example fixed distance in mm

    def update_dimensions(self, nearest_distance):
        if nearest_distance is not None:
            # Calculate real-world dimensions
            ratioW = (nearest_distance * (-0.0012)) + 6.4944
            ratioH = (nearest_distance * (-0.0013)) + 6.6671
            realW = (
                self.calculate_total_length()
            )  # Calculate total length from stitched parts
            print(
                f"Length: {realW:.2f} mm, Height: {nearest_distance:.2f} mm, Width: {realW * ratioW:.2f} mm"
            )

    def calculate_total_length(self):
        total_length = 0
        for part in self.stitched_parts:
            _, _, w, _ = part
            total_length += w  # Sum the widths of all stitched parts
        return total_length

    def display_results(self, frame):
        # Display the results on the frame
        for part in self.stitched_parts:
            x, y, w, h = part
            cv2.putText(
                frame,
                f"Part: {w} mm",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )


if __name__ == "__main__":
    video_path = "car1.mp4"  # Replace with your video file path
    app = VehicleDimensionApp(video_path)
    app.process_video()
