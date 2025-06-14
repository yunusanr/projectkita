from ultralytics import YOLO
import cv2
import numpy as np

# Load the YOLOv8 model
model = YOLO("yolov8m-seg.pt")
saved_frames = []  # To store frames for stitching
stitching_window_name = "Stitched Image"


def customized_segmentation_video(video_path):
    # Open the video file
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    try:
        previous_boxes = []  # To store previous bounding boxes
        stable_frame_count = 0  # Counter for stable frames
        movement_detected = False  # Flag for movement detection
        movement_threshold = 5  # Adjust this value as needed
        required_stable_frames = 5  # Number of frames to confirm movement

        while True:
            # Read a frame from the video
            ret, frame = cap.read()
            if not ret:
                break  # Exit if no frames are left

            h, w = frame.shape[:2]
            imgsz = max(w, h)

            # Create a dummy depth image for testing (you can replace this with actual depth data if available)
            depth = np.zeros((h, w), dtype=np.uint16)  # Dummy depth image
            depth_mask = np.zeros((h, w), dtype=np.uint8)
            depth_mask[(depth > 0) & (depth < 2500)] = (
                255  # Mask for depth values < 2500 and not 0
            )

            # Find contours in the depth mask
            contours, _ = cv2.findContours(
                depth_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea) if contours else None

            result = model(frame, imgsz=imgsz)
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
                current_boxes = []
                for box, cls, mask in zip(boxes, clss, masks):
                    name = result[0].names[int(cls)]
                    if name in [
                        "car",
                        "motorcycle",
                        "truck",
                        "airplane",
                        "laptop",
                        "person",
                        "book",
                    ]:
                        mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
                        color_map = {
                            "car": [0, 255, 255],  # Yellow
                            "motorcycle": [255, 0, 255],  # Magenta
                            "book": [255, 255, 0],  # Cyan
                            "laptop": [255, 255, 0],  # Cyan
                            "person": [0, 0, 255],  # Red
                            "truck": [255, 165, 0],  # Orange
                            "airplane": [128, 0, 128],  # Purple
                        }
                        mask[(mask == 255).all(-1)] = color_map[name]
                        frame = cv2.addWeighted(frame, 1, mask, 0.5, 0)

                        # Draw contours on the black background
                        contours, _ = cv2.findContours(
                            mask[:, :, 0], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                        )
                        cv2.drawContours(
                            contour_frame, contours, -1, (0, 255, 0), 2
                        )  # Green contours
                        current_contours, _ = cv2.findContours(
                            mask[:, :, 0], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                        )

                        if current_contours:
                            # Calculate the bounding box for the current contour
                            x, y, w, h = cv2.boundingRect(current_contours[0])
                            current_boxes.append((x, y, w, h))

                            # Initialize minimum distance
                            min_distance = float("inf")

                            # Iterate through all pixels in the bounding box
                            for i in range(y, y + h):
                                for j in range(x, x + w):
                                    # Ensure we are within the bounds of the depth image
                                    if (
                                        0 <= i < depth.shape[0]
                                        and 0 <= j < depth.shape[1]
                                    ):
                                        depth_value = depth[
                                            i, j
                                        ]  # Depth in millimeters
                                        if (
                                            depth_value > 0
                                        ):  # Ignore invalid depth values
                                            min_distance = min(
                                                min_distance, depth_value
                                            )

                            # Convert minimum distance to meters
                            if min_distance != float("inf"):
                                min_distance_m = (
                                    min_distance / 1000.0
                                )  # Convert to meters
                            else:
                                min_distance_m = (
                                    0  # Handle case where no valid depth was found
                                )

                            # Draw a rectangle around the detected object
                            cv2.rectangle(
                                frame, (x, y), (x + w, y + h), (0, 0, 255), 2
                            )  # Red rectangle

                            # Prepare the text to display
                            text = f"W: {w}, H: {h}, D: {min_distance_m:.2f} m"  # Add distance to text

                            # Put text on the top-left corner of the rectangle
                            cv2.putText(
                                frame,
                                text,
                                (x, y - 10),  # Position the text above the rectangle
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.7,  # Font scale
                                (255, 255, 255),  # White color for the text
                                2,  # Thickness
                            )

                            # Check for movement
                            if previous_boxes:
                                for prev_box, curr_box in zip(
                                    previous_boxes, current_boxes
                                ):
                                    prev_x, prev_y, prev_w, prev_h = prev_box
                                    curr_x, curr_y, curr_w, curr_h = curr_box

                                    # Calculate the center of the previous and current boxes
                                    prev_center = (
                                        prev_x + prev_w // 2,
                                        prev_y + prev_h // 2,
                                    )
                                    curr_center = (
                                        curr_x + curr_w // 2,
                                        curr_y + curr_h // 2,
                                    )

                                    # Calculate the distance between the centers
                                    distance = np.linalg.norm(
                                        np.array(prev_center) - np.array(curr_center)
                                    )

                                    # If the distance is significant, consider it as moving
                                    if distance > movement_threshold:
                                        stable_frame_count += 1
                                        if stable_frame_count >= required_stable_frames:
                                            movement_detected = True
                                    else:
                                        stable_frame_count = (
                                            0  # Reset if no movement detected
                                        )

                            # Update the previous boxes
                            previous_boxes = current_boxes

            # Draw the largest contour on the RGB frame if it exists
            if largest_contour is not None:
                # Calculate the center of the largest contour
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                # Scale factor
                scale_factor = 1.5  # Adjust this value as needed

                # Scale the contour while keeping the center fixed
                scaled_contour = largest_contour * scale_factor

                # Adjust the contour points to keep the center aligned
                scaled_contour[:, :, 0] = scaled_contour[:, :, 0] + (
                    cX * (1 - scale_factor)
                )
                scaled_contour[:, :, 1] = scaled_contour[:, :, 1] + (
                    cY * (1 - scale_factor)
                )

                # Draw the scaled largest contour in blue
                # cv2.drawContours(frame, [scaled_contour.astype(np.int32)], -1, (255, 0, 0), 2)

            # Overlay depth information on the original frame
            depth_colored = cv2.applyColorMap(
                cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET
            )
            depth_colored = cv2.bitwise_and(
                depth_colored, depth_colored, mask=depth_mask
            )  # Apply depth mask

            # Show the original frame with segmentation
            cv2.imshow("Customized Segmentation Video", frame)
            # Show the contour frame
            cv2.imshow("Contours YOLO", contour_frame)
            # Show the depth information
            cv2.imshow("Depth Information", depth_colored)

            # If movement is detected, save the current frame for stitching
            if movement_detected:
                saved_frames.append(frame)  # Save the current frame for stitching

            # If there are saved frames, perform stitching
            if len(saved_frames) > 0:
                stitched_image = cv2.hconcat(
                    saved_frames
                )  # Simple horizontal stitching
                stitched_height, stitched_width = stitched_image.shape[:2]

                # Resize the stitched image to a smaller size
                scale_factor = 0.25  # Adjust this value to change the size
                new_width = int(stitched_width * scale_factor)
                new_height = int(stitched_height * scale_factor)
                resized_stitched_image = cv2.resize(
                    stitched_image, (new_width, new_height)
                )

                cv2.imshow("Stitched Image", resized_stitched_image)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        # Release the video capture object
        cap.release()
        cv2.destroyAllWindows()


# Example usage
video_path = "test4.mp4"  # Replace with your video file path
customized_segmentation_video(video_path)
