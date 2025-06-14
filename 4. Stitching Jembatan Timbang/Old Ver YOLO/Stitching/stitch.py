from ultralytics import YOLO
import cv2
import pyrealsense2 as rs
import numpy as np

# Load the YOLOv8 model
model = YOLO("yolov8n-seg.pt")
bef_contour = None
saved_frames = []  # To store frames for stitching
stitching_window_name = "Stitched Image"


def customized_segmentation_realsense():
    global bef_contour  # Declare bef_contour as global
    bef_contour = None  # Initialize bef_contour
    # Configure Intel RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

    # Start streaming
    pipeline.start(config)

    try:
        while True:
            # Wait for a frame
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Convert the image to numpy array
            frame = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            h, w = frame.shape[:2]
            imgsz = max(w, h)

            # Process the depth data
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
            # boxes = result[0].boxes.xyxy.cpu().numpy().astype(int)
            # masks = (result[0].masks.data.cpu().numpy() * 255).astype("uint8")
            # clss = result[0].boxes.cls
            # Check if the result contains boxes and masks
            if result[0].boxes is not None:
                boxes = result[0].boxes.xyxy.cpu().numpy().astype(int)
                clss = result[0].boxes.cls

                # Check if masks are available
                if result[0].masks is not None:
                    masks = (result[0].masks.data.cpu().numpy() * 255).astype("uint8")
                else:
                    masks = np.zeros(
                        (h, w), dtype=np.uint8
                    )  # Create an empty mask if none are found
            else:
                boxes = np.empty((0, 4))  # No boxes detected
                masks = np.zeros(
                    (h, w), dtype=np.uint8
                )  # Create an empty mask if no boxes
                clss = np.empty((0,))  # No classes detected

            # Create a black background for contours
            contour_frame = np.zeros((h, w, 3), dtype=np.uint8)

            moving_objects = []  # To track moving objects

            for box, cls, mask in zip(boxes, clss, masks):
                name = result[0].names[int(cls)]
                if name in ["car", "motorcycle", "truck", "airplane"]:
                    mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
                    color_map = {
                        "car": [0, 255, 255],
                        "motorcycle": [255, 0, 255],
                        "truck": [255, 255, 0],
                        "airplane": [255, 255, 0],
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

                    if largest_contour is not None:
                        # Calculate the center of the largest contour
                        M = cv2.moments(largest_contour)
                        if M["m00"] != 0:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                        else:
                            cX, cY = 0, 0

                        # Extract the depth values for the pixels in the largest contour
                        depth_values = []
                        for point in largest_contour[
                            :, 0
                        ]:  # Iterate through the contour points
                            x, y = point[0], point[1]
                            depth_value = depth[
                                y, x
                            ]  # Get the depth value at the contour point
                            if depth_value > 0:  # Only consider valid depth values
                                depth_values.append(depth_value)

                    if current_contours:
                        cv2.drawContours(
                            contour_frame, current_contours, -1, (0, 255, 0), 2
                        )  # Green contours

                        # Calculate the bounding box for the current contour
                        x, y, w, h = cv2.boundingRect(current_contours[0])
                        if w > 10:
                            nearest_distance = min(depth_values)
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                            cv2.putText(
                                frame,
                                f"L: {w}, W: {h}, H: {2700 - nearest_distance} ",
                                (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.7,
                                (255, 255, 255),
                                2,
                            )

                        moving_objects.append((x, y, w, h))  # Store bounding box

                        # Check if the object is moving
                        if bef_contour is not None:
                            # Compare with previous bounding box
                            prev_x, prev_y, prev_w, prev_h = bef_contour
                            if (
                                abs(x - prev_x) > 5 or abs(y - prev_y) > 5
                            ):  # Movement threshold
                                # Save the current frame for stitching
                                saved_frames.append(frame.copy())
                        else:
                            # First frame, just save the bounding box
                            saved_frames.append(frame.copy())

                        # Update the previous contour
                        bef_contour = (x, y, w, h)

            # If there are saved frames, perform stitching
            if saved_frames:
                stitched_image = cv2.hconcat(
                    saved_frames
                )  # Simple horizontal stitching
                stitched_height, stitched_width = stitched_image.shape[:2]

                # Resize the stitched image to a smaller size
                scale_factor = 0.2  # Adjust this value to change the size
                new_width = int(stitched_width * scale_factor)
                new_height = int(stitched_height * scale_factor)
                resized_stitched_image = cv2.resize(
                    stitched_image, (new_width, new_height)
                )

                cv2.imshow("Stitched Image", resized_stitched_image)
                # cv2.imshow(stitching_window_name, stitched_image)

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
                cv2.drawContours(
                    frame, [scaled_contour.astype(np.int32)], -1, (255, 0, 0), 2
                )

            # Overlay depth information on the original frame
            depth_colored = cv2.applyColorMap(
                cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET
            )
            depth_colored = cv2.bitwise_and(
                depth_colored, depth_colored, mask=depth_mask
            )  # Apply depth mask

            # Show the original frame with segmentation
            cv2.imshow("Customized Segmentation RealSense", frame)
            # Show the contour frame
            cv2.imshow("Contours YOLO", contour_frame)
            # Show the depth information
            cv2.imshow("Depth Information", depth_colored)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()


# Example usage
customized_segmentation_realsense()
