from ultralytics import YOLO
import cv2
import pyrealsense2 as rs
import numpy as np

# Load the YOLOv8 model
model = YOLO("yolov8n-seg.pt")
saved_frames = []  # To store frames for stitching
stitching_window_name = "Stitched Image"


def initialize_pipeline():
    """Initialize the Intel RealSense pipeline."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    pipeline.start(config)
    return pipeline


def process_depth_data(depth):
    """Create a depth mask from the depth data."""
    depth_mask = np.zeros(depth.shape, dtype=np.uint8)
    depth_mask[(depth > 0) & (depth < 2500)] = (
        255  # Mask for depth values < 2500 and not 0
    )
    return depth_mask


def find_largest_contour(depth_mask):
    """Find the largest contour in the depth mask."""
    contours, _ = cv2.findContours(
        depth_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    return max(contours, key=cv2.contourArea) if contours else None


def process_frame(frame, depth, previous_boxes):
    """Process the frame for object detection and movement detection."""
    h, w = frame.shape[:2]
    imgsz = max(w, h)

    depth_mask = process_depth_data(depth)
    largest_contour = find_largest_contour(depth_mask)

    result = model(frame, imgsz=imgsz)
    boxes = result[0].boxes.xyxy.cpu().numpy().astype(int)
    masks = (
        (result[0].masks.data.cpu().numpy() * 255).astype("uint8")
        if result[0].masks is not None
        else None
    )
    clss = result[0].boxes.cls

    return boxes, masks, clss, largest_contour, depth_mask, result


def draw_objects(frame, boxes, masks, clss, result):
    """Draw detected objects and their masks on the frame."""
    contour_frame = np.zeros(frame.shape, dtype=np.uint8)
    current_boxes = []

    if boxes.size > 0 and clss is not None:
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

                if contours:
                    x, y, w, h = cv2.boundingRect(contours[0])
                    current_boxes.append((x, y, w, h))
                    cv2.rectangle(
                        frame, (x, y), (x + w, y + h), (0, 0, 255), 2
                    )  # Red rectangle
                    cv2.putText(
                        frame,
                        f"W: {w}, H: {h}",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                    )

    return frame, contour_frame, current_boxes


def detect_movement(
    previous_boxes, current_boxes, movement_threshold, required_stable_frames
):
    """Detect movement based on bounding box positions."""
    stable_frame_count = 0
    movement_detected = False

    if previous_boxes:
        for prev_box, curr_box in zip(previous_boxes, current_boxes):
            prev_x, prev_y, prev_w, prev_h = prev_box
            curr_x, curr_y, curr_w, curr_h = curr_box

            prev_center = (prev_x + prev_w // 2, prev_y + prev_h // 2)
            curr_center = (curr_x + curr_w // 2, curr_y + curr_h // 2)

            distance = np.linalg.norm(np.array(prev_center) - np.array(curr_center))

            if distance > movement_threshold:
                stable_frame_count += 1
                if stable_frame_count >= required_stable_frames:
                    movement_detected = True
            else:
                stable_frame_count = 0  # Reset if no movement detected

    return movement_detected, current_boxes


def overlay_depth_information(frame, depth, depth_mask):
    """Overlay depth information on the original frame."""
    depth_colored = cv2.applyColorMap(
        cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET
    )
    depth_colored = cv2.bitwise_and(
        depth_colored, depth_colored, mask=depth_mask
    )  # Apply depth mask
    return depth_colored


def stitch_frames(saved_frames):
    """Stitch saved frames together."""
    if len(saved_frames) > 0:
        stitched_image = cv2.hconcat(saved_frames)  # Simple horizontal stitching
        stitched_height, stitched_width = stitched_image.shape[:2]
        scale_factor = 0.25  # Adjust this value to change the size
        new_width = int(stitched_width * scale_factor)
        new_height = int(stitched_height * scale_factor)
        return cv2.resize(stitched_image, (new_width, new_height))
    return None


def customized_segmentation_realsense():
    """Main function for customized segmentation using RealSense."""
    pipeline = initialize_pipeline()
    previous_boxes = []  # To store previous bounding boxes

    try:
        movement_threshold = 25  # Adjust this value as needed
        required_stable_frames = 10  # Number of frames to confirm movement

        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())

            boxes, masks, clss, largest_contour, depth_mask, result = process_frame(
                frame, depth, previous_boxes
            )
            frame, contour_frame, current_boxes = draw_objects(
                frame, boxes, masks, clss, result
            )

            movement_detected, previous_boxes = detect_movement(
                previous_boxes,
                current_boxes,
                movement_threshold,
                required_stable_frames,
            )

            if largest_contour is not None:
                # Draw the largest contour on the RGB frame if it exists
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                # Scale the contour while keeping the center fixed
                scale_factor = 1.5  # Adjust this value as needed
                scaled_contour = largest_contour * scale_factor
                scaled_contour[:, :, 0] += cX * (1 - scale_factor)
                scaled_contour[:, :, 1] += cY * (1 - scale_factor)

                # Draw the scaled largest contour in blue
                cv2.drawContours(
                    frame, [scaled_contour.astype(np.int32)], -1, (255, 0, 0), 2
                )

            depth_colored = overlay_depth_information(frame, depth, depth_mask)

            # Show the original frame with segmentation
            cv2.imshow("Customized Segmentation RealSense", frame)
            # Show the contour frame
            cv2.imshow("Contours YOLO", contour_frame)
            # Show the depth information
            cv2.imshow("Depth Information", depth_colored)

            if movement_detected:
                saved_frames.append(frame)  # Save the current frame for stitching

            stitched_image = stitch_frames(saved_frames)
            if stitched_image is not None:
                cv2.imshow(stitching_window_name, stitched_image)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()


# Example usage
customized_segmentation_realsense()
