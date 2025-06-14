from ultralytics import YOLO
import cv2
import pyrealsense2 as rs
import numpy as np

# Load the YOLOv8 model
model = YOLO("yolov8l-seg.pt")

stitched_image = None  # Global stitched image for vertical setup


def stitch_segments_vertical(new_segment, stitched_image):
    """
    Stitches new segments vertically.
    Args:
        new_segment: The new segment to be added.
        stitched_image: The current stitched image.
    Returns:
        Updated stitched image.
    """
    if stitched_image is None:
        # If no stitched image exists, initialize it with the first segment
        stitched_image = new_segment
    else:
        # Ensure both images have the same width before stacking
        height, width = stitched_image.shape[:2]
        new_height, new_width = new_segment.shape[:2]

        if width != new_width:
            # Resize the new segment to match the stitched image width
            new_segment = cv2.resize(new_segment, (width, new_height))

        # Stack the new segment below the stitched image
        stitched_image = np.vstack((stitched_image, new_segment))

    return stitched_image


def customized_segmentation_vertical_realsense():
    global stitched_image

    # Configure Intel RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    pipeline.start(config)

    try:
        while True:
            # Wait for a frame
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert the image to numpy array
            frame = np.asanyarray(color_frame.get_data())
            h, w = frame.shape[:2]
            imgsz = max(w, h)

            # YOLO segmentation
            result = model(frame, imgsz=imgsz)
            boxes = result[0].boxes.xyxy.cpu().numpy().astype(int)
            masks = (
                (result[0].masks.data.cpu().numpy() * 255).astype("uint8")
                if result[0].masks is not None
                else None
            )
            clss = result[0].boxes.cls

            # Process detected objects
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
                        # Extract region of interest (ROI)
                        y1, y2 = max(box[1], 0), min(box[3], h)
                        x1, x2 = max(box[0], 0), min(box[2], w)
                        roi = frame[y1:y2, x1:x2]

                        # Update the stitched image with the new ROI
                        stitched_image = stitch_segments_vertical(roi, stitched_image)

            # Display the stitched image
            if stitched_image is not None:
                cv2.imshow("Stitched Image - Vertical", stitched_image)

            # Display the current frame for reference
            cv2.imshow("Current Frame", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()


# Example usage
customized_segmentation_vertical_realsense()
