import tkinter as tk
from tkinter import ttk, messagebox
import sqlite3
import pyrealsense2 as rs
import numpy as np
import cv2
from PIL import Image, ImageTk
import threading
from ultralytics import YOLO  # Import YOLO from ultralytics
import time
import mysql.connector  # Import MySQL connector
import torch  # Import PyTorch for CUDA support


class VehicleDimensionApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Vehicle Dimensions System")
        self.root.geometry("1200x600")
        self.root.configure(bg="#eaeaea")

        # Initialize instance variables for database settings
        self.username_var = tk.StringVar()
        self.password_var = tk.StringVar()
        self.host_var = tk.StringVar()
        self.database_var = tk.StringVar()
        self.scale_var = tk.DoubleVar(value=0.90)
        self.height_surface_var = tk.IntVar(value=720)
        self.min_height_object_var = tk.IntVar(value=600)

        # Main frame
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E))

        # Title
        title_label = ttk.Label(
            main_frame, text="Vehicle Dimensions System", font=("Poppins", 16, "bold")
        )
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))

        # Scale Factor
        ttk.Label(main_frame, text="Scale Factor:", font=("Poppins", 10)).grid(
            row=1, column=0, sticky=tk.W
        )
        scale = ttk.Scale(
            main_frame, from_=0.0, to=5.0, variable=self.scale_var, orient=tk.HORIZONTAL
        )
        scale.grid(row=1, column=1, sticky=(tk.W, tk.E))

        # Label to display the scale factor value
        self.scale_value_label = ttk.Label(
            main_frame, text=f"{self.scale_var.get():.2f}", font=("Poppins", 10)
        )
        self.scale_value_label.grid(row=1, column=2, sticky=tk.W)

        # Bind the scale to update the label
        scale.bind("<Motion>", self.update_scale_value)

        # Height Surface
        ttk.Label(main_frame, text="Height Surface:", font=("Poppins", 10)).grid(
            row=2, column=0, sticky=tk.W
        )
        height_surface_entry = ttk.Entry(
            main_frame, textvariable=self.height_surface_var, font=("Poppins", 10)
        )
        height_surface_entry.grid(row=2, column=1, sticky=(tk.W, tk.E))

        # Edit button for Height Surface
        edit_button = ttk.Button(
            main_frame, text="Edit", command=self.edit_height_surface
        )
        edit_button.grid(row=2, column=2, padx=5)

        # Min Height Object
        ttk.Label(main_frame, text="Min Height Object:", font=("Poppins", 10)).grid(
            row=3, column=0, sticky=tk.W
        )
        min_height_object_entry = ttk.Entry(
            main_frame, textvariable=self.min_height_object_var, font=("Poppins", 10)
        )
        min_height_object_entry.grid(row=3, column=1, sticky=(tk.W, tk.E))

        update_button = ttk.Button(
            main_frame, text="Update", command=self.update_min_height
        )
        update_button.grid(row=3, column=2, padx=5)

        # USB Connection Status
        self.usb_status_label = ttk.Label(
            main_frame, text="USB Not Detected", font=("Poppins", 10), foreground="red"
        )
        self.usb_status_label.grid(row=4, column=0, columnspan=3, pady=(10, 0))

        # MySQL Connection Status
        self.mysql_status_label = ttk.Label(
            main_frame,
            text="MySQL Not Connected",
            font=("Poppins", 10),
            foreground="red",
        )
        self.mysql_status_label.grid(row=5, column=0, columnspan=3, pady=(10, 0))

        # Result
        result_frame = ttk.Frame(
            self.root, padding="20", relief=tk.RAISED, borderwidth=2
        )
        result_frame.grid(row=1, column=0, sticky=(tk.W, tk.E))

        ttk.Label(result_frame, text="Result", font=("Poppins", 14, "bold")).grid(
            row=0, column=0, columnspan=3
        )

        self.length_label = ttk.Label(
            result_frame, text="Length:", font=("Poppins", 10)
        )
        self.length_value_label = ttk.Label(
            result_frame, text="0 mm", font=("Poppins", 10)
        )
        self.width_label = ttk.Label(result_frame, text="Width:", font=("Poppins", 10))
        self.width_value_label = ttk.Label(
            result_frame, text="0 mm", font=("Poppins", 10)
        )
        self.height_label = ttk.Label(
            result_frame, text="Height:", font=("Poppins", 10)
        )
        self.height_value_label = ttk.Label(
            result_frame, text="0 mm", font=("Poppins", 10)
        )

        # Place the labels in the grid
        self.length_label.grid(row=1, column=0, sticky=tk.W)
        self.length_value_label.grid(row=1, column=1, sticky=tk.W)
        self.width_label.grid(row=2, column=0, sticky=tk.W)
        self.width_value_label.grid(row=2, column=1, sticky=tk.W)
        self.height_label.grid(row=3, column=0, sticky=tk.W)
        self.height_value_label.grid(row=3, column=1, sticky=tk.W)

        send_button = ttk.Button(result_frame, text="Send", command=self.send_data)
        send_button.grid(row=4, column=0, columnspan=3, pady=10)

        # Placeholder Image
        self.image_label = ttk.Label(
            self.root,
            text="Image Placeholder",
            background="#c0c0c0",
            width=40,
            relief=tk.SUNKEN,
        )
        self.image_label.grid(
            row=0, column=1, rowspan=2, padx=20, sticky=(tk.N, tk.S, tk.E, tk.W)
        )

        # Settings Button
        settings_button = ttk.Button(
            main_frame, text="Settings", command=self.open_settings
        )
        settings_button.grid(row=4, column=3, padx=10)

        # Styling
        for child in main_frame.winfo_children():
            child.grid_configure(padx=5, pady=5)

        for child in result_frame.winfo_children():
            child.grid_configure(padx=5, pady=5)

        # Initialize database
        self.init_db()

        # Load settings from the database
        self.load_settings()  # Load settings to populate the entries

        # Initialize RealSense pipeline
        self.pipeline = None
        self.thread = None
        self.running = False

        # Load YOLO model with CUDA support
        self.model = YOLO("yolov8l-seg.pt")  # Use 'cuda' for GPU

        # Check if CUDA is available
        if torch.cuda.is_available():
            print("CUDA is available. Using GPU.")
        else:
            print("CUDA is not available. Using CPU.")

        # Start the USB monitoring thread
        self.monitor_thread = threading.Thread(target=self.monitor_usb_connection)
        self.monitor_thread.daemon = True  # Daemonize thread
        self.monitor_thread.start()

        # Start the MySQL connection monitoring thread
        self.mysql_thread = threading.Thread(target=self.monitor_mysql_connection)
        self.mysql_thread.daemon = True  # Daemonize thread
        self.mysql_thread.start()

    def init_db(self):
        # Create database and table if not exists
        conn = sqlite3.connect("settings.db")
        cursor = conn.cursor()

        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS settings (
                id INTEGER PRIMARY KEY,
                username TEXT,
                password TEXT,
                host TEXT,
                database TEXT,
                scale_factor REAL,
                height_surface INTEGER,
                min_height_object INTEGER
            )
        """
        )

        # Insert default settings if the table is empty
        cursor.execute("SELECT COUNT(*) FROM settings")
        if cursor.fetchone()[0] == 0:
            cursor.execute(
                """
                INSERT INTO settings (id, username, password, host, database, scale_factor, height_surface, min_height_object)
                VALUES (1, "", "", "", "", 0.90, 720, 600)
            """
            )

        conn.commit()
        conn.close()

    def open_settings(self):
        # Open a pop-up window for settings
        settings_window = tk.Toplevel(self.root)
        settings_window.title("Database Settings")
        settings_window.geometry("300x300")

        # Title in pop-up
        title_label = ttk.Label(
            settings_window, text="Database Settings", font=("Poppins", 14, "bold")
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 10))

        # Entry for username
        ttk.Label(settings_window, text="Username:", font=("Poppins", 10)).grid(
            row=1, column=0, sticky=tk.W
        )
        self.username_var = tk.StringVar()
        username_entry = ttk.Entry(
            settings_window, textvariable=self.username_var, font=("Poppins", 10)
        )
        username_entry.grid(row=1, column=1, sticky=(tk.W, tk.E))

        # Entry for password
        ttk.Label(settings_window, text="Password:", font=("Poppins", 10)).grid(
            row=2, column=0, sticky=tk.W
        )
        self.password_var = tk.StringVar()
        self.password_entry = ttk.Entry(
            settings_window,
            textvariable=self.password_var,
            show="*",
            font=("Poppins", 10),
        )
        self.password_entry.grid(row=2, column=1, sticky=(tk.W, tk.E))

        # Button to show/hide password
        self.show_password_var = tk.BooleanVar(value=False)
        show_password_button = ttk.Checkbutton(
            settings_window,
            text="Show Password",
            variable=self.show_password_var,
            command=self.toggle_password,
        )
        show_password_button.grid(row=2, column=2, sticky=tk.W)

        # Entry for host
        ttk.Label(settings_window, text="Host:", font=("Poppins", 10)).grid(
            row=3, column=0, sticky=tk.W
        )
        self.host_var = tk.StringVar()
        host_entry = ttk.Entry(
            settings_window, textvariable=self.host_var, font=("Poppins", 10)
        )
        host_entry.grid(row=3, column=1, sticky=(tk.W, tk.E))

        # Entry for database
        ttk.Label(settings_window, text="Database:", font=("Poppins", 10)).grid(
            row=4, column=0, sticky=tk.W
        )
        self.database_var = tk.StringVar()
        database_entry = ttk.Entry(
            settings_window, textvariable=self.database_var, font=("Poppins", 10)
        )
        database_entry.grid(row=4, column=1, sticky=(tk.W, tk.E))

        # Save button
        save_button = ttk.Button(
            settings_window, text="Save", command=self.save_settings
        )
        save_button.grid(row=5, column=0, columnspan=2, pady=10)

        # Load existing settings
        self.load_settings()

    def toggle_password(self):
        # Toggle password visibility
        if self.show_password_var.get():
            self.password_entry.config(show="")
        else:
            self.password_entry.config(show="*")

    def load_settings(self):
        # Load settings from database
        conn = sqlite3.connect("settings.db")
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM settings WHERE id = 1")
        row = cursor.fetchone()
        print(row)
        if row:
            self.username_var.set(row[1])
            self.password_var.set(row[2])
            self.host_var.set(row[3])
            self.database_var.set(row[4])
            self.scale_var.set(row[5])  # Assuming scale_var is a DoubleVar
            self.height_surface_var.set(row[6])
            self.min_height_object_var.set(row[7])
        conn.close()

    def save_settings(self):
        # Save settings to database
        conn = sqlite3.connect("settings.db")
        cursor = conn.cursor()

        # Update the settings for id = 1
        cursor.execute(
            """
            UPDATE settings
            SET username = ?, password = ?, host = ?, database = ?
            WHERE id = 1
        """,
            (
                self.username_var.get(),
                self.password_var.get(),
                self.host_var.get(),
                self.database_var.get(),
            ),
        )

        conn.commit()
        conn.close()
        messagebox.showinfo("Success", "Settings saved successfully!")

    def edit_height_surface(self):
        # Create a new popup window to edit Height Surface
        edit_window = tk.Toplevel(self.root)
        edit_window.title("Edit Height Surface")
        edit_window.geometry("640x480")

        # Label for Height Surface
        ttk.Label(edit_window, text="Height Surface:", font=("Poppins", 10)).grid(
            row=0, column=0, sticky=tk.W, padx=10, pady=10
        )

        # Entry for Height Surface
        self.height_surface_edit_var = tk.StringVar(
            value=""
        )  # Store the variable for the entry
        height_surface_entry = ttk.Entry(
            edit_window, textvariable=self.height_surface_edit_var, font=("Poppins", 10)
        )
        height_surface_entry.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=10)

        # Save button to update the value
        save_button = ttk.Button(
            edit_window,
            text="Save",
            command=lambda: self.save_height_surface(
                self.height_surface_edit_var.get(), edit_window
            ),
        )
        save_button.grid(row=0, column=2, padx=10)

        # Create a label to display the depth image
        self.depth_frame_label = ttk.Label(edit_window)
        self.depth_frame_label.grid(row=1, column=0, columnspan=3)

        # Bind the click event to the depth frame label
        self.depth_frame_label.bind(
            "<Button-1>", lambda event: self.get_depth_at_click(event, edit_window)
        )

        # Start capturing frames for the popup
        self.update_depth_frame(edit_window)

    def update_depth_frame(self, window):
        # Get frames from the RealSense camera
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            return

        # Convert depth frame to numpy array
        self.depth_image = np.asanyarray(
            depth_frame.get_data()
        )  # Store depth image for access in click event

        # Normalize the depth image for display
        normalized_depth_image = cv2.normalize(
            self.depth_image, None, 0, 255, cv2.NORM_MINMAX
        )
        normalized_depth_image = np.uint8(normalized_depth_image)

        # Apply a jet color map to the depth image
        color_depth_image = cv2.applyColorMap(normalized_depth_image, cv2.COLORMAP_JET)

        # Resize the color depth image for display
        color_depth_image = cv2.resize(color_depth_image, (640, 480))

        # Convert the image to PhotoImage format
        self.photo = self.convert_cv_to_tk(color_depth_image)

        # Update the label with the new image
        self.depth_frame_label.configure(image=self.photo)
        self.depth_frame_label.image = self.photo

        # Call this function again after a delay
        if window.winfo_exists():  # Check if the window is still open
            window.after(10, lambda: self.update_depth_frame(window))

    def get_depth_at_click(self, event, window):
        # Get the x and y coordinates of the click
        x = event.x
        y = event.y

        # Retrieve the depth value at the clicked position
        if 0 <= x < 640 and 0 <= y < 480:  # Ensure the click is within the image bounds
            depth_value = self.depth_image[y, x]  # Get the depth value in millimeters
            self.height_surface_edit_var.set(
                depth_value
            )  # Update the entry with the depth value

    def convert_cv_to_tk(self, cv_image):
        """Convert an OpenCV image to a Tkinter PhotoImage."""
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
        height, width, _ = cv_image.shape
        return tk.PhotoImage(
            width=width, height=height, data=cv2.imencode(".ppm", cv_image)[1].tobytes()
        )

    def save_height_surface(self, new_value, window):
        # Update the height surface variable and the entry in the main window
        self.height_surface_var.set(new_value)

        # Save the new height surface value to the SQLite database
        conn = sqlite3.connect("settings.db")
        cursor = conn.cursor()

        # Update the height surface in the settings table for id = 1
        cursor.execute(
            """
            UPDATE settings
            SET height_surface = ?
            WHERE id = 1
        """,
            (new_value,),
        )

        conn.commit()
        conn.close()  # Close the database connection

        window.destroy()  # Close the popup

    def start_depth_camera(self):
        if self.pipeline is None:
            # Check for connected RealSense devices
            ctx = rs.context()
            devices = ctx.query_devices()

            if len(devices) == 0:
                self.usb_status_label.config(text="USB Not Detected", foreground="red")
                print("No RealSense device detected.")
                return

            # If a device is detected, proceed to start the pipeline
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

            try:
                self.pipeline.start(config)
                self.usb_status_label.config(text="USB Detected", foreground="green")
            except Exception as e:
                self.usb_status_label.config(text="USB Not Detected", foreground="red")
                print(f"Error starting camera: {e}")
                return

            self.running = True
            self.thread = threading.Thread(target=self.update_image)
            self.thread.start()

    def update_image(self):
        while self.running:
            # Get frame from pipeline
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Convert the image to numpy array
            frame = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            h, w = frame.shape[:2]
            imgsz = max(w, h)

            # Get the minimum height object value from the entry
            min_height_object = int(
                self.min_height_object_var.get()
            )  # Convert to integer

            # Run YOLO model
            result = self.model(frame, imgsz=imgsz)
            masks = (
                (result[0].masks.data.cpu().numpy() * 255).astype("uint8")
                if result[0].masks is not None
                else None
            )
            clss = result[0].boxes.cls

            # Initialize variables to track dimensions
            detected = False
            accumulated_length = 0
            accumulated_width = 0
            accumulated_height = 0
            object_count = 0

            if masks is not None and clss is not None:
                for mask, cls in zip(masks, clss):
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
                        # "airplane",
                        "person",
                    ]:
                        mask_resized = cv2.resize(
                            mask, (frame.shape[1], frame.shape[0])
                        )
                        contours, _ = cv2.findContours(
                            mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                        )

                        for contour in contours:
                            # Calculate the depth values for the pixels in the contour
                            depth_values = []
                            for point in contour[
                                :, 0
                            ]:  # Iterate through the contour points
                                x, y = point[0], point[1]
                                depth_value = depth[
                                    y, x
                                ]  # Get the depth value at the contour point
                                if depth_value > 0:  # Only consider valid depth values
                                    depth_values.append(depth_value)

                            if depth_values:
                                detected = True  # Object is detected
                                nearest_distance = min(depth_values)
                                # Draw the contour and display the nearest distance
                                cv2.drawContours(
                                    frame, [contour], -1, (0, 255, 0), 2
                                )  # Green contours
                                x, y, w, h = cv2.boundingRect(contour)
                                cv2.rectangle(
                                    frame, (x, y), (x + w, y + h), (0, 0, 255), 2
                                )

                                # Calculate real-world dimensions
                                ratioW = (nearest_distance * (-0.0012)) + 6.4944
                                ratioH = (nearest_distance * (-0.0013)) + 6.6671
                                realW = w / ratioW * self.scale_var.get()
                                realH = h / ratioH * self.scale_var.get()

                                # Accumulate dimensions
                                accumulated_length += realW
                                accumulated_width += realH
                                accumulated_height += (
                                    self.height_surface_var.get() - nearest_distance
                                )
                                object_count += 1

            # If an object was detected, update the labels
            if detected and object_count > 0:
                self.length_value_label.config(
                    text=f"{accumulated_length / object_count:.2f} mm"
                )
                self.width_value_label.config(
                    text=f"{accumulated_width / object_count:.2f} mm"
                )
                self.height_value_label.config(
                    text=f"{accumulated_height / object_count:.2f} mm"
                )
            else:
                # If no object is detected, retain the last known dimensions
                self.length_value_label.config(
                    text=self.length_value_label.cget("text")
                )
                self.width_value_label.config(text=self.width_value_label.cget("text"))
                self.height_value_label.config(
                    text=self.height_value_label.cget("text")
                )

            # Resize and convert the frame for display
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_rgb = cv2.resize(frame_rgb, (640, 480))

            # Convert to PIL Image and update the label
            frame_pil = Image.fromarray(frame_rgb)
            frame_tk = ImageTk.PhotoImage(image=frame_pil)
            self.image_label.config(image=frame_tk)
            self.image_label.image = frame_tk

    def stop_camera(self):
        if self.pipeline:
            self.running = False
            self.pipeline.stop()
            self.pipeline = None
            self.usb_status_label.config(text="USB Not Detected", foreground="red")

    def update_min_height(self):
        # Get the new values from the variables
        new_min_height = self.min_height_object_var.get()
        new_scale_factor = self.scale_var.get()
        new_height_surface = (
            self.height_surface_var.get()
        )  # Get the height surface value

        # Print the updated values
        print(f"Height Surface updated to: {new_height_surface}")
        print(f"Min Height Object updated to: {new_min_height}")

        # Save the new scale factor, min height object, and height surface to the SQLite database
        conn = sqlite3.connect("settings.db")
        cursor = conn.cursor()

        # Update the scale factor, min height object, and height surface in the settings table for id = 1
        cursor.execute(
            """
            UPDATE settings
            SET scale_factor = ?, min_height_object = ?, height_surface = ?
            WHERE id = 1
        """,
            (new_scale_factor, new_min_height, new_height_surface),
        )

        conn.commit()
        conn.close()
        messagebox.showinfo(
            "Success",
            "Scale factor, minimum height object, and height surface updated successfully!",
        )

    def send_data(self):
        print("Data sent!")

    def update_scale_value(self, event):
        # Update the scale value label
        self.scale_value_label.config(text=f"{self.scale_var.get():.2f}")

    def on_closing(self):
        # Stop the pipeline when closing the application
        self.stop_camera()
        self.root.destroy()

    def monitor_usb_connection(self):
        while True:
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                self.usb_status_label.config(text="USB Not Detected", foreground="red")
                self.stop_camera()
            else:
                if self.pipeline is None:
                    self.start_depth_camera()
                self.usb_status_label.config(text="USB Detected", foreground="green")
            time.sleep(1)  # Check every second

    def monitor_mysql_connection(self):
        while True:
            try:
                # Attempt to connect to MySQL database
                conn = mysql.connector.connect(
                    host=self.host_var.get(),
                    user=self.username_var.get(),
                    password=self.password_var.get(),
                    database=self.database_var.get(),
                )
                self.mysql_status_label.config(
                    text="MySQL Connected", foreground="green"
                )
                conn.close()
            except mysql.connector.Error as err:
                self.mysql_status_label.config(
                    text="MySQL Not Connected", foreground="red"
                )
            time.sleep(1)  # Check every second


if __name__ == "__main__":
    root = tk.Tk()
    app = VehicleDimensionApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)  # Ensure pipeline stops on close
    root.mainloop()
