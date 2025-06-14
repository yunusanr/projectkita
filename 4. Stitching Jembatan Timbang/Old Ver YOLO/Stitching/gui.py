import tkinter as tk
from tkinter import ttk, messagebox
import sqlite3
import pyrealsense2 as rs
import numpy as np
import cv2
from PIL import Image, ImageTk
import threading
from ultralytics import YOLO  # Import YOLO from ultralytics


class VehicleDimensionApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Vehicle Dimensions System")
        self.root.geometry("1300x600")
        self.root.configure(bg="#eaeaea")

        # Frame utama
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E))

        # Judul
        title_label = ttk.Label(
            main_frame, text="Vehicle Dimensions System", font=("Poppins", 16, "bold")
        )
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))

        # Dropdown for camera mode
        self.selected_camera = tk.StringVar()
        camera_options = ["Depth Camera", "ROI Object", "RGB Camera"]
        camera_dropdown = ttk.Combobox(
            main_frame,
            textvariable=self.selected_camera,
            values=camera_options,
            font=("Poppins", 10),
        )
        camera_dropdown.grid(row=0, column=3, padx=10)
        camera_dropdown.current(0)  # Set default value
        camera_dropdown.bind("<<ComboboxSelected>>", self.camera_selected)

        # Scale Factor
        ttk.Label(main_frame, text="Scale Factor:", font=("Poppins", 10)).grid(
            row=1, column=0, sticky=tk.W
        )
        self.scale_var = tk.DoubleVar(value=0.90)
        scale = ttk.Scale(
            main_frame, from_=0.1, to=2.0, variable=self.scale_var, orient=tk.HORIZONTAL
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
        self.height_surface_var = tk.StringVar(value="720 mm")
        height_surface_entry = ttk.Entry(
            main_frame, textvariable=self.height_surface_var, font=("Poppins", 10)
        )
        height_surface_entry.grid(row=2, column=1, sticky=(tk.W, tk.E))

        # Min Height Object
        ttk.Label(main_frame, text="Min Height Object:", font=("Poppins", 10)).grid(
            row=3, column=0, sticky=tk.W
        )
        self.min_height_object_var = tk.StringVar(value="600 mm")
        min_height_object_entry = ttk.Entry(
            main_frame, textvariable=self.min_height_object_var, font=("Poppins", 10)
        )
        min_height_object_entry.grid(row=3, column=1, sticky=(tk.W, tk.E))

        update_button = ttk.Button(
            main_frame, text="Update", command=self.update_min_height
        )
        update_button.grid(row=3, column=2, padx=5)

        # Hasil
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
            result_frame, text="400 mm", font=("Poppins", 10)
        )
        self.width_label = ttk.Label(result_frame, text="Width:", font=("Poppins", 10))
        self.width_value_label = ttk.Label(
            result_frame, text="400 mm", font=("Poppins", 10)
        )
        self.height_label = ttk.Label(
            result_frame, text="Height:", font=("Poppins", 10)
        )
        self.height_value_label = ttk.Label(
            result_frame, text="400 mm", font=("Poppins", 10)
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

        # Placeholder Gambar
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

        # Tombol Settings
        settings_button = ttk.Button(
            main_frame, text="Settings", command=self.open_settings
        )
        settings_button.grid(row=4, column=3, padx=10)

        # Styling
        for child in main_frame.winfo_children():
            child.grid_configure(padx=5, pady=5)

        for child in result_frame.winfo_children():
            child.grid_configure(padx=5, pady=5)

        # Inisialisasi database
        self.init_db()

        # Initialize RealSense pipeline
        self.pipeline = None
        self.thread = None
        self.running = False

        # Load YOLO model
        self.model = YOLO("yolov8l-seg.pt")  # Load your YOLOv8 model here

    def init_db(self):
        # Membuat database dan tabel jika belum ada
        conn = sqlite3.connect("settings.db")
        cursor = conn.cursor()
        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS settings (
                id INTEGER PRIMARY KEY,
                username TEXT,
                password TEXT,
                host TEXT,
                database TEXT
            )
        """
        )
        conn.commit()
        conn.close()

    def open_settings(self):
        # Membuka jendela pop-up untuk pengaturan
        settings_window = tk.Toplevel(self.root)
        settings_window.title("Setting Database")
        settings_window.geometry("300x300")

        # Judul di pop-up
        title_label = ttk.Label(
            settings_window, text="Setting Database", font=("Poppins", 14, "bold")
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 10))

        # Entri untuk username
        ttk.Label(settings_window, text="Username:", font=("Poppins", 10)).grid(
            row=1, column=0, sticky=tk.W
        )
        self.username_var = tk.StringVar()
        username_entry = ttk.Entry(
            settings_window, textvariable=self.username_var, font=("Poppins", 10)
        )
        username_entry.grid(row=1, column=1, sticky=(tk.W, tk.E))

        # Entri untuk password
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

        # Tombol untuk melihat/menghiding password
        self.show_password_var = tk.BooleanVar(value=False)
        show_password_button = ttk.Checkbutton(
            settings_window,
            text="Show Password",
            variable=self.show_password_var,
            command=self.toggle_password,
        )
        show_password_button.grid(row=2, column=2, sticky=tk.W)

        # Entri untuk host
        ttk.Label(settings_window, text="Host:", font=("Poppins", 10)).grid(
            row=3, column=0, sticky=tk.W
        )
        self.host_var = tk.StringVar()
        host_entry = ttk.Entry(
            settings_window, textvariable=self.host_var, font=("Poppins", 10)
        )
        host_entry.grid(row=3, column=1, sticky=(tk.W, tk.E))

        # Entri untuk database
        ttk.Label(settings_window, text="Database:", font=("Poppins", 10)).grid(
            row=4, column=0, sticky=tk.W
        )
        self.database_var = tk.StringVar()
        database_entry = ttk.Entry(
            settings_window, textvariable=self.database_var, font=("Poppins", 10)
        )
        database_entry.grid(row=4, column=1, sticky=(tk.W, tk.E))

        # Tombol Save
        save_button = ttk.Button(
            settings_window, text="Save", command=self.save_settings
        )
        save_button.grid(row=5, column=0, columnspan=2, pady=10)

        # Load existing settings
        self.load_settings()

    def toggle_password(self):
        # Mengubah tampilan password
        if self.show_password_var.get():
            self.password_entry.config(show="")
        else:
            self.password_entry.config(show="*")

    def load_settings(self):
        # Memuat pengaturan dari database
        conn = sqlite3.connect("settings.db")
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM settings ORDER BY id DESC LIMIT 1")
        row = cursor.fetchone()
        if row:
            self.username_var.set(row[1])
            self.password_var.set(row[2])
            self.host_var.set(row[3])
            self.database_var.set(row[4])
        conn.close()

    def save_settings(self):
        # Menyimpan pengaturan ke database
        conn = sqlite3.connect("settings.db")
        cursor = conn.cursor()
        cursor.execute(
            """
            INSERT INTO settings (username, password, host, database)
            VALUES (?, ?, ?, ?)
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

    def camera_selected(self, event):
        selected = self.selected_camera.get()
        if selected == "Depth Camera":
            self.start_depth_camera()
        elif selected == "RGB Camera":
            print("Displaying RGB Camera from Intel RealSense.")
            self.stop_camera()
        else:
            print("ROI Object selected.")
            self.stop_camera()

    def start_depth_camera(self):
        if self.pipeline is None:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.pipeline.start(config)

            self.running = True
            self.thread = threading.Thread(target=self.update_image)
            self.thread.start()

    def update_image(self):
        while self.running:
            # Ambil frame dari pipeline
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

            # Process the depth data
            depth_mask = np.zeros((h, w), dtype=np.uint8)
            depth_mask[(depth > 0) & (depth < 2500)] = (
                255  # Mask for depth values < 2500 and not 0
            )

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

                        # Find contours in the mask
                        current_contours, _ = cv2.findContours(
                            mask[:, :, 0], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                        )

                        for contour in current_contours:
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
                                nearest_distance = min(depth_values)
                                # Draw the contour and display the nearest distance
                                cv2.drawContours(
                                    contour_frame, [contour], -1, (0, 255, 0), 2
                                )  # Green contours
                                x, y, w, h = cv2.boundingRect(contour)
                                cv2.rectangle(
                                    frame, (x, y), (x + w, y + h), (0, 0, 255), 2
                                )
                                cv2.putText(
                                    frame,
                                    f"Nearest Dist: {nearest_distance}",
                                    (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7,
                                    (0, 255, 0),
                                    2,
                                )

                                # Calculate real-world dimensions
                                ratioW = (nearest_distance * (-0.0012)) + 6.4944
                                ratioH = (nearest_distance * (-0.0013)) + 6.6671
                                realW = w / ratioW
                                realH = h / ratioH
                                self.length_value_label.config(text=f"{realW:.2f} mm")
                                self.width_value_label.config(text=f"{realH:.2f} mm")
                                self.height_value_label.config(
                                    text=f"{nearest_distance:.2f} mm"
                                )
                                print(realW, realH, nearest_distance)

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

    def update_min_height(self):
        print(f"Height Surface updated to: {self.height_surface_var.get()}")
        print(f"Min Height Object updated to: {self.min_height_object_var.get()}")

    def send_data(self):
        print("Data sent!")

    def update_scale_value(self, event):
        # Update the scale value label
        self.scale_value_label.config(text=f"{self.scale_var.get():.2f}")

    def on_closing(self):
        # Stop the pipeline when closing the application
        self.stop_camera()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = VehicleDimensionApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)  # Ensure pipeline stops on close
    root.mainloop()
