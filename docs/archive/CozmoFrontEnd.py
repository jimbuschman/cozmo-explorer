import pycozmo
import tkinter as tk
from PIL import Image, ImageTk, ImageDraw
import numpy as np
import math
import time

class SimpleCozmoController:    
    def __init__(self):
        self.cozmo = None
        self.speed = 100
        self.camera_active = False
        self.current_frame = None
        self.head_angle = 0.0 
        self.battery_voltage = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.pose_angle = 0.0
        self.path_history = []  # Stores (x, y) positions
        self.max_path_points = 500  # Limit history size
        self.log_file = None
        self.log_start_time = 0
        self.eye_color = (0, 255, 0)  # Default green
        self.animations = {
            "Happy": "anim_happy_01",
            "Sad": "anim_sad_01",
            "Excited": "anim_excited_01",
            "Angry": "anim_anger_01",
            "Dizzy": "anim_dizzy_reaction_medium_01",
            "Win": "anim_memorymatch_winhand_01",
            "Lose": "anim_memorymatch_losehand_01",
            "Spark": "anim_sparking_01",
            "Sing": "anim_singing_01",
            "Bored": "anim_bored_01"
        }

        # Head control with physical synchronization
        min_angle = pycozmo.robot.MIN_HEAD_ANGLE.radians  # Down limit (~ -0.436)
        max_angle = pycozmo.robot.MAX_HEAD_ANGLE.radians  # Up limit (~ 0.776)

        # Calculate intermediate steps:
        step = (max_angle - min_angle) / 4  # 4 intervals = 5 positions
        self.head_positions = [
            min_angle,           # Down Down (lowest)
            min_angle + step,    # Down
            min_angle + 2*step,  # Center-ish (more precise center)
            min_angle + 3*step,  # Up
            max_angle            # Up Up (highest)
        ]
        self.current_head_index = 2  # Assume center
        self.head_physically_moving = False

        # Arm control with physical synchronization
        self.lift_positions = [
            0.0,    # Down
            0.3,     # Mid-low
            0.6,     # Mid-high
            1.0      # Up
        ]
        self.current_lift_index = 0  # Start with arm down
        self.lift_physically_moving = False

    def connect(self):
        try:
            self.cozmo = pycozmo.Client()
            self.cozmo.start()
            self.cozmo.connect()
            self.cozmo.wait_for_robot()
            
            self._sync_head_position()
            self._sync_lift_position()
    
            self.cozmo.enable_camera(enable=True, color=True)
            self.cozmo.add_handler(pycozmo.event.EvtNewRawCameraImage, self.on_camera_image)
            self.camera_active = True
    
            # Register for the specific packet type that contains state updates
            self.cozmo.add_handler(pycozmo.protocol_encoder.RobotState, self._on_robot_state_updated)
        
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def _sync_head_position(self):
        if not self.cozmo:
            return
        self.cozmo.set_head_angle(0.0)
        self.head_angle = 0.0
        self.current_head_index = 2
        self.head_physically_moving = False

    # def _sync_lift_position(self):
    #     if not self.cozmo:
    #         return
    #     self.cozmo.set_lift_height(5)        
    #     self.current_lift_index = 0
    #     self.lift_physically_moving = False

    def _sync_lift_position(self):
        if not self.cozmo:
            return
        try:
            print("Attempting to sync lift position...")
            self.cozmo.set_lift_height(0.0)
            print("Lift sync command sent")
        except Exception as e:
            print(f"Lift sync error: {e}")
        self.current_lift_index = 0
        self.lift_physically_moving = False
    def turn_left_arc(self, radius=200):
        """Turn left in an arc like a car"""
        if self.cozmo:
            # For arc turning: inner wheel slower, outer wheel faster
            # The ratio depends on the radius
            outer_speed = self.speed
            inner_speed = self.speed * 0.5  # Adjust this ratio for tighter/wider turns
            self.cozmo.drive_wheels(lwheel_speed=inner_speed, rwheel_speed=outer_speed)

    def turn_right_arc(self, radius=200):
        """Turn right in an arc like a car"""
        if self.cozmo:
            outer_speed = self.speed
            inner_speed = self.speed * 0.5
            self.cozmo.drive_wheels(lwheel_speed=outer_speed, rwheel_speed=inner_speed)

    def reverse_turn_left(self):
        """Turn left while reversing (like a car in reverse)"""
        if self.cozmo:
            # When reversing, left turn means right wheel slower
            self.cozmo.drive_wheels(lwheel_speed=-self.speed, rwheel_speed=-self.speed * 0.7)

    def reverse_turn_right(self):
        """Turn right while reversing (like a car in reverse)"""
        if self.cozmo:
            # When reversing, right turn means left wheel slower
            self.cozmo.drive_wheels(lwheel_speed=-self.speed * 0.7, rwheel_speed=-self.speed)

    def gentle_turn_left(self):
        """Very gentle left turn for trailers"""
        if self.cozmo:
            self.cozmo.drive_wheels(lwheel_speed=self.speed * 0.7, rwheel_speed=self.speed)

    def gentle_turn_right(self):
        """Very gentle right turn for trailers"""
        if self.cozmo:
            self.cozmo.drive_wheels(lwheel_speed=self.speed, rwheel_speed=self.speed * 0.7)
    def raise_arm(self):
        print(f"raise_arm called - index: {self.current_lift_index}")
    
        if self.cozmo and not self.lift_physically_moving and self.current_lift_index < len(self.lift_positions) - 1:
            self.lift_physically_moving = True
            self.current_lift_index += 1
        
            try:
                # Try move_lift with speed control
                self.cozmo.move_lift(2.0)  # Positive speed = up
                self.root.after(500, lambda: self.cozmo.move_lift(0))  # Stop after 500ms
                print("Move lift up command sent")
            except AttributeError:
                # Fallback to set_lift_height
                height = self.lift_positions[self.current_lift_index]
                self.cozmo.set_lift_height(height)
                print(f"Set lift height to {height}")
            except Exception as e:
                print(f"Lift command error: {e}")
        
            self.root.after(1000, lambda: setattr(self, 'lift_physically_moving', False))

    def lower_arm(self):
        print(f"lower_arm called - index: {self.current_lift_index}")
    
        if self.cozmo and not self.lift_physically_moving and self.current_lift_index > 0:
            self.lift_physically_moving = True
            self.current_lift_index -= 1
        
            try:
                # Try move_lift with speed control
                self.cozmo.move_lift(-2.0)  # Negative speed = down
                self.root.after(500, lambda: self.cozmo.move_lift(0))  # Stop after 500ms
                print("Move lift down command sent")
            except AttributeError:
                # Fallback to set_lift_height
                height = self.lift_positions[self.current_lift_index]
                self.cozmo.set_lift_height(height)
                print(f"Set lift height to {height}")
            except Exception as e:
                print(f"Lower error: {e}")
        
            self.root.after(1000, lambda: setattr(self, 'lift_physically_moving', False))

    def reset_arm(self):
        if self.cozmo and not self.lift_physically_moving:
            self.lift_physically_moving = True
            self.current_lift_index = 0
        
            try:
                # Move down for longer to ensure we reach bottom
                self.cozmo.move_lift(-2.0)
                self.root.after(1500, lambda: self.cozmo.move_lift(0))
                print("Reset arm command sent")
            except AttributeError:
                self.cozmo.set_lift_height(0.0)
                print("Reset to 0.0")
            except Exception as e:
                print(f"Reset error: {e}")
        
            self.root.after(2000, lambda: setattr(self, 'lift_physically_moving', False))

    def reset_arm(self):
        if self.cozmo and not self.lift_physically_moving:
            self.lift_physically_moving = True
            self.current_lift_index = 0
            self.cozmo.set_lift_height(0.0)
            self.root.after(800, lambda: setattr(self, 'lift_physically_moving', False))

    def display_text(self, text):
        """Display text on Cozmo's face"""
        if self.cozmo:
            try:
                # Create a black background image 
                img = Image.new('RGB', (128, 32), (0, 0, 0))
                draw = ImageDraw.Draw(img)
            
                # Draw text in the eye color
                draw.text((10, 10), text, fill=self.eye_color)
            
                # Convert to binary format (required by PyCozmo)
                img_gray = img.convert('L')
                threshold = 10  # Adjust threshold as needed
                img_binary = img_gray.point(lambda p: p > threshold and 255)
                img_binary = img_binary.convert('1')  # Convert to binary format
            
                # Display on Cozmo
                self.cozmo.display_image(img_binary, duration=2.0)
            
            except Exception as e:
                print(f"Text display error: {e}")
                import traceback
                traceback.print_exc()

    def play_animation(self, anim_name):
        """Play a named animation"""
        if self.cozmo and anim_name in self.animations:
            try:
                self.cozmo.play_animation(self.animations[anim_name])
            except Exception as e:
                print(f"Animation error: {e}") 

    def set_eye_color(self, r, g, b):
        """Set Cozmo's eye color (RGB 0-255)"""
        try:
            print(f"Setting color - R:{r}, G:{g}, B:{b}")  # Debug
            self.eye_color = (int(r), int(g), int(b))
        
            if not self.cozmo:
                print("Cozmo not connected!")
                return

            # Create a proper PIL Image object (128x32 grayscale)
            img = Image.new('L', (128, 32), 0)  # 'L' mode = 8-bit grayscale
            draw = ImageDraw.Draw(img)
        
            # Calculate brightness from RGB
            brightness = int(0.299*r + 0.587*g + 0.114*b)
        
            # Draw eyes
            draw.ellipse((20, 5, 50, 25), fill=brightness)  # Left eye
            draw.ellipse((78, 5, 108, 25), fill=brightness)  # Right eye
        
            # Convert to the exact format PyCozmo expects
            # First ensure we have a proper PIL Image
            if not isinstance(img, Image.Image):
                raise ValueError("Failed to create proper image")
            
            # Then convert to numpy array
            img_array = np.array(img, dtype=np.uint8)
        
            # Verify array shape
            if img_array.shape != (32, 128):
                raise ValueError(f"Incorrect array shape: {img_array.shape}")
        
            # Display on Cozmo
            self.cozmo.display_image(img_array, duration=2.0)
        
        except Exception as e:
            print(f"Eye color error: {str(e)}")
            import traceback
            traceback.print_exc()

    def _verify_image(self, img):
        """Helper method to verify image format"""
        if not isinstance(img, Image.Image):
            return False
        if img.mode != 'L':
            return False
        if img.size != (128, 32):
            return False
        return True

    def start_logging(self, filename="cozmo_pose_log.csv"):
        """Start logging pose data to a file"""
        self.log_file = open(filename, "w")
        self.log_file.write("timestamp,x,y,z,angle,battery\n")
        self.log_start_time = time.time()
        
    def stop_logging(self):
        """Stop logging and close the file"""
        if self.log_file:
            self.log_file.close()
            self.log_file = None

    def _on_robot_state_updated(self, cli, pkt):
        """Handle robot state updates from RobotState packets."""
        try:
            # Update controller state directly from packet attributes
            self.battery_voltage = pkt.battery_voltage
        
            # Pose information
            self.pose_x = pkt.pose_x
            self.pose_y = pkt.pose_y
            self.pose_z = pkt.pose_z
            self.pose_angle = pkt.pose_angle_rad
        
            # Head angle
            self.head_angle = pkt.head_angle_rad
            # Add to path history
            self.path_history.append((self.pose_x, self.pose_y))
            if len(self.path_history) > self.max_path_points:
                self.path_history.pop(0)

            # Log data if logging is active
            if self.log_file:
                timestamp = time.time() - self.log_start_time
                self.log_file.write(
                    f"{timestamp:.3f},{self.pose_x:.2f},{self.pose_y:.2f},"
                    f"{self.pose_z:.2f},{self.pose_angle:.4f},"
                    f"{self.battery_voltage:.2f}\n"
                )            
            
            # Update sensor state
            self.last_sensor_state = type('SensorState', (), {                                
                'cliff_detected': not any(r > 100 for r in pkt.cliff_data_raw)
            })
        except Exception as e:
            print(f"Error processing state packet: {e}")

    def look_up(self):
        if self.cozmo and not self.head_physically_moving and self.current_head_index < len(self.head_positions) - 1:
            self.head_physically_moving = True
            self.current_head_index += 1
            angle = self.head_positions[self.current_head_index]
            self.cozmo.set_head_angle(angle)
            self.head_angle = angle
            self.root.after(800, lambda: setattr(self, 'head_physically_moving', False))

    def look_down(self):
        if self.cozmo and not self.head_physically_moving and self.current_head_index > 0:
            self.head_physically_moving = True
            self.current_head_index -= 1
            angle = self.head_positions[self.current_head_index]
            self.cozmo.set_head_angle(angle)
            self.head_angle = angle
            self.root.after(800, lambda: setattr(self, 'head_physically_moving', False))

    def center_head(self):
        if self.cozmo and not self.head_physically_moving:
            self.head_physically_moving = True
            self.current_head_index = 2
            angle = 0.0
            self.cozmo.set_head_angle(angle)
            self.head_angle = angle
            self.root.after(800, lambda: setattr(self, 'head_physically_moving', False))

    def on_camera_image(self, cli, image):
        self.current_frame = image

    def disconnect(self):
        if self.cozmo:
            self.camera_active = False
            self.cozmo.disconnect()
            self.cozmo = None

    def get_camera_frame(self):
        if self.current_frame is not None:
            frame = np.array(self.current_frame)
            frame = np.clip(frame * 1.5, 0, 255).astype(np.uint8)
            return frame
        return None

    # ---- Movement Controls ----
    def drive_forward(self):
        if self.cozmo:
            self.cozmo.drive_wheels(self.speed, self.speed)

    def drive_backward(self):
        if self.cozmo:
            self.cozmo.drive_wheels(-self.speed, -self.speed)

    def turn_left(self):
        if self.cozmo:
            self.cozmo.drive_wheels(-self.speed, self.speed)

    def turn_right(self):
        if self.cozmo:
            self.cozmo.drive_wheels(self.speed, -self.speed)

    def stop(self):
        if self.cozmo:
            self.cozmo.drive_wheels(0, 0)   

class CozmoGUI:
    def __init__(self):
        self.controller = SimpleCozmoController()
        self.root = tk.Tk()
        self.root.title("Reliable Cozmo Control")
        
        # Store root reference in controller
        self.controller.root = self.root
        
        # Connection panel with sync button
        conn_frame = tk.Frame(self.root)
        conn_frame.pack(pady=5)
        tk.Button(conn_frame, text="Connect", 
                command=self._connect_with_feedback).pack(side=tk.LEFT)
        tk.Button(conn_frame, text="Re-Sync Head", 
                command=self.controller._sync_head_position).pack(side=tk.LEFT)
        tk.Button(conn_frame, text="Re-Sync Arm", 
                command=self.controller._sync_lift_position).pack(side=tk.LEFT)
        tk.Button(conn_frame, text="Disconnect", 
                command=self.controller.disconnect).pack(side=tk.LEFT)
        self.conn_status = tk.Label(conn_frame, text="Disconnected", fg="red")
        self.conn_status.pack(side=tk.LEFT, padx=10)
        
        # Logging controls
        log_frame = tk.Frame(self.root)
        log_frame.pack(pady=5)
        
        tk.Button(log_frame, text="Start Logging", 
                 command=lambda: self.controller.start_logging()).pack(side=tk.LEFT)
        tk.Button(log_frame, text="Stop Logging", 
                 command=self.controller.stop_logging).pack(side=tk.LEFT)
        
        # Camera display with head position overlay
        self.camera_label = tk.Label(self.root)
        self.camera_label.pack()
        
        # Add Animation Controls Frame
        anim_frame = tk.Frame(self.root)
        anim_frame.pack(pady=5)
    
        tk.Label(anim_frame, text="Animations:").pack(side=tk.LEFT)
    
        # Add animation buttons
        anim_buttons = ["Happy", "Sad", "Excited", "Angry", "Dizzy"]
        for anim in anim_buttons:
            tk.Button(anim_frame, text=anim, 
                    command=lambda a=anim: self.controller.play_animation(a)
                    ).pack(side=tk.LEFT, padx=2)

        # Add more animations in a second row
        anim_frame2 = tk.Frame(self.root)
        anim_frame2.pack(pady=5)
    
        anim_buttons2 = ["Win", "Lose", "Spark", "Sing", "Bored"]
        for anim in anim_buttons2:
            tk.Button(anim_frame2, text=anim, 
                    command=lambda a=anim: self.controller.play_animation(a)
                    ).pack(side=tk.LEFT, padx=2)
    
        # Eye Color Controls
        color_frame = tk.Frame(self.root)
        color_frame.pack(pady=5)

        tk.Label(color_frame, text="Eye Color:").pack(side=tk.LEFT)

        color_buttons = [
            ("Red", (255, 0, 0)),
            ("Green", (0, 255, 0)),
            ("Blue", (0, 0, 255)),
            ("Yellow", (255, 255, 0)),
            ("Purple", (255, 0, 255)),
            ("White", (255, 255, 255))
        ]

        for name, color in color_buttons:
            btn = tk.Button(
                color_frame,
                text=name,
                command=lambda c=color: [
                    print(f"Sending color - R:{c[0]}, G:{c[1]}, B:{c[2]}"),  # Debug
                    self.controller.set_eye_color(c[0], c[1], c[2])
                ]
            )
            btn.config(
                bg=f'#{color[0]:02x}{color[1]:02x}{color[2]:02x}',
                activebackground=f'#{color[0]:02x}{color[1]:02x}{color[2]:02x}',
                fg='black' if sum(color) > 384 else 'white'
            )
            btn.pack(side=tk.LEFT, padx=2)
    
        # Add Custom Text Entry
        text_frame = tk.Frame(self.root)
        text_frame.pack(pady=5)
    
        self.face_text = tk.StringVar()
        tk.Entry(text_frame, textvariable=self.face_text, width=15).pack(side=tk.LEFT)
        tk.Button(text_frame, text="Display Text",
                 command=lambda: self.controller.display_text(self.face_text.get())
                 ).pack(side=tk.LEFT, padx=5)
    
        # Add Clear Face button
        tk.Button(text_frame, text="Clear Face",
                 command=lambda: self.controller.set_eye_color(*self.controller.eye_color)
                 ).pack(side=tk.LEFT, padx=5)

        # Head position indicator (visual bar)
        self.head_indicator = tk.Label(self.root, text="[▁ ▂ ▅ ▇ █]", 
                                      font=("Courier", 20))
        self.head_indicator.pack()
        
        # Arm control buttons
        arm_frame = tk.Frame(self.root)
        arm_frame.pack(pady=5)
        
        tk.Label(arm_frame, text="Arm Control:").pack(side=tk.LEFT)
        tk.Button(arm_frame, text="Raise", command=self.controller.raise_arm).pack(side=tk.LEFT, padx=2)
        tk.Button(arm_frame, text="Lower", command=self.controller.lower_arm).pack(side=tk.LEFT, padx=2)
        tk.Button(arm_frame, text="Reset", command=self.controller.reset_arm).pack(side=tk.LEFT, padx=2)
        
        # Arm position indicator
        self.arm_indicator = tk.Label(self.root, text="[▁ ▂ ▃ █]", 
                                     font=("Courier", 20))
        self.arm_indicator.pack()
        
        # Control panel
        ctrl_frame = tk.Frame(self.root)
        ctrl_frame.pack(pady=10)        

        # Speed control
        tk.Label(ctrl_frame, text="Drive Speed:").grid(row=0, column=0)
        self.speed_scale = tk.Scale(ctrl_frame, from_=50, to=500, 
                                  orient=tk.HORIZONTAL, length=200)
        self.speed_scale.set(100)
        self.speed_scale.grid(row=0, column=1)
        self.speed_scale.config(command=self.update_speed)
        
        self.status_frame = tk.Frame(self.root)
        self.status_frame.pack(pady=5)

        self.battery_label = tk.Label(self.status_frame, text="Battery: -- V")
        self.battery_label.pack(side=tk.LEFT, padx=10)

        self.pose_label = tk.Label(self.status_frame, text="Pose: (x=--, y=--, angle=--)")
        self.pose_label.pack(side=tk.LEFT, padx=10)

        # Key bindings
        # Keep arrow keys for sharp turns
        self.root.bind('<Up>', lambda e: self.controller.drive_forward())
        self.root.bind('<Down>', lambda e: self.controller.drive_backward())
        self.root.bind('<Left>', lambda e: self.controller.turn_left())
        self.root.bind('<Right>', lambda e: self.controller.turn_right())
        self.root.bind('<KeyRelease>', lambda e: self.controller.stop())

        # Add A/D for gentle arc turns (good for trailers)
        self.root.bind('<a>', lambda e: self.controller.gentle_turn_left())
        self.root.bind('<d>', lambda e: self.controller.gentle_turn_right())
        self.root.bind('<q>', lambda e: self.controller.reverse_turn_left())
        self.root.bind('<e>', lambda e: self.controller.reverse_turn_right())
        
        # Discrete head controls
        self.root.bind('<Prior>', lambda e: self.move_up())     # Page Up
        self.root.bind('<Next>', lambda e: self.move_down())    # Page Down
        self.root.bind('<Home>', lambda e: self.move_center())  # Home
        
        # Arm control key bindings
        self.root.bind_all('<KeyPress-r>', lambda e: self.controller.raise_arm())  # 'r' for raise
        self.root.bind_all('<KeyPress-l>', lambda e: self.controller.lower_arm())  # 'l' for lower
        self.root.bind_all('<KeyPress-d>', lambda e: self.controller.reset_arm())  # 'd' for down
        
        self.root.bind('<plus>', self.increase_speed)   # '+' key
        self.root.bind('<minus>', self.decrease_speed)  # '-' key

        # Battery indicator
        self.battery_canvas = tk.Canvas(self.status_frame, width=100, height=30)
        self.battery_canvas.pack(side=tk.LEFT, padx=10)
        self.update_battery_display()

        # Start systems
        self.update_camera()
        self.update_head_display()
        self.update_arm_display()
        self.update_status()
        
        # Path display
        self.path_canvas = tk.Canvas(self.root, width=300, height=300, bg="white")
        self.path_canvas.pack()
        self.update_path_display()
        
        sensor_frame = tk.Frame(self.root)
        sensor_frame.pack(pady=5)        
        
        # Cliff sensor
        self.cliff_sensor = tk.Label(sensor_frame, text="Cliff: --", width=10)
        self.cliff_sensor.pack(side=tk.LEFT, padx=5)
        
        self.update_sensor_display()

    def update_arm_display(self):
        """Update the arm position indicator"""
        indicators = ["▁", "▂", "▃", "█"]
        display_text = "[ " + " ".join(
            indicators[i] if i == self.controller.current_lift_index else " " 
            for i in range(len(self.controller.lift_positions))
        ) + " ]"
        self.arm_indicator.config(text=display_text)
        self.root.after(50, self.update_arm_display)

    def update_sensor_display(self):
        """Update sensor status indicators with real data"""
        if self.controller.cozmo and hasattr(self.controller, 'last_sensor_state'):
            state = self.controller.last_sensor_state        
        
            # Cliff sensor (1 = cliff detected, 0 = safe)
            cliff_status = "Cliff!" if state.cliff_detected else "Safe"
            self.cliff_sensor.config(text=f"Cliff: {cliff_status}",
                                   fg="red" if state.cliff_detected else "green")
        else:                        
            self.cliff_sensor.config(text="Cliff: --", fg="gray")
        
        self.root.after(100, self.update_sensor_display)

    def update_path_display(self):
        """Update the movement path visualization"""
        self.path_canvas.delete("all")
        
        if not self.controller.cozmo or len(self.controller.path_history) < 2:
            self.path_canvas.create_text(150, 150, text="No path data")
            self.root.after(500, self.update_path_display)
            return
        
        # Scale and center the path
        all_x = [p[0] for p in self.controller.path_history]
        all_y = [p[1] for p in self.controller.path_history]
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        
        scale_x = 280 / (max_x - min_x + 1) if max_x != min_x else 1
        scale_y = 280 / (max_y - min_y + 1) if max_y != min_y else 1
        scale = min(scale_x, scale_y, 20)  # Limit maximum zoom
        
        # Draw path
        points = []
        for x, y in self.controller.path_history:
            px = 150 + (x - (min_x + max_x)/2) * scale
            py = 150 + (y - (min_y + max_y)/2) * scale
            points.extend([px, py])
        
        self.path_canvas.create_line(*points, fill="blue", width=2)
        
        # Draw current position
        last_x, last_y = self.controller.path_history[-1]
        cx = 150 + (last_x - (min_x + max_x)/2) * scale
        cy = 150 + (last_y - (min_y + max_y)/2) * scale
        self.path_canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill="red")
        
        self.root.after(500, self.update_path_display)

    
    def update_battery_display(self):
        """Update the battery level graphic"""
        self.battery_canvas.delete("all")
        
        # Battery outline
        self.battery_canvas.create_rectangle(5, 5, 80, 25, outline="black", width=2)
        self.battery_canvas.create_rectangle(80, 10, 85, 20, fill="black")
        
        # Battery level
        if not self.controller.cozmo:
            level = 0
            color = "gray"
        else:
            voltage = self.controller.battery_voltage
            level = min(max((voltage - 3.5) / (4.2 - 3.5), 0.0), 1.0)  # Normalize 3.5V-4.2V
            color = "green" if level > 0.3 else "orange" if level > 0.1 else "red"
        
        fill_width = 70 * level
        self.battery_canvas.create_rectangle(10, 10, 10 + fill_width, 20, fill=color)
        
        # Voltage text
        voltage_text = f"{self.controller.battery_voltage:.2f}V" if self.controller.cozmo else "--V"
        self.battery_canvas.create_text(40, 35, text=voltage_text)
        
        self.root.after(1000, self.update_battery_display)  # Update every second

    def update_status(self):
        if self.controller.cozmo:  # connected
            self.battery_label.config(text=f"Battery: {self.controller.battery_voltage:.2f} V")
            self.pose_label.config(
                text=f"Pose: (x={self.controller.pose_x:.1f}, y={self.controller.pose_y:.1f}, angle={math.degrees(self.controller.pose_angle):.1f}°)"
            )
        else:
            self.battery_label.config(text="Battery: -- V")
            self.pose_label.config(text="Pose: (x=--, y=--, angle=--)")
        self.root.after(500, self.update_status)  # update every 500ms

    def increase_speed(self, event=None):
        new_speed = min(self.controller.speed + 10, 500)  # Max 500
        self.controller.speed = new_speed
        self.speed_scale.set(new_speed)
        
    def decrease_speed(self, event=None):
        new_speed = max(self.controller.speed - 10, 50)   # Min 50
        self.controller.speed = new_speed
        self.speed_scale.set(new_speed)

    def _connect_with_feedback(self):
        if self.controller.connect():
            self.conn_status.config(text="Connected", fg="green")
            self.update_head_indicator()  # Initialize display
            self.update_arm_display()    # Initialize arm display
        else:
            self.conn_status.config(text="Failed!", fg="red")
            
    def update_speed(self, val):
        self.controller.speed = float(val)
        
    def update_camera(self):
        if self.controller.camera_active:
            try:
                frame = self.controller.get_camera_frame()
                if frame is not None:
                    img = Image.fromarray(frame).resize((320, 240))
                    
                    # Add head angle overlay
                    draw = ImageDraw.Draw(img)
                    angle_deg = math.degrees(self.controller.head_angle)
                    draw.text((10, 10), f"Head: {angle_deg:.1f}°", fill="red")
                    
                    # Add arm height overlay
                    lift_percent = self.controller.lift_positions[self.controller.current_lift_index] * 100
                    draw.text((10, 30), f"Arm: {lift_percent:.0f}%", fill="red")
                    
                    photo = ImageTk.PhotoImage(image=img)
                    self.camera_label.config(image=photo)
                    self.camera_label.image = photo
            except Exception as e:
                print(f"Camera error: {e}")
                
        self.root.after(100, self.update_camera)
        
    def update_head_display(self):
        """Update both numeric and visual indicators"""
        angle_deg = math.degrees(self.controller.head_angle)
        self.update_head_indicator()
        self.root.after(50, self.update_head_display)
        
    def update_head_indicator(self):
        """Update the position bar display"""
        indicators = ["▁", "▂", "▅", "▇", "█"]
        display_text = "[ " + " ".join(
            indicators[i] if i == self.controller.current_head_index else " " 
            for i in range(len(self.controller.head_positions))
        ) + " ]"
        self.head_indicator.config(text=display_text)
        
    def move_up(self):
        self.controller.look_up()
        self.update_head_indicator()
        
    def move_down(self):
        self.controller.look_down()
        self.update_head_indicator()
        
    def move_center(self):
        self.controller.center_head()
        self.update_head_indicator()
        
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = CozmoGUI()
    app.run()
