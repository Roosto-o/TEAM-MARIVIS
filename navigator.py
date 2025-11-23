class RobotNavigator:
    def _init_(self):
        """Initializes the motor control hardware interface."""
        print("Initializing Robot Navigator (Motor Interface)...")
        # In a real robot: self.motor = MotorDriver()

    def move_forward(self, speed=40):
        print(f"ACTION: Moving forward at speed {speed}")    
 # self.motor.set_velocity(speed)

    def stop(self):
        print("ACTION: Stopping all movement.")
        # self.motor.set_velocity(0)

    def turn_to_center(self, target_center_x, frame_center_x):
        """Calculates turn direction based on target position."""
        error = target_center_x - frame_center_x
        
        # Threshold to avoid continuous minor adjustments
        if abs(error) > 50: 
            if error > 0:
                print("ACTION: Turning Right")
                # self.motor.turn_right(power=abs(error) * 0.1) 
            else:
                print("ACTION: Turning Left")
                # self.motor.turn_left(power=abs(error) * 0.1) 
            return True
        return False

    def navigate_to_target(self, bbox, frame_width=640, frame_height=480):
        """Controls movement based on the target object's bounding box."""
        x1, y1, x2, y2 = bbox
        target_center_x = (x1 + x2) // 2
        
        is_turning = self.turn_to_center(target_center_x, frame_width // 2)
        
        # Simple distance check based on object size
        box_area = (x2 - x1) * (y2 - y1)
        
        if box_area > (frame_width * frame_height * 0.3):
            # Target is large/close
            self.stop()
        elif not is_turning:
            # Target centered, move forward
            self.move_forward(speed=30)
            
        time.sleep(0.05) 
