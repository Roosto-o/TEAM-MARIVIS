# TEAM-MARIVIS
SAUVC 2026 AUV: Vision-Based Autonomous Navigation

Project Overview

This repository contains the conceptual design, software stack specifications, and core navigation logic for an Autonomous Underwater Vehicle (AUV) designed to compete in the Singapore Autonomous Underwater Vehicle Challenge (SAUVC) 2026.

The primary focus of this initial development phase is successfully completing the Navigation Task, which involves passing through a submerged gate while strictly avoiding an orange flare.

Technical Approach

The AUV utilizes an integrated Computer Vision (CV) system for target identification and navigation:

ML-Based Detection (YOLO/EfficientDet): Used for robust and primary detection of large, structured targets (like the Gate and Drums) even in poor visibility.

Traditional CV (HSV Color Masking): Used for rapid and precise identification of the specific marker colors, crucial for locating and avoiding the mandatory Orange Flare.

Navigation Strategy (First Round)

Condition

Action

Gate Found, Flare Absent

AUV aligns using the Gate's central coordinates ($C_G$) and moves forward using a PID controller.

Gate Found, Flare in Path

Priority Shift: Lateral evasive maneuver away from the Flare's center ($C_F$), followed by re-alignment to the Gate.

No Target Found

Initiate a slow, controlled search pattern (panning).

Software Stack and Environment Setup (Conda)

To ensure consistency across development environments (local machines and the on-board Jetson computer), we use Conda for package management.

Prerequisites

Anaconda or Miniconda installed on your system.

Environment Setup

Use the provided conda_environment.yml file to create and activate the project environment:

# 1. Create the environment using the YAML file
conda env create -f conda_environment.yml

# 2. Activate the new environment
conda activate sauvc-2026-auv


conda_environment.yml

This file specifies all necessary dependencies:

name: sauvc-2026-auv
channels:
  - defaults
  - conda-forge
  - pytorch
dependencies:
  - python=3.10
  - pip
  - numpy=1.24
  - pandas
  - scipy
  - matplotlib
  - jupyter
  # Core Computer Vision Libraries
  - opencv=4.6  # OpenCV for image processing (CV)
  # Deep Learning/ML Frameworks (Choose one based on deployment target)
  # - pytorch::pytorch=2.0.1
  # - pytorch::torchvision
  - tensorflow=2.12  # TensorFlow for YOLO/EfficientDet implementation
  # Robotics and Communication (Placeholder)
  # - ros-core # If using ROS integration (complex dependency)
  - pyserial # For communicating with thruster controllers (if applicable)
  - pip:
    # Additional required Python packages
    - imutils  # Handy image utility functions
    - scikit-learn
    - Pillow
    # Placeholder for specific YOLO framework dependencies (e.g., ultralytics/yolov8)
    # - ultralytics


Conceptual Code: navigation_logic.py

The core logic demonstrates how the AUV processes vision data (simulated here) and issues directional commands, prioritizing flare avoidance.

# AUV Vision Navigation Logic - Conceptual Python Code
import cv2
import numpy as np
# from auv_control import ThrusterController # Placeholder for motor control
# from yolo_model import load_yolo, detect_objects # Placeholder for ML library

# --- System & Model Initialization ---
# model = load_yolo("yolo_sauvc_gate.weights")
# camera = cv2.VideoCapture(0) # Assuming camera index 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTER_X = FRAME_WIDTH // 2
PIXEL_TOLERANCE = 20 # Allowed pixel error before correction

def detect_gate_and_flare(frame):
    """
    Uses both ML (for gate) and CV (for flare) to find targets.
    """
    # 1. ML-Object Detection: (Gate detection using YOLO)
    # yolo_detections = detect_objects(frame, model)
    
    # Placeholder for ML result: Assume gate is roughly in the center
    gate_detected = True 
    gate_center = (CENTER_X, FRAME_HEIGHT // 2)

    # 2. HSV Color Detection: (Orange Flare detection)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define orange color range for the flare (Adjust for underwater conditions)
    # Example HSV range for Orange
    lower_orange = np.array([5, 150, 150])
    upper_orange = np.array([20, 255, 255])
    
    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
    
    # Find contours (potential flare objects)
    contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    flare_detected = False
    flare_center = None
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 500: # Filter small noise
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                flare_center = (cX, cY)
                flare_detected = True

    return gate_detected, gate_center, flare_detected, flare_center

def control_thrusters(direction, power):
    """
    Sends commands to the motors (Placeholder function).
    Direction: "FORWARD", "LEFT", "RIGHT", "PAN_SEARCH"
    """
    print(f"Thruster Command: Direction={direction}, Power={power}")
    # Actual AUV communication code goes here (e.g., ThrusterController.send_command(direction, power))

def navigate_auv(gate_center, flare_center):
    """
    Navigates the AUV based on the detected targets.
    """
    target_x = gate_center[0]
    error_x = target_x - CENTER_X

    # --- 1. Flare Avoidance Logic ---
    if flare_center:
        # Check if the flare is close to the center and a threat
        if abs(flare_center[0] - CENTER_X) < 150: 
            print("WARNING: Orange Flare Detected! Initiating Avoidance Maneuver.")
            # Move away from the flare's position
            if flare_center[0] < CENTER_X:
                control_thrusters("RIGHT", 50) # Shift right
            else:
                control_thrusters("LEFT", 50) # Shift left
            return # Exit navigation logic to prioritize avoidance

    # --- 2. Gate Alignment Logic ---
    if abs(error_x) > PIXEL_TOLERANCE:
        # Adjust lateral position
        if error_x > 0:
            control_thrusters("RIGHT", abs(error_x) * 0.1) # Proportional control
        else:
            control_thrusters("LEFT", abs(error_x) * 0.1)
    else:
        # Aligned, move forward
        control_thrusters("FORWARD", 60)

def main_loop():
    # In a real environment, this loop reads from the camera
    # while True:
    #     ret, frame = camera.read()
    #     if not ret: break

    # Using a placeholder frame for simulation output
    frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8) 
    frame[:, :, 2] = 100 # Simulate blue underwater background

    gate_detected, gate_center, flare_detected, flare_center = detect_gate_and_flare(frame)

    if gate_detected:
        print(f"Gate Center: {gate_center}")
        if flare_detected:
            print(f"Flare Center: {flare_center}")
        
        navigate_auv(gate_center, flare_center)
    else:
        # If gate is not detected, search for it
        control_thrusters("PAN_SEARCH", 10) 

    print("--- End of Vision Cycle ---")

# Execute simulation
main_loop()
print("\nAUV Navigation System Initialized. Starting Main Loop Simulation (Check terminal output for thruster commands).")
