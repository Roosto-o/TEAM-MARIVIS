def main():
    # --- Configuration ---
    CAMERA_INDEX = 0
    TARGET_OBJECT = 'cat' 
    FRAME_WIDTH, FRAME_HEIGHT = 640, 480
    # ---------------------

    # 1. Initialize Components
    detector = ObjectDetector()
    navigator = RobotNavigator()
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    
    

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # 2. Detect Target Object (e.g., the 'cat')
        detections = detector.detect(frame, TARGET_OBJECT)

        # 3. Decision Making and Action
        if detections:
            # Assume we focus on the first detected target
            target_box = detections[0]['bbox']
            
            # Use navigator to move towards the detected object
            navigator.navigate_to_target(target_box, FRAME_WIDTH, FRAME_HEIGHT)
            
        else:
            print(f"Target '{TARGET_OBJECT}' not found. Searching...")
    # If target is lost, maybe slowly turn to search
            # navigator.turn_in_place()

        # 4. Display (for debugging)
        detector.display_frame(frame, detections)

        # Exit on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    navigator.stop()

if _name_ == "_main_":
    main()

