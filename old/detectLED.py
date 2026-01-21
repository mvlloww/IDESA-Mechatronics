import cv2
import numpy as np

def detect_red_led(frame, lower_red1, upper_red1, lower_red2, upper_red2):
    # Convert BGR to HSV for better color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create masks for red color
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Check if red regions detected
    if contours:
        print("FIRE HAS STARTED!")
        return True, mask
    return False, mask

# Main loop - change video source as needed (either 0 or 1)
cap = cv2.VideoCapture(0)

# Create window and trackbars
cv2.namedWindow('Red LED Detection')
cv2.createTrackbar('Lower H1', 'Red LED Detection', 0, 180, lambda x: None)
cv2.createTrackbar('Upper H1', 'Red LED Detection', 10, 180, lambda x: None)
cv2.createTrackbar('Lower S', 'Red LED Detection', 100, 255, lambda x: None)
cv2.createTrackbar('Upper S', 'Red LED Detection', 255, 255, lambda x: None)
cv2.createTrackbar('Lower V', 'Red LED Detection', 100, 255, lambda x: None)
cv2.createTrackbar('Upper V', 'Red LED Detection', 255, 255, lambda x: None)
cv2.createTrackbar('Lower H2', 'Red LED Detection', 170, 180, lambda x: None)
cv2.createTrackbar('Upper H2', 'Red LED Detection', 180, 180, lambda x: None)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Get trackbar values
    lower_h1 = cv2.getTrackbarPos('Lower H1', 'Red LED Detection')
    upper_h1 = cv2.getTrackbarPos('Upper H1', 'Red LED Detection')
    lower_s = cv2.getTrackbarPos('Lower S', 'Red LED Detection')
    upper_s = cv2.getTrackbarPos('Upper S', 'Red LED Detection')
    lower_v = cv2.getTrackbarPos('Lower V', 'Red LED Detection')
    upper_v = cv2.getTrackbarPos('Upper V', 'Red LED Detection')
    lower_h2 = cv2.getTrackbarPos('Lower H2', 'Red LED Detection')
    upper_h2 = cv2.getTrackbarPos('Upper H2', 'Red LED Detection')
    
    lower_red1 = np.array([lower_h1, lower_s, lower_v])
    upper_red1 = np.array([upper_h1, upper_s, upper_v])
    lower_red2 = np.array([lower_h2, lower_s, lower_v])
    upper_red2 = np.array([upper_h2, upper_s, upper_v])
    
    detected, mask = detect_red_led(frame, lower_red1, upper_red1, lower_red2, upper_red2)
    
    cv2.imshow('Red LED Detection', mask)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()