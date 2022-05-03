import apriltag
import cv2

cap = cv2.VideoCapture(0)
at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11  tag25h9'))

while(1):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray)
    
    for tag in tags:
        
        print(tag.tag_id)
        mx = int((tag.corners[0].astype(int)[0] + tag.corners[2].astype(int)[0]) / 2)
        my = int((tag.corners[0].astype(int)[1] + tag.corners[2].astype(int)[1]) / 2)

        cv2.line(frame, (mx - 10, my), (mx + 10, my), (0, 255, 0), 2)
        cv2.line(frame, (mx, my - 10), (mx, my + 10), (0, 255, 0), 2)
        cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2)  # left-top
        cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2)  # right-top
        cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2)  # right-bottom
        cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2)  # left-bottom
    
    cv2.imshow('capture', frame)
    k = cv2.waitKey(1)
    if k == 27:
        break
