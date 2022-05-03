import cv2

cap = cv2.VideoCapture(0)

while(1):        
        ret, img = cap.read()
        mx=250
        my=180
        cv2.rectangle(img, (int(mx) - 10, int(my) - 10), (int(mx + 10), int(my + 10)), (255, 0, 0), 1)
        cv2.imshow('capture', img)
        k=cv2.waitKey(1)
        if k==27:
            break