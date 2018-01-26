import cv2

cam = cv2.VideoCapture(1)
captured_mode = False
while True:
    if captured_mode == False:
        _, image = cam.read()
    cv2.imshow('Camera Feed',image)
    userEntry = cv2.waitKey(1) & 0xFF
    if (userEntry == ord('q') or userEntry == 27):
        break
    elif (userEntry == ord('c')):
        captured_mode = not captured_mode

cam.release()
cv2.destroyAllWindows()