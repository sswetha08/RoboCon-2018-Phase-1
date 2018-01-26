# This program thresholds the RED values of an image. Keep sure you have G and B thresholding, otherwise we won't be able to see it.

import cv2

threshold_values = {
    'R':127,
    'G':100,
    'B':100
}

cam = cv2.VideoCapture(0)

while True:
    _ , image = cam.read()
    image = cv2.flip(image ,1)
    _ ,mask_red = cv2.threshold(image[:,:,2], threshold_values['R'], 255, cv2.THRESH_BINARY)
    _ , mask_blue = cv2.threshold(image[:,:,0], threshold_values['G'], 255, cv2.THRESH_BINARY_INV)
    _, mask_green = cv2.threshold(image[:, :, 1], threshold_values['B'], 255, cv2.THRESH_BINARY_INV)
    mask = cv2.bitwise_and(mask_blue,mask_green)
    mask = cv2.bitwise_and(mask,mask_red)
    # print(image.shape)
    cv2.imshow("Mask Filter", mask)
    cv2.imshow("Feed", image)
    filteredImage = cv2.bitwise_and(image,image,mask=mask)
    cv2.imshow("Red Object Tracker", filteredImage)
    userKeyEntry = cv2.waitKey(1) & 0xFF
    if userKeyEntry == ord('q') or userKeyEntry == 27:
        break

cam.release()
cv2.destroyAllWindows()