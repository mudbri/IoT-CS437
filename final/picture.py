from picamera import PiCamera
import time
import pytesseract
from PIL import Image
import cv2

camera = PiCamera()
camera.resolution = (1280, 720)
#camera.vflip = True
#camera.contrast = 10
time.sleep(1)
for i in range(100):
    camera.capture("img3.jpg")
    time.sleep(1)
    img = cv2.imread('img3.jpg',cv2.IMREAD_COLOR) #Open the image from which charectors has to be recognized
    img = cv2.resize(img, (620,480) ) #resize the image if required

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert to grey to reduce detials
    # gray = cv2.bilateralFilter(gray, 11, 17, 17) #Blur to reduce noise

    original = pytesseract.image_to_string(gray, config='')
    test = (pytesseract.image_to_data(gray) ) #get confidence level if required
    print(pytesseract.image_to_boxes(gray))

    print (original)
    text = pytesseract.image_to_string(img, config='')
    print (text)

print("Done.")
