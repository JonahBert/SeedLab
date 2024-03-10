#Demo 1: get images of chess board for camera calibration, store in folder
import cv2

# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)

#set dimensions
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

#establish folder path
folderName = '/home/seedlab/Demo1/CalibrationImgs/'

#take 10 images for test patterns
for i in range(0,10):

    #wait for key, then take picture
    k = cv2.waitKey(0) & 0xFF
    if k == ord("q"):
        break
    print('Picture taken!')
    ret, image = camera.read()

    #convert image to greyscale
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fileName = folderName + 'Image' + str(i) + '.jpg'

    #save image, throw execption if image could not be saved
    try:
        cv2.imwrite(fileName, image)
    except:
        print("Could not save " + fileName)
        pass