#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from scipy.ndimage.filters import convolve, gaussian_filter
from scipy.misc import imread, imshow

def detect_edges(image,blur = 1, highThreshold = 91, lowThreshold = 31):

    image_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    # converted the image to float value in order to avoid the clipping values

    image_float=np.array(image_gray, dtype=float) 
    '''
     Temporary variable for storing the row and column of the image

    '''
    row=image_float.shape[0]
    col=image_float.shape[1]

    '''
        Applying the Gaussian filter - to reduce the noice in the image
    '''
    image_gau = gaussian_filter(image_float, blur)

    '''
        Applying the Sobel operator and identifying the Gradient direction 
    '''

    #Applying the sobel operator to get the horizontal and the vertical gradients properly
    image_horizon = convolve(image_gau,[[-1,0,1],[-2,0,2],[-1,0,1]]) 
    image_vertical = convolve(image_gau,[[1,2,1],[0,0,0],[-1,-2,-1]])
    #Get gradient and direction using the horizonal and the vertical gradient 
    image_grad = np.power(np.power(image_horizon, 2.0) + np.power(image_horizon, 2.0), 0.5)
    theta = np.arctan2(image_vertical, image_horizon)
    thetaQ = (np.round(theta * (5.0 / np.pi)) + 5) % 5 #Quantize direction

    '''
        Applying the Non-Maximum Suppression -Suppressing the pixels that are having the non-maximum values to the 
                                              pixels that are having the maximum values i.e. the  upper threshold value 
    '''

    image_gradSup = image_grad.copy()
    for r in range(row):
        for c in range(col):
            if r == 0 or r == row-1 or c == 0 or c == col - 1:
                image_gradSup[r, c] = 0
                continue
            tq = thetaQ[r, c] % 4
            if tq == 0: #0 is E-W (horizontal)
                if image_grad[r, c] <= image_grad[r, c-1] or image_grad[r, c] <= image_grad[r, c+1]:
                    image_gradSup[r, c] = 0
                if tq == 1: #1 is NE-SW
                    if image_grad[r, c] <= image_grad[r-1, c+1] or image_grad[r, c] <= image_grad[r+1, c-1]:
                        image_gradSup[r, c] = 0
                if tq == 2: #2 is N-S (vertical)
                    if image_grad[r, c] <= image_grad[r-1, c] or image_grad[r, c] <= image_grad[r+1, c]:
                        image_gradSup[r, c] = 0
                if tq == 3: #3 is NW-SE
                    if image_grad[r, c] <= image_grad[r-1, c-1] or image_grad[r, c] <= image_grad[r+1, c+1]:
                        image_gradSup[r, c] = 0
            
    
    # Having the double threshold check for the suppression
    image_strong_edge = (image_gradSup > highThreshold)

    #Strong has value 2, weak has value 1
    image_thresholded_edges = np.array(image_strong_edge, dtype=np.uint8) + (image_gradSup > lowThreshold)
    
    '''
        Edge tracing using the concept of the Hysteresis
    '''
    
    #Find weak edge pixels near strong edge pixels
    image_final_Edges = image_strong_edge.copy()
    current_pixels = []
    for r in range(1, row-1):
        for c in range(1, col-1):
            if image_thresholded_edges[r, c] != 1:
                continue # these are not weak pixels
            # Here we are applying the 3*3 patch
            local_patch = image_thresholded_edges[r-1:r+2,c-1:c+2]
            patchMax = local_patch.max() #extracting the max value in the patch

            if patchMax == 2:
                current_pixels.append((r, c))
                image_final_Edges[r, c] = 1

    #Using the current pixel concept, extending the strong pixel
    while len(current_pixels) > 0:
        new_pixel_value = []
        for r, c in current_pixels:
            for dr in range(-1, 2):
                for dc in range(-1, 2):
                    if dr == 0 and dc == 0: 
                        continue
                    r2 =r+dr
                    c2= c+dc
                    if image_thresholded_edges[r2, c2] == 1 and image_final_Edges[r2, c2] == 0:
                        new_pixel_value.append((r2, c2)) # Pasting the weak pixel to the final result
                        image_final_Edges[r2, c2] = 1
        
        current_pixels = new_pixel_value
    imshow(image_final_Edges)
    #print(image_final_Edges)

def canny_webcam():
    "Live capture frames from webcam and show the canny edge image of the captured frames."

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()  # ret gets a boolean value. True if reading is successful (I think). frame is an
        cv2.imshow('Original',frame)
        #detect_edges(frame) -- Try implementing all the algorithm used in canny algorithm
        hsv_format = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_threshold = np.array([30,150,50]) 
        upper_threshold = np.array([255,255,180])

        mask = cv2.inRange(hsv_format, lower_threshold, upper_threshold) 
        res = cv2.bitwise_and(frame,frame, mask= mask) 
        edges = cv2.Canny(frame,100,200) 
        cv2.imshow('canny',edges)

        if cv2.waitKey(20) == ord('q'):  # Introduce 20 milisecond delay. press q to exit.
            break
    cap.release()

if __name__=="__main__":
    rospy.init_node('cannyEdge', anonymous=True) 
    try:
        canny_webcam()
    except rospy.ROSInterruptException:
        pass