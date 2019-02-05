import cv2
import numpy as np

def region_of_interest(img_file):
    img = cv2.imread(img_file)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
    ## Gen lower mask (0-5) and upper mask (175-180) of RED
    mask1 = cv2.inRange(img_hsv, (0,50,20), (5,255,255))
    mask2 = cv2.inRange(img_hsv, (175,50,20), (180,255,255))

    ## Merge the mask and crop the red regions
    mask = cv2.bitwise_or(mask1, mask2)
    
    print(mask.sum() )
    # red light sum:  798660
    # green light sum: 13515
    if mask.sum() > 100000:
        print " red"
    else:
        print " not red"
    

region_of_interest('wrong/testimage1.png')
region_of_interest('wrong/testimage2.png')
region_of_interest('wrong/testimage3.png')