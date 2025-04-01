from skimage.morphology import remove_small_holes, remove_small_objects, binary_closing, binary_opening, disk, binary_erosion
from skimage.measure import label, regionprops, regionprops_table
from scipy.ndimage import gaussian_filter
import pandas as pd
import numpy as np
import time 

from uservice import service

# detection of the balls in the picture of a certain color
def ball(image, color):
    # color thresholds element 0 = blue, 1 = red
    # Blue
    b_low = [120,0]

    # Green
    g_low = [60,0]

    # Red
    r_low = [10,150]

    # color of the thresholds (for images in BGR)
    if (color == 0):
        mask = (
            (image[:,:,0] > image[:,:,1]) & (image[:,:,0] > image[:,:,2]) & # blue intensity is higher than green and red
            (image[:,:,0] >= b_low[color]) & # blue 
            (image[:,:,1] >= g_low[color]) & # green
            (image[:,:,2] >= r_low[color]) & # red
            ~((image[:, :, 0] >= 253) & (image[:, :, 1] >= 253) & (image[:, :, 2] >= 253)) # color is not whit
        )
    #elif (color == 1): 
    
    # clean up the picture   
    mask = remove_small_holes(mask, 500)
    mask = remove_small_objects(mask, 500)
    mask = binary_opening(mask, disk(10))
    mask = binary_closing(mask, disk(5))
    mask[:200,:] = 0 # remove the upper part of the picture
    

    # find the middle of the ball from the up left corner
    labeled_image, n_labels = label(mask, background=0,return_num=True,connectivity=1)
    regions = regionprops(labeled_image)

    # create a table with the properties of the regions
    # centroid = (y,x) = (row, column)
    region_table = regionprops_table(labeled_image, properties=['centroid', 'area', 'axis_major_length']) 
    pd_regions = pd.DataFrame(region_table)
    pd_regions = pd_regions.sort_values(by='centroid-0', ascending=False) # sort by y coordinate

    status = 99
    xy = []
    width = 0

    if (len(pd_regions) == 1):
        xy = tuple(map(int, regions[0].centroid[::-1])) 
        width = pd_regions.iloc[0]['axis_major_length']
        status = 1

    elif(pd_regions.empty):
        status = 0

    else:
        xy = (int(pd_regions.iloc[0]['centroid-1']), int(pd_regions.iloc[0]['centroid-0']))
        width = pd_regions.iloc[0]['axis_major_length']
        status = 2

    # gives back a tuple with the pixel position of the (roughly) middle of the ball and the result
    return xy, status, width 

# drive the robot so that the object is in the middle of the picture
def move_middle(xy):
    #the whole image is of the size 616x820x3
    middle_x = 410
    range = 10
    status = 99
    wait = 0.0
    e = abs(xy[0] - middle_x)

    if(xy[0] > middle_x + range):
        #then turn left
        service.send(service.topicCmd + "ti/rc","0.05 -0.25")
        status = 1
    elif(xy[0] < middle_x - range):
        #then turn right
        service.send(service.topicCmd + "ti/rc","0.05 0.25")
        status = 2
    else:
        #ball is in the middle
        status = 0
        service.send(service.topicCmd + "ti/rc","0 0")

    wait = (e/middle_x)*0.6+0.05
    #stop to update the picture and the ball detection
    time.sleep(wait)
    service.send(service.topicCmd + "ti/rc", "0 0")

    return status

# calculate the distance to the ball
def distance(xy, width):
    # calculate the distance to the ball by the measurement of the width of the ball
    a = 0.007494
    b = -1.955569
    c = 160.717271
    #calibrated for distances between 30 and 85 cm

    distance_width = (a*width**2 + b*width + c)*10 #in mm

    #calculate the distance to the ball by the coordinates of the ball
    a2 = 0.000668
    b2 = -0.825487
    c2 = 288.263289

    distance_xy = (a2*xy[1]**2 + b2*xy[1] + c2)*10 #in mm

    # if the difference between the two distances is small, use the average
    if (abs(distance_xy - distance_width)  < 5):
        distance = (distance_xy + distance_width)/2
        status = 1
    else:
        print("Distance calculation error")
        print("Distance xy: ", distance_xy, "Distance width: ", distance_width)
        distance = distance_xy
        status = 0

    return distance, status

# move to the ball
def move_straight(xy, width):
    arm_length = 300 #in mm
    distance = 0.0
    status = -1
    wait = 0.0
    velocity = 0.0
    middle_x = 410

    if (xy != []):
        if abs(xy[0] - middle_x) > 10:
            #wrong ball
            xy = []
        else:
            #calculate the distance to the ball
            distance, status_d = distance(xy, width)
            distance = distance - arm_length #in mm

    if (distance > 500.0): #out of calibration range
        velocity = 0.1 #in m/s
        wait = (distance-500.0)/1000/velocity

        service.send(service.topicCmd + "ti/rc", f"{velocity:.2f} 0")
        time.sleep(wait)
        service.send(service.topicCmd + "ti/rc", "0 0")
        status = 3
    elif (distance > 250.0):
        velocity = 0.1 #in m/s
        wait = (distance-250.0)/1000/velocity

        service.send(service.topicCmd + "ti/rc", f"{velocity:.2f} 0")
        time.sleep(wait)
        service.send(service.topicCmd + "ti/rc", "0 0")
        status = 2
    elif (distance > 0.0):
        velocity = 0.1 #in m/s
        wait = distance/1000/velocity

        service.send(service.topicCmd + "ti/rc", f"{velocity:.2f} 0")
        time.sleep(wait)
        service.send(service.topicCmd + "ti/rc", "0 0")
        status = 1
    else:
        # stop
        service.send(service.topicCmd + "ti/rc", "0 0")
        # lowering the arm
        service.send(service.topicCmd + "T0/servo", "1 -80 200")
        status = 0

    return status


# detect holes on black surface in the pictures
def hole(image):
    # calculate the differences between the main color values in the picture with some blurring
    d_B = gaussian_filter(image, sigma=1, order=(0,0,1))
    
    # mark the areas below a threshold
    mask = np.zeros_like(d_B)
    mask = d_B[:,:,2] < 40
    mask_cl = remove_small_objects(mask, min_size=1000, connectivity=1)

    #mark the areas, which are black in the picture and fill in the spaces in between
    black = (image[:,:,0] > 60) & (image[:,:,0] < 110) & (image[:,:,1] > 60) & (image[:,:,1] < 120) & (image[:,:,2] > 55) & (image[:,:,2] < 110)
    black_cl = binary_erosion(black, footprint=[(np.ones((20, 1)), 1), (np.ones((1, 20)), 1)])
    black_cl = remove_small_holes(black_cl, 10000, 1)
    
    # find the space on the black surface, which has changes in color -> hole
    hole = (black_cl == 1) & (mask_cl == 1)

    # calculate the middle
    labeled_image, n_labels = label(hole, background=0,return_num=True,connectivity=2)
    regions = regionprops(label_image=labeled_image)
    xy = regions[0].centroid    
    
    return xy # gives back a tuple with the pixel position of the (roughly) middle of the hole 
