from skimage.morphology import remove_small_holes, remove_small_objects, binary_closing, binary_opening, disk, binary_erosion
from skimage.measure import label, regionprops, regionprops_table
from scipy.ndimage import gaussian_filter
import pandas as pd
import numpy as np
import time 
import cv2 as cv
from scam import cam
#from mqtt-client import imageAnalysis

from uservice import service

def imageAnalysis(save):
  if cam.useCam:
    ok, img, imgTime = cam.getImage()
    if not ok: # size(img) == 0):
      if cam.imageFailCnt < 5:
        print("% Failed to get image.")
    else:
      h, w, ch = img.shape
      if not service.args.silent:
        # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
        pass
      #edge.paint(img)
      if save:
        fn = f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
        cv.imwrite(fn, img)
        if not service.args.silent:
          print(f"% Saved image {fn}")
      pass
    pass
  pass
  return img

# detection of the balls in the picture of a certain color
def ball(image, color):
    # color thresholds element 0 = blue, 1 = red
    # Blue
    b_low = [120,0]
    b_high = [255, 30]

    # Green
    g_low = [60,0]
    g_high = [255,100]

    # Red
    r_low = [10,80]

    mask = np.zeros_like(image[:,:,0], dtype=bool) # create a mask with the same size as the image
    # color of the thresholds (for images in BGR)
    if (color == 0): #blue
        mask = (
            (image[:,:,0] > image[:,:,1]) & (image[:,:,0] > image[:,:,2]) & # blue intensity is higher than green and red
            (image[:,:,0] >= b_low[color]) & # blue 
            (image[:,:,1] >= g_low[color]) & # green
            (image[:,:,2] >= r_low[color]) & # red
            ~((image[:, :, 0] >= 253) & (image[:, :, 1] >= 253) & (image[:, :, 2] >= 253)) # color is not white
        )
    elif (color == 1): #orange
        mask = (
            (image[:,:,2] > image[:,:,0]) & (image[:,:,2] > image[:,:,1]) & # red intensity is higher than green and blue
            ((image[:,:,1] + 10) > image[:,:,0]) & # green intensity is higher than blue
            (image[:,:,0] >= b_low[color]) & (image[:,:,0] <= b_high[color]) & # blue 
            (image[:,:,1] >= g_low[color]) & (image[:,:,1] <= g_high[color]) & # green
            (image[:,:,2] >= r_low[color]) & # red
            ~((image[:, :, 0] >= 253) & (image[:, :, 1] >= 253) & (image[:, :, 2] >= 253)) # color is not white
        )
    
    # clean up the picture   
    mask = remove_small_holes(mask, 500)
    mask = binary_closing(mask, disk(5))
    mask = remove_small_objects(mask, 500)
    mask = binary_opening(mask, disk(10))
    

    #prevent detecting the arm or the background
    mask[:200,:] = 0      # remove the upper part of the picture
    mask[:, :70] = 0      # Left side (0 to 70 pixels)
    mask[:, 750:] = 0     # Right side (750 to 820 pixels)
    

    # find the middle of the ball from the up left corner
    labeled_image, n_labels = label(mask, background=0,return_num=True,connectivity=1)
    regions = regionprops(labeled_image)

    # create a table with the properties of the regions
    # centroid = (y,x) = (row, column)
    region_table = regionprops_table(labeled_image, properties=['centroid', 'axis_major_length']) 
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
    range = 5
    status = 99
    wait = 0.0
    e = abs(xy[0] - middle_x)

    if(xy[0] > middle_x + range):
        #then turn left
        service.send(service.topicCmd + "ti/rc","0.05 -0.30")
        status = 1
    elif(xy[0] < middle_x - range):
        #then turn right
        service.send(service.topicCmd + "ti/rc","0.05 0.30")
        status = 2
    else:
        #ball is in the middle
        status = 0
        service.send(service.topicCmd + "ti/rc","0 0")

    wait = (e/middle_x)*0.6+0.1
    #stop to update the picture and the ball detection
    time.sleep(wait)
    service.send(service.topicCmd + "ti/rc", "0 0")

    return status

# calculate the distance to the ball
def distance_calc(xy, width, type):
    # type = 0 for oval balls, 1 for others (golf ball, holes, etc.)

    distance = 0.0
    
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

    if xy != []:
        distance_xy = (a2*xy[1]**2 + b2*xy[1] + c2)*10 #in mm

    # if the difference between the two distances is small, use the average
    if (type == 0) &(abs(distance_xy - distance_width)  < 5):
        distance = (distance_xy + distance_width)/2
        status = 1
    elif (type == 1):
        distance = distance_xy
        status = 1
    else:
        print("Distance xy: ", distance_xy, "Distance width: ", distance_width)
        distance = distance_xy
        status = 0

    return distance, status

# move to the ball
def move_straight(xy, width, type):
    arm_length = 300 #in mm
    distance = 1.0
    status = 99
    wait = 0.0
    velocity = 0.0
    middle_x = 410

    if (xy != []):
        if (abs(xy[0] - middle_x) > 10):
            #adjust the position of the ball in the middle of the picture
            move_middle(xy)
        else:
            #calculate the distance to the ball
            distance, status_d = distance_calc(xy, width, type)
            distance = distance - arm_length #in mm
    else:
        img = imageAnalysis(False)
        xy, status_b, width = ball(img, 0)

        if xy != []:
            distance, status_d = distance_calc(xy, width)
            distance = distance - arm_length #in mm
        else:
           distance = 0
            
    print("Distance: ", distance)

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

    if xy != []:
       state = 1
    
    width = regions[0].axis_major_length
    
    return xy, state, width # gives back a tuple with the pixel position of the (roughly) middle of the hole 

# drive to the ball
def drive2ball(xy, stat_ball, width, state_ia, type):
    stat_middle = -1
    stat_straight = -1

    if state_ia == 0: # find a ball

        #starting position Servo
        service.send(service.topicCmd + "T0/servo","1 -900 200")

        if (stat_ball >= 1) & (xy != (0,0)): # found one or more balls
          print("Found one or more balls. Nearest ball:", xy)
          stat_middle = move_middle(xy)

          if stat_middle == 0:
          # ball is in the middle
            state_ia = 1

        else: # no ball found -> turn
          print("No ball found. Turn to find the ball.")
          service.send(service.topicCmd + "ti/rc","0.05 -0.25")
          time.sleep(0.1)
          service.send(service.topicCmd + "ti/rc", "0 0")

    elif state_ia == 1: # ball in the middle
        print("Ball in the middle. Move straight.")
        stat_straight = move_straight(xy, width, type)
        
        if stat_straight == 0:
          # ball is captured
          service.send(service.topicCmd + "T0/servo", "1 -150 200")
          state_ia = 2
          print("Ball is picked up.")

    return state_ia