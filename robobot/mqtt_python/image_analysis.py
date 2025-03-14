from skimage.morphology import binary_erosion
from skimage.morphology import remove_small_holes
from skimage.morphology import remove_small_objects
from skimage.measure import label
from skimage.measure import regionprops
from scipy.ndimage import gaussian_filter
import numpy as np

from uservice import service

# detection of the balls in the picture of a certain color
def ball(image, color):
    # color thresholds element 0 = blue, 1 = red
    # Blue
    b_high = [255,80]
    b_low = [200,0]

    # Green
    g_high = [245,20]
    g_low = [160,0]

    # Red
    r_high = [170,255]
    r_low = [20,150]

    # color of the thresholds (for images in BGR)
    image_ball = (image[:,:,0] >= b_low[color]) & (image[:,:,0] <= b_high[color]) & (image[:,:,1] >= g_low[color]) & (image[:,:,1] <= g_high[color]) & (image[:,:,2] >= r_low) & (image[:,:,2] <= r_high)
    
    # clean up the picture
    image_ball_cl = remove_small_holes(image_ball, 10000, 1)
    mask = remove_small_objects(image_ball_cl, min_size=500, connectivity=1)

    # find the middle of the ball from the up left corner
    labeled_image, n_labels = label(mask, background=0,return_num=True,connectivity=2)
    regions = regionprops(label_image=labeled_image)

    status = 99
    xy = [0,0]

    if (n_labels == 1):
        xy = regions[0].centroid
        status = 0
    elif(n_labels == 0):
        print('No ball found')
        status = 1
    else:
        print('More than one ball found')
        status = 2

    return xy, status # gives back a tuple with the pixel position of the (roughly) middle of the ball and the result

# drive the robot so that the object is in the middle of the picture
def move_middle(xy):
    #the whole image is of the size 616x820x3
    middle_x = 410
    range = 5
    status = 99

    if(xy[0] > middle_x + range):
        #then turn left
        service.send(service.topicCmd + "ti/rc","0.05 0.5")
        status = 1
    elif(xy[0] < middle_x - range):
        #then turn right
        service.send(service.topicCmd + "ti/rc","0.05 -0.5")
        status = 2
    else:
        #ball is in the middle
        status = 0

    #update the picture and the ball detection

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
