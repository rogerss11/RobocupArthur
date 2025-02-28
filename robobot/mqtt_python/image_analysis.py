import cv2 as cv
from skimage.morphology import remove_small_holes
from skimage.morphology import remove_small_objects
from skimage.measure import label
from skimage.measure import regionprops

# detection of the blue balls in the picture
def blue_ball(image):
    # color thresholds
    image_bl = (image[:,:,0] > 200) & (image[:,:,1] > 160) & (image[:,:,1] < 245) & (image[:,:,2] > 20) & (image[:,:,2] < 170)
    
    # clean up the picture
    image_bl_cl = remove_small_holes(image_bl, 10000, 1)
    mask = remove_small_objects(image_bl_cl, min_size=500, connectivity=1)

    # find the middle of the ball from the up left corner
    labeled_image, n_labels = label(mask, background=0,return_num=True,connectivity=2)
    regions = regionprops(label_image=labeled_image)
    if (n_labels == 1):
        xy = regions[0].centroid
    elif(n_labels == 0):
        print('No ball found')
        xy = [0,0]
    else:
        print('More than one ball found')

    return xy

'''
#the whole image is of the size 616x820x3
def middle_ball(xy):
    if(xy[0] > 410):
        #then turn left
    elif(xy[0] < 410):
        #then turn right
    else:
        #ball is in the middle
'''