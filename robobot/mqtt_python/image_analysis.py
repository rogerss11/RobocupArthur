from skimage.morphology import (
    remove_small_holes,
    remove_small_objects,
    binary_closing,
    binary_opening,
    disk,
    binary_erosion,
)
from skimage.measure import label, regionprops, regionprops_table
from scipy.ndimage import gaussian_filter
import pandas as pd
import numpy as np
import cv2 as cv
import time
import drive

from scam import cam
from uservice import service
from sedge import edge
from sgpio import gpio


def imageAnalysis(save):
    if cam.useCam:
        ok, img, imgTime = cam.getImage()
        if not ok:  # size(img) == 0):
            if cam.imageFailCnt < 5:
                print("% Failed to get image.")
        else:
            h, w, ch = img.shape
            if not service.args.silent:
                # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
                pass
            # edge.paint(img)
            if save:
                fn = f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
                cv.imwrite(fn, img)
                if not service.args.silent:
                    print(f"% Saved image {fn}")
            pass
        pass
    pass
    return img, ok


def servo_up():
    service.send(service.topicCmd + "T0/servo", "1 -900 0")
    time.sleep(0.1)
    pass


def servo_down():
    service.send(service.topicCmd + "T0/servo", "1 -150 0")
    time.sleep(0.1)
    pass


def ball(image, color: int):
    """
    Detects a ball of a certain color in the image.
     image: the image to be analyzed
     color: the color of the ball (0 = blue, 1 = orange, 2 = red)
    """
    # color thresholds element 0 = blue, 1 = orange, 2 = red
    # Blue
    b_low = [190, 0, 0]
    b_high = [255, 30, 100]

    # Green
    g_low = [60, 0, 0]
    g_high = [255, 100, 50]

    # Red
    r_low = [10, 80, 150]

    mask = np.zeros_like(
        image[:, :, 0], dtype=bool
    )  # create a mask with the same size as the image

    # color of the thresholds (for images in BGR)
    if color == 0:  # blue
        mask = (
            (image[:, :, 0] > image[:, :, 1]) & (image[:, :, 0] > image[:, :, 2] + 20)  # blue intensity is higher than green and red
            & (image[:, :, 0] >= b_low[color])  # blue
            & (image[:, :, 1] >= g_low[color])  # green
            & (image[:, :, 2] >= r_low[color])  # red
            & ((image[:, :, 0].astype(np.int32) + image[:, :, 1].astype(np.int32)+ image[:, :, 2].astype(np.int32))< 650)  # color is not white
        )
    elif color == 1:  # orange
        mask = (
            (image[:, :, 2] > image[:, :, 0]) & (image[:, :, 2] > image[:, :, 1])  # red intensity is higher than green and blue
            & ((image[:, :, 1] + 10) > image[:, :, 0])  # green intensity is higher than blue
            & (image[:, :, 0] >= b_low[color]) & (image[:, :, 0] <= b_high[color])  # blue
            & (image[:, :, 1] >= g_low[color]) & (image[:, :, 1] <= g_high[color])  # green
            & (image[:, :, 2] >= r_low[color])  # red
        )
    elif color ==  2: #red
        mask = (
            (image[:, :, 2] > image[:, :, 0]) & (image[:, :, 2] > image[:, :, 1])  # red intensity is higher than green and blue
            & (image[:, :, 0]  > image[:, :, 1])  # green intensity is lower than blue
            & (image[:, :, 0] >= b_low[color]) & (image[:, :, 0] <= b_high[color])  # blue
            & (image[:, :, 1] >= g_low[color]) & (image[:, :, 1] <= g_high[color])  # green
            & (image[:, :, 2] >= r_low[color])  # red
        )


    if color < 2:
        # clean up the picture
        mask = remove_small_holes(mask, 500)
        mask = binary_closing(mask, disk(5))
        mask = remove_small_objects(mask, 200)
        mask = binary_opening(mask, disk(5))
        mask = remove_small_objects(mask, 300)

        # prevent detecting the arm or the background
        mask[:300, :] = 0  # remove the upper part of the picture
        mask[:, :70] = 0  # Left side (0 to 70 pixels)
        mask[:, 700:] = 0  # Right side (700 to 820 pixels)
    else:
        mask = remove_small_holes(mask, 500)
        mask = binary_closing(mask, disk(5))
        mask = remove_small_objects(mask, 100)
        mask = remove_small_objects(mask, 300)

        #prevent detecting the arm or the background
        mask[:150,:] = 0      # remove the upper part of the picture
        mask[:, :70] = 0      # Left side (0 to 70 pixels)
        mask[:, 700:] = 0     # Right side (750 to 820 pixels)

    # find the middle of the ball from the up left corner
    labeled_image, n_labels = label(mask, background=0, return_num=True, connectivity=1)
    regions = regionprops(labeled_image)

    # create a table with the properties of the regions
    # centroid = (y,x) = (row, column)
    region_table = regionprops_table(
        labeled_image, properties=["centroid"]
    )
    pd_regions = pd.DataFrame(region_table)
    pd_regions = pd_regions.sort_values(
        by="centroid-0", ascending=False
    )  # sort by y coordinate

    xy = []

    if len(pd_regions) == 1:
        xy = tuple(map(int, regions[0].centroid[::-1]))

    elif len(pd_regions) > 1:
        xy = (
            int(pd_regions.iloc[0]["centroid-1"]),
            int(pd_regions.iloc[0]["centroid-0"]),
        )

    # gives back a tuple with the pixel position of the (roughly) middle of the ball 
    return xy  


def move_middle(xy, range, color):
    """
    Moves the robot so that the object is in the middle of the picture.
     xy: the coordinates of the object in the picture
    """
    # the whole image is of the size 616x820x3
    middle_x = 410
    status = 99
    wait = 0.0
    if color == 0:
        velocity = 0.6
    else:
        velocity = 0.9

    # distance of the object to the middle of the picture
    e = abs(xy[0] - middle_x)
    print("MM: Distance to middle: ", e)

    if xy[0] > middle_x + range:
        # then turn left
        service.send(service.topicCmd + "ti/rc", f"0.01 -{velocity:.2f}")
        status = 1
    elif xy[0] < middle_x - range:
        # then turn right
        service.send(service.topicCmd + "ti/rc", f"0.01 {velocity:.2f}")
        status = 2
    else:
        # ball is in the middle
        status = 0
        service.send(service.topicCmd + "ti/rc", "0 0")

    if color == 0:
        wait = (e / middle_x) * 0.6 + 0.05
    else:
        wait = (e / middle_x) * 0.6 + 0.025
    # stop to update the picture and the ball detection
    time.sleep(wait)
    service.send(service.topicCmd + "ti/rc", "0 0")

    return status


# calculate the distance to the ball
def distance_calc(xy, color: int):
    """
    Calculate the distance to the ball based on its width and coordinates.
     xy: the coordinates of the ball in the picture
     type: the type of the ball (0 = oval, 1 = others)
    """

    distance = 0.0

    if xy != []:
        # calculate the distance to the ball by the coordinates of the ball
        if color == 0:  # blue ball
            a = -0.0000079881
            b = 0.011625897
            c = -5.74405
            d = 997.518

        elif color == 1:  # orange ball
            a = -0.00000502
            b = 0.00765108
            c = -4.0016825
            d = 751.342459

        distance = (a * xy[1] ** 3 + b * xy[1] ** 2 + c * xy[1] + d) * 10  # in mm

    print("Distance: ", distance)

    arm_length = 220  # in mm

    if distance > arm_length:
        distance = distance - arm_length  # in mm
        status = 1
    else:
        drive.driveXMeters(x=-0.1, vel=0.2)  # move back a little bit
        distance = 0.0
        status = -1

    return distance, status


# move to the ball
def move_straight(xy, distance, state: int, line: int, color: int, whiggle):
    """
    Moves the robot straight to the ball.
        xy: the coordinates of the ball in the picture
        distance: the distance to the object in mm
        state: start with 3 to remove obstacles, then use the state of the function
        line: 0 = normal drive, 1 = line following
    """

    final_distance = 0

    if color == 0:  # blue
        final_distance = 150
    elif color == 1:  # orange
        final_distance = 250

    if state == 3:
        print("MS: Remove obstacles")
        # remove obstacles in the way
        drive.turnInPlace(deg=30, dir=0)
        servo_down()
        drive.turnInPlace(deg=60, dir=1)
        servo_up()
        drive.turnInPlace(deg=30, dir=0)
        state = 2

    elif state == 2:
        # adjust the position of the ball in the middle of the picture

        print("MS: Move to the middle")
        if distance > final_distance:
            stat_m = move_middle(xy, 10, color)
        else:
            if color == 0:
                stat_m = move_middle(xy, 5, color)
            else:
                stat_m = move_middle(xy, 3, color)

        if stat_m == 0:
            # ball is in the middle
            state = 1
    elif state == 1:
        # move for 20 cm
        velocity = 0.1  # in m/s

        state = 0

        if line == 0:
            print("MS: Move straight")

            if distance > 500.0:
                drive.driveXMeters(x=0.10, vel=velocity)
            elif distance > final_distance:
                drive.driveXMeters(x=0.05, vel=velocity)
            else:
                if color == 0:
                    drive.driveXMeters(x=(distance - 45) / 1000, vel=velocity)
                if color == 1:
                    velocity = 0.05
                    drive.driveXMeters(x=(distance - 35) / 1000, vel=velocity)

        else:
            if distance > final_distance:
                wait = 100.0 / 1000 / velocity
            else:
                wait = distance / 1000 / velocity
            print("MS: Move straight on line")
            edge.lineControl(velocity, 0.0)
            time.sleep(wait)
            edge.lineControl(0.0, 0.0)

        if distance < final_distance:
            # move the arm up
            print("MS: Pick up ball")
            state = 100
            servo_down()
            if whiggle:
                for i in range(2):
                    print("MS: Shaking")
                    service.send(service.topicCmd + "ti/rc", "0.1 0.6")
                    time.sleep(0.2)
                    service.send(service.topicCmd + "ti/rc", "0 0")
                    time.sleep(0.1)
                    service.send(service.topicCmd + "ti/rc", "-0.1 -0.6")
                    time.sleep(0.2)
                    service.send(service.topicCmd + "ti/rc", "0 0")
                    time.sleep(0.1)
            service.send(service.topicCmd + "ti/rc", "0 0")

    else:
        state = -1

    return state


# drive to the ball
def drive2ball(case: int):
    """
    Drive to the ball and pick it up.
        case: the type of the object (0 = oval blue ball, 1 = orange golf ball without line, 2 = orange golf ball with line)
    """

    img_num = 0

    # decode the cases
    if case == 0:
        # state_straight = 3 # remove obstacles
        state_straight_init = 2  # move object to middle first
        color = 0  # blue
        state = 0  # start with the first state
        whiggle = 1
    elif case == 1:
        state_straight_init = 2  # move object to middle first
        color = 1  # orange
        state = 0  # start with the first state
        whiggle = 1
    elif case == 2:
        state_straight_init = 1  # just move straight
        color = 1  # orange
        state = 1  # start with the first state
        whiggle = 0

    state_straight = state_straight_init  # state of the straight movement

    while state != 2:
        # Take a picture, until taking a picture is successful
        ok = False
        img = np.zeros((616, 820, 3), dtype=np.uint8)  # create an empty image

        servo_up()
        while not ok:
            img, ok = imageAnalysis(0)
            img_num += 1

        # find ball in the picture
        xy = ball(img, color)
        print("DB: Ball found:", xy)

        # Visualize the ball in the picture
        if len(xy) == 2:
            img = cv.circle(img, xy, radius=10, color=(0, 0, 255), thickness=-1)
            # Show the image for debugging
            if not gpio.onPi:
                print("DB: Show image")
                cv.imshow("frame for analysis", img)

                # fn = f"{img_num}_analyzed.jpg"
                # cv.imwrite(fn, img)

        if state == 0:
            print("DB: State 0")

            if xy != []:  # found one or more balls
                print("DB: Found one or more balls. Nearest ball:", xy)
                status_middle = move_middle(xy, 10, color)

                if status_middle == 0:
                    # ball is in the middle
                    state = 1
                    print("DB: Ball in the middle. Move straight.")
        elif state == 1:
            print("DB: State 1")
            # move straight to the ball
            if xy == []:
                drive.driveXMeters(x=-0.07, vel=0.2)  # move back a little bit
            distance, status_d = distance_calc(xy, color)
            print("DB: Distance to ball:", distance)
            if status_d != -1:
                if state_straight == 0:
                    state_straight = state_straight_init

                print("DB: State straight:", state_straight)
                state_straight = move_straight(
                    xy, distance, state_straight, case == 2, color, whiggle
                )

            else:
                # stop
                service.send(service.topicCmd + "ti/rc", "0 0")

            if state_straight == 100:
                # ball is captured
                state = 2
                print("DB: Ball is picked up.")

    return state
