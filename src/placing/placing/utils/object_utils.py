"""
Object processing utilities for mask extraction and kernel generation.
Handles object mask operations and background subtraction.
"""
import numpy as np
import cv2


def get_points(mask):
    """
    Calculates the minimum and maximum x and y points of the largest contour in the mask

    Inputs:
        - mask: the mask containing the largest contour

    Returns:
        - [ymin,xmin]: Array containing the minimum x and y coordinates (in reference to the whole mask)
        - [ymax,xmax]: Array containing the maximum x and y coordinates (in reference to the whole mask)
    """
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnt = max(cnts, key=cv2.contourArea)  # Get largest contour (object to be placed)

    pts = cnt.reshape(-1, 2)
    xmin, ymin = pts.min(axis=0)
    xmax, ymax = pts.max(axis=0)

    return [ymin, xmin], [ymax, xmax]


def get_kernel(objectMask):
    """
    Creates a mask containing the object centered

    Inputs:
        - objectMask: the mask of the object to place
        - debug: boolean for visualizing outputs

    Returns:
        - kernel: kernel to use for dilation, contains object centered
    """
    objMask = objectMask.copy()
    flip = False  # boolean to determine if we flip the mask before computing
    minPoints, maxPoints = get_points(objMask)

    xD = maxPoints[1] - minPoints[1]
    yD = maxPoints[0] - minPoints[0]

    large = max(xD, yD)

    if large == xD: 
        flip = True
        objMask = objMask.T  # Rotates mask to line up the largest axis of the object with the y-axis
        
        minPoints = [minPoints[1], minPoints[0]]
        maxPoints = [maxPoints[1], maxPoints[0]]

        xDc = xD
        xD = yD
        yD = xDc

    kyMid = round(large/2)  # index of the middle of the object's largest axis

    ky = objMask[minPoints[0] + kyMid]  # points in the middle of the object's largest axis

    kyLength = np.count_nonzero(ky[minPoints[1]:maxPoints[1]])  # number of object pixels in the row
    
    kxMidIndex = 0  # represents the index of the median valid point in the row
    currIndW = 0  # to count number of valid pixels
    currInd = minPoints[1]  # to track the column index
    for i in ky[minPoints[1]:maxPoints[1]]:
        if i > 0: 
            currIndW += 1

        if currIndW == round(kyLength/2):
            kxMidIndex = currInd
            break
        currInd += 1

    kxMin = kxMidIndex - minPoints[1]  # Distance from the objects minimum x-point to the median point in the middle of the object
    kxMax = maxPoints[1] - kxMidIndex  # Distance from the objects maximum x-point to the median point in the middle of the object

    shiftX = kxMin - kxMax  # the difference between the distance of the objects middle point and the x-axis edge points (the amount of padding that will be added to center the object)

    # xRange represents the x-coords of the kernel in which the object will be placed
    if shiftX < 0: 
        xRange = [abs(shiftX), abs(shiftX) + xD]  # if the left side will be padded, the object should be placed after the padding
    else: 
        xRange = [0, xD]  # if the right side or no side will be padded, the object should be placed at the beginning of the kernel 

    kernel = np.zeros((yD, xD + abs(shiftX)))
    
    kernel[0:yD, xRange[0]:xRange[1]] = objMask[minPoints[0]:maxPoints[0], minPoints[1]:maxPoints[1]]  # Adds the object mask to the kernel

    if flip: 
        kernel = kernel.T  # If needed rotates the kernel to return it with the same orientation as the input

    return kernel