# -*- coding: utf-8 -*-
"""
Created on Jul 1 23:31:48 2021

@author: Lingyun
"""

import cv2
import math
import numpy as np
import sys
from matplotlib import pyplot as plt
import imutils


def convert(src_im: str, dst: str):
    aspect_ratio = 0.75 # width / height
    target_image_width  = 300
    target_image_height = int(target_image_width/aspect_ratio)
    img = cv2.imread(src_im)
    img_for_cog = cv2.imread(src_im, 0)
    width = int(img.shape[1])
    height = int(img.shape[0])
    print("Original Width",width)
    print("Original Height",height)
    img_aspect = float(width)/height
    print("Original Image Aspect Ratio", img_aspect)

    if img_aspect>aspect_ratio:
        resized_image = imutils.resize(img, height=target_image_height)
        resized_img_for_cog = imutils.resize(img_for_cog, height=target_image_height)
    else:
        resized_image = imutils.resize(img, width=target_image_width)
        resized_img_for_cog = imutils.resize(img_for_cog, width=target_image_width)

    width = int(resized_image.shape[1])
    height = int(resized_image.shape[0])
    print("Resized Width",width)
    print("Resized Height",height)

    #resized_image = cv2.medianBlur(resized_image, 5)
    #resized_image = cv2.adaptiveThreshold(resized_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
    #        cv2.THRESH_BINARY,11,2)
    #ret,thresh = cv2.threshold(gray,127,255,0)
    
    M = cv2.moments(resized_img_for_cog)
    # print(M)
    if (M["m00"] == 0):
        cX = int (resized_image.shape[1]/2)
        cY = int (resized_image.shape[0]/2)
    else:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    print("gravity center_x", cX)
    print("gravity_center_y", cY)

    left_pos = cX - int(target_image_width / 2) + 1
    right_pos = cX + int(target_image_width / 2)
    top_pos = cY - int(target_image_height / 2) + 1
    bottom_pos = cY + int(target_image_height / 2)
    print("Based on COG", left_pos, right_pos, top_pos, bottom_pos)

    if width>target_image_width:
        if left_pos<0:
            left_pos = 0
            right_pos = left_pos + target_image_width -1
        if right_pos>=width:
            right_pos = width -1
            left_pos = right_pos - target_image_width + 1
        top_pos = 0
        bottom_pos = target_image_height - 1
    else:
        if top_pos<0:
            top_pos = 0
            bottom_pos = top_pos + target_image_height -1
        if bottom_pos>=height:
            bottom_pos = height -1
            top_pos = bottom_pos - target_image_width + 1
        left_pos = 0
        right_pos = target_image_width - 1

    print("After Limit", left_pos, right_pos, top_pos, bottom_pos)
    resized_image = resized_image[top_pos:bottom_pos, left_pos:right_pos]
    #crop_img[target_image_height][target_image_width] = gray[]
    #resized = imutils.resize(img, target_image_width)
    gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
    edged = imutils.auto_canny(gray)
    #cv2.imshow("Original", resized_image)
    #cv2.imshow("Automatic Edge Map", edged)
    #cv2.waitKey()

    #edged = cv2.Canny(resized_image, 70, 250)
    
    #edged_test = cv2.
    plt.imshow(edged, extent = [0, 400, 0, 400])
    #ax=plt.axes()
    #ax.set_aspect('equal')
    plt.show()
    #M = cv2.momen
    #gray = cv2.bilateralFilter(gray, 50, 50, 10)
    # new_image = np.empty([len(img), len(img[0])], dtype=int)
    # for i in range(0, len(img)):
    #     for j in range(0, len(img[0])):
    #         if img[i][j][0] == 0:
    #             new_image[i][j] = 1

    size_y = len(resized_image)
    size_x = len(resized_image[0])
    print("Processed src_im...")
    print(size_x, size_y)

    # plot image
    #$plt.subplot(1,3,1)
    #plt.imshow(img, cmap='gray')

    # plot contour
    #plt.subplot(1,3,2)
    a = plt.contour(edged, levels=[255], colors='black', origin='image')
    # plot separated contour
    #plt.subplot(1,3,3)
    # 
    print("Plotted stuff...")

    p = 0
    last_percent_x = 0.5
    last_percent_y = 0.5
    count = 0
    v_limit = 0.002
    move_away_duration = 20
    hovering_duration = 20
    feed_in_duration = 40
    f = open(dst, 'w')

    for i in range(len(a.allsegs[0])):
        if len(a.allsegs[0][i]) >= 15:
            p = p + 1
            for j in range(len(a.allsegs[0][i])):
                x = a.allsegs[0][i][j, 0]
                y = a.allsegs[0][i][j, 1]
                current_percent_x = x / size_x
                current_percent_y = 1 - y / size_y
                target_percent_x = current_percent_x
                target_percent_y = current_percent_y
                distance_x = target_percent_x - last_percent_x
                distance_y = target_percent_y - last_percent_y
                distance = math.sqrt(distance_x ** 2 + distance_y ** 2)

                if j == 0:
                    # while (distance > v_limit):
                    internal_seg = math.ceil(distance / v_limit)
                    for k in range(move_away_duration):
                        count = count + 1
                        f.write("{} {} {} {}\n".format(count, last_percent_x, last_percent_y, 0))
                    for k in range(internal_seg):
                        current_percent_x = last_percent_x + distance_x / internal_seg * k
                        current_percent_y = last_percent_y + distance_y / internal_seg * k
                        count = count + 1
                        f.write("{} {} {} {}\n".format(count, current_percent_x, current_percent_y, 0))
                    for k in range(hovering_duration):
                        count = count + 1
                        f.write("{} {} {} {}\n".format(count, target_percent_x, target_percent_y, 0))
                    for k in range(feed_in_duration):
                        count = count + 1
                        f.write("{} {} {} {}\n".format(count, target_percent_x, target_percent_y, 1))

                else:
                    internal_seg = math.ceil(distance / v_limit)
                    for k in range(internal_seg):
                        current_percent_x = last_percent_x + distance_x / internal_seg * k
                        current_percent_y = last_percent_y + distance_y / internal_seg * k
                        count = count + 1
                        f.write("{} {} {} {}\n".format(count, current_percent_x, current_percent_y, 1))
                    count = count + 1
                    f.write("{} {} {} {}\n".format(count, target_percent_x, target_percent_y, 1))

                last_percent_x = target_percent_x
                last_percent_y = target_percent_y

                # print(x)
                # print(y)
                # plt.plot(x,y)
    for i in range(100):
        count = count + 1
        f.write(str(count) + ' ' + str(last_percent_x) + ' ' + str(last_percent_y) + ' ' + str(0) + '\n')

    f.close()

    with open(dst, "r+") as f:
        s = f.read()
        f.seek(0)
        f.write(str(count) + "\n" + s)

    #plt.set_aspect('equal')

if __name__ == '__main__':
    #print(len(sys.argv))
    #print(sys.argv[1])
    #print(sys.argv[2])
    #convert("image_0.jpg","a.txt")
    convert("khatib.PNG", "kh.txt")
