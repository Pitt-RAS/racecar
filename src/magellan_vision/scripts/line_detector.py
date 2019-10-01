import cv2
import numpy as np
import time
import math
from statistics import mean
# import matplotlib.pyplot as plt

ddepth = cv2.CV_16S
kernel_size = 3
skipped_frames = 0
num_of_frames = 0


def skeletize(img, size, skel):
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    done = False
    while(not done):
        eroded = cv2.erode(img, element)
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(img, temp)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded.copy()
        zeros = size - cv2.countNonZero(img)
        if zeros == size:
            done = True
    return skel


def draw_lines(img, lines, color=[0, 255, 0], thickness=3):
    if lines is None:
        return
    img = np.copy(img)
    line_img = np.zeros(
        (
            img.shape[0],
            img.shape[1],
            3,
        ),
        dtype=np.uint8,
    )
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)

    img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
    return img


def best_fit(x_points, y_points):

    m = (((mean(x_points)*mean(y_points)) - mean(x_points*y_points)) /
         ((mean(x_points)*mean(x_points))-mean(x_points*x_points)))

    b = mean(y_points) - m*mean(x_points)

    return m, b


def process_lines(linesP, frame):
    i = 0
    left_line_x = []
    left_line_y = []
    right_line_x = []
    right_line_y = []

    crop = frame

    for line in linesP:
        for x1, y1, x2, y2 in line:
            if i == 0:
                i += 1
                max_y = y2
                min_y = y1
            if y2 < min_y:
                min_y = y1
            if y1 < max_y:
                max_y = y2
            slope = (y2 - y1) / (x2 - x1)  # <-- Calculating the slope.
        if math.fabs(slope) < 0.8:  # <-- Only consider extreme slope
            continue
        if x1 < frame.shape[1]/2 and x2 < frame.shape[1]/2:  # <-- If the slope is negative, left group.
            left_line_x.extend([x1, x2])
            left_line_y.extend([y1, y2])
        else:  # <-- Otherwise, right group.
            right_line_x.extend([x1, x2])
            right_line_y.extend([y1, y2])
    # TODO: Dynamically assign min and max y, using the iteration above???
    min_y = (int)(frame.shape[0] * (3/5))
    max_y = frame.shape[0]

    # TODO: Instead of poly, maybe try cv2.rectangle to match the area covered by linesP more accurately
    if len(left_line_x) != 0 and len(left_line_y) != 0 and len(right_line_y) != 0 and len(right_line_x) != 0:
        poly_left = np.poly1d(np.polyfit(
            left_line_y,
            left_line_x,
            deg=1
        ))
        left_x_start = int(poly_left(max_y))
        left_x_end = int(poly_left(min_y))

        poly_right = np.poly1d(np.polyfit(
            right_line_y,
            right_line_x,
            deg=1
        ))
        right_x_start = int(poly_right(max_y))
        right_x_end = int(poly_right(min_y))
        # TODO: Filter out "bad" lines (i.e. if the two lines intersect)
        crop = draw_lines(
            frame,
            [[
                [left_x_start, max_y, left_x_end, min_y],
                [right_x_start, max_y, right_x_end, min_y],
            ]],
            thickness=5,
        )

    cv2.imshow('Processed', crop)
    return crop


video = cv2.VideoCapture("IGVC_Video.mp4")
while True:
    ret, frame = video.read()
    tic = time.time()
    height, width, channels = frame.shape
    # print("height: {}, width: {}".format(height,width))
    if not ret:
        video = cv2.VideoCapture("IGVC_Video.mp4")
        continue
    num_of_frames += 1
    # print(num_of_frames)
    crop = frame[360:670, 0:1280]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    ret, thresh4 = cv2.threshold(blurred, 200, 255, cv2.THRESH_TOZERO)
    # cv2.imshow(cropped_image)
    size = np.size(thresh4)
    skel = np.zeros(thresh4.shape, np.uint8)

    skeleton = skeletize(thresh4, size, skel)

    dst = cv2.Laplacian(skeleton, ddepth, ksize=kernel_size)
    abs_dst = cv2.convertScaleAbs(dst)

    linesP = cv2.HoughLinesP(abs_dst, rho=6, theta=np.pi / 60, threshold=50,
                             lines=np.array([]), minLineLength=30, maxLineGap=10)

    # x_array_left = [[]]
    # x_array_right = [[]]
    # print(x_array.shape)
    # y_array_left = [[]]
    # y_array_right = [[]]
    # print(y_array.shape)

    crop2 = crop

    if linesP is not None and len(linesP) > 40:
        # processed = process_lines(linesP,crop)
        # print(len(linesP))
        for line in linesP:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1)
                # <-- Calculating the slope.
                if math.fabs(slope) < .5:
                    # <-- Only consider extreme slope
                    continue
                if x1 < frame.shape[1]/2 and x2 < frame.shape[1]/2:
                    # <-- If the slope is negative, left group
                    # x_array_left.append([x1,x2])
                    # y_array_left.append([y1,y2])
                    cv2.line(crop2, (x1, y1), (x2, y2), (0, 0, 255), 3)
                    # cv2.circle(crop, (x1, y1),(5), (0,0,255), 3)
                    # cv2.circle(crop, (x2, y2),(5), (0,0,255), 3)
                else:  # <-- Otherwise, right group.
                    # x_array_right.append([x1,x2])
                    # y_array_right.append([y1,y2])
                    cv2.line(crop2, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    # cv2.circle(crop, (x1, y1),(5), (0,255,0), 3)
                    # cv2.circle(crop, (x2, y2),(5), (0,255,0), 3)
            circles = crop
            lines = crop2

    if linesP is None or len(linesP) < 40:
        skipped_frames += 1

    cv2.imshow('Crop', crop)
    # linesAndCircles = cv2.addWeighted(lines, 0.5, circles, 0.5,0)
    # final = cv2.addWeighted(processed, 0.8, linesAndCircles, 0.5,0)
    # cv2.imshow('Original', frame)
    # cv2.imshow('Combo', final)
    # print(skipped_frames)

    # FIXME: what is this crap, please FIX ME
    # plt.plot(np.unique(x_array_right),
    #          np.poly1d(np.polyfit(x_array_right, y_array_right, 1))
    #          (np.unique(x_array_right)))

    # right_line_slope,right_line_intercept = best_fit(np.asarray(x_array_right),np.asarray(y_array_right))
    # left_line_slope,left_line_intercept = best_fit(np.asarray(x_array_left),np.asarray(y_array_left))

    toc = time.time()
    delta_t = toc - tic
    print("FPS: {}".format(1/delta_t))

    key = cv2.waitKey(25)
    if key == 27:
        break

video.release()
cv2.destroyAllWindows
