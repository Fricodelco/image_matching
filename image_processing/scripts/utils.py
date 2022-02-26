import cv2
import numpy as np
from decimal import Decimal
from geodetic_conv import GeodeticConvert


def resize_img(img, scale):
    width = int(img.shape[1] * Decimal(scale))
    height = int(img.shape[0] * Decimal(scale))
    dim = (width, height)  
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized


def rotate_image(mat, angle):
    height, width = mat.shape[:2] # image shape has 3 dimensions
    image_center = (width/2, height/2) # getRotationMatrix2D needs coordinates in reverse order (width, height) compared to shape

    rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1.)
    # rotation calculates the cos and sin, taking absolutes of those.
    abs_cos = abs(rotation_mat[0,0]) 
    abs_sin = abs(rotation_mat[0,1])

    # find the new width and height bounds
    bound_w = int(height * abs_sin + width * abs_cos)
    bound_h = int(height * abs_cos + width * abs_sin)

    # subtract old image center (bringing image back to origo) and adding the new image center coordinates
    rotation_mat[0, 2] += bound_w/2 - image_center[0]
    rotation_mat[1, 2] += bound_h/2 - image_center[1]

    # rotate image with the new bounds and translated rotation matrix
    rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h))
    return rotated_mat

def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

def draw_circle_on_map_by_coord(img, coord, circle_coord, pixel_size):
    g_c = GeodeticConvert()
    g_c.initialiseReference(coord[0], coord[1], 0)
    x, y, z = g_c.geodetic2Ned(circle_coord[0], circle_coord[1], 0)
    pixel_x = int(x/float(pixel_size))
    pixel_y = int(y/float(pixel_size))
    print(pixel_y, pixel_x)
    img = cv2.circle(img, (pixel_y, pixel_x), 10, 255, 2)
    return img
