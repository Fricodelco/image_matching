import cv2
import numpy as np
from decimal import Decimal
from geodetic_conv import GeodeticConvert
from math import atan2, pi

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

def draw_circle_on_map_by_coord_and_angles(img, coord, circle_coord, pixel_size, yaw, color):
    try: img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    except: img = img
    g_c = GeodeticConvert()
    g_c.initialiseReference(coord[0], coord[1], 0)
    y, x, z = g_c.geodetic2Ned(circle_coord[0], circle_coord[1], 0)
    
    pixel_x = int(x/float(pixel_size))
    pixel_y = int(-y/float(pixel_size))
    x2 = int(pixel_x + 20 * np.cos(yaw))
    # x2 = pixel_x + 50
    y2 = int(pixel_y - 20 * np.sin(yaw))
    # y2 = pixel_y
    img = cv2.line(img, (pixel_x, pixel_y), (x2, y2), color, 2)
    img = cv2.circle(img, (pixel_x, pixel_y), 4, color, 2)
    return img

def CrossProduct(A):
    X1 = (A[1][0] - A[0][0])
    Y1 = (A[1][1] - A[0][1])
    X2 = (A[2][0] - A[0][0])
    Y2 = (A[2][1] - A[0][1])
    return (X1 * Y2 - Y1 * X2)
        
def isConvex(points, shape, angle_restriction_homography, low_scale_restriction_homography, high_scale_restriction_homography):
    p = []
    for point in points:
        p.append([point[0][0], point[0][1]])
    N = len(points)
    prev = 0
    curr = 0
    for i in range(N):
        temp = [p[i], p[(i + 1) % N],
                        p[(i + 2) % N]]
        curr = CrossProduct(temp)
        if (curr != 0):
            if (curr * prev < 0):
                return False
            else:
                prev = curr
    dist1 = calculate_euqlidian_dist(p[0], p[1])
    dist2 = calculate_euqlidian_dist(p[1], p[2])
    # print(shape[0]/dist1, shape[1]/dist2)
    ls = low_scale_restriction_homography
    hs = high_scale_restriction_homography
    if shape[0]/dist1 < ls or shape[1]/dist2 < ls or shape[0]/dist1 > hs or shape[1]/dist2 > hs:
        return False
    # print(shape, dist1, dist2)
    angle_1 = angle(p[0], p[1], p[2])
    angle_2 = angle(p[1], p[2], p[3])
    angle_3 = angle(p[2], p[3], p[0])
    angle_4 = angle(p[3], p[0], p[1])
    # print(angle_1, angle_2, angle_3, angle_4)
    # print(abs(angle_1 - np.pi/2), abs(angle_2 - np.pi/2), abs(angle_3 - np.pi/2), abs(angle_4 - np.pi/2))
    delta = angle_restriction_homography
    if abs(angle_1 - np.pi/2) > delta:
        return False
    if abs(angle_2 - np.pi/2) > delta:
        return False
    if abs(angle_3 - np.pi/2) > delta:
        return False
    if abs(angle_4 - np.pi/2) > delta:
        return False
    
    # print(shape[0]/dist1, shape[1]/dist2)
    return True

def angle(A, B, C, /):
    Ax, Ay = A[0]-B[0], A[1]-B[1]
    Cx, Cy = C[0]-B[0], C[1]-B[1]
    a = atan2(Ay, Ax)
    c = atan2(Cy, Cx)
    if a < 0: a += pi*2
    if c < 0: c += pi*2
    return (pi*2 + c - a) if a > c else (c - a)

def calculate_euqlidian_dist(point1, point2):
    dist = np.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
    return dist 
    