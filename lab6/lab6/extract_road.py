import cv2
import numpy
import math

image = cv2.imread('/home/tarun/shot.png')

def order_box(box: numpy.ndarray) -> numpy.ndarray: # orders points of bounding box to ensure a consistent order (top-left, top-right, bottom-right, bottom-left)
    srt = numpy.argsort(box[:, 1])
    btm1, btm2 = box[srt[0]], box[srt[1]]
    top1, top2 = box[srt[2]], box[srt[3]]

    btm_l = btm1 if btm1[0] < btm2[0] else btm2
    btm_r = btm2 if btm1[0] < btm2[0] else btm1

    top_l = top1 if top1[0] < top2[0] else top2
    top_r = top2 if top1[0] < top2[0] else top1

    return numpy.array([top_l, top_r, btm_r, btm_l])


def calc_line_length(p1, p2) -> float: # calculates euclidean distance between 2 points p1 and p2
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return numpy.sqrt(dx * dx + dy * dy)

def calc_box_vector(box: numpy.ndarray): # calculates midpoint of the shorter side of the bounding box
    v_side = calc_line_length(box[0], box[3])
    h_side = calc_line_length(box[0], box[1])
    idx = [0, 3, 1, 2] if v_side < h_side else [0, 1, 2, 3]
    return ((box[idx[0]][0] + box[idx[1]][0]) // 2, (box[idx[0]][1] + box[idx[1]][1]) // 2), (
        (box[idx[2]][0] + box[idx[3]][0]) // 2,
        (box[idx[2]][1] + box[idx[3]][1]) // 2,
    )

def get_vert_angle(p1, p2, w: float, h: float) -> float: # calculates vertical angle between points p1 and p2
    
    px1, px2 = p1[0] - w / 2, p2[0] - w / 2
    py1, py2 = h - p1[1], h - p2[1]
    if abs(px1-px2)>80:
        if abs(px1)<abs(px2):
            pass
        else:
            px1,px2 = px2, px1
            py1,py2=py2,py1

    angle = numpy.degrees(math.atan2((py2-py1),(px2-px1)))
    if angle >= -180 and angle <= -90:
        angle = 360+angle
    return angle

def mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        h = image[y, x, 0]
        s = image[y, x, 1]
        v = image[y, x, 2]
        print("H:", h)
        print("S:", s)
        print("V:", v)

cv2.namedWindow('mouse')
cv2.setMouseCallback('mouse', mouse)
cv2.imshow("original image", image)
cv2.imshow("mouse", image)
cv2.waitKey(0)
cv2.destroyAllWindows()

light_line = numpy.array([0, 0, 0])
dark_line = numpy.array([260, 100, 100])
mask = cv2.inRange(image, light_line, dark_line)
r1 = 170
c1 = 0
img = mask[r1:r1+300, c1:c1+512]
cv2.imshow('mask', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
cnts, _ = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
print([cv2.contourArea(c) for c in cnts])
max_contour = max(cnts, key=cv2.contourArea)
rect=cv2.minAreaRect(max_contour)
print("-------",rect[2])


# canny = cv2.Canny(mask, 30, 5)
# cv2.imshow('edge', canny)
# cv2.waitKey(0)
# cv2.destroyAllWindows()



# print(canny.shape)

# r1 = 250
# c1 = 0
# img = canny[r1:r1+300, c1:c1+512]
# lines = cv2.HoughLines(img, 1, numpy.pi/180, 100)
# print("lines", lines)
# if lines is not None:
#     print("lines", lines)
#     for line in lines:
#         rho, theta = line[0]
#         angle = numpy.degrees(theta)
#         print("----",angle)
# cv2.imshow('crop', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# edge = []
# row = 150
# for i in range(512):
#     if img[row, i] == 255:
#         edge.append(i)
# print(edge)

# if len(edge) == 4:
#     left_edge = edge[0]
#     right_edge = edge[2]
#     print(edge)

# if len(edge) == 3:
#     if edge[1] - edge[0] > 5:
#         left_edge = edge[0]
#         right_edge = edge[2]
#     else:
#         left_edge = edge[0]
#         right_edge = edge[2]

# road_width = right_edge - left_edge
# frame_mid = left_edge + (road_width / 2)
# mid_point = 512 / 2
# img[row, int(mid_point)] = 255
# print(mid_point)

# error = mid_point - frame_mid
# if error < 0:
#     action = "Go Right"
# else:
#     action = "Go Left"

# print("error", error)
# img[row, int(frame_mid)] = 255
# print("mid point of the frame", frame_mid)

# f_image = cv2.putText(img, action, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1, cv2.LINE_AA)
# cv2.imshow('final image', f_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
