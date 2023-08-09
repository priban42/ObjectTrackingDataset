"""
Generates a picture to print a new aruco board. These specific parameters (resolution, size etc.) have no been tested yet.
"""

import cv2
import numpy as np

aruco_dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board_size = (10,14)
size_chess_square = 0.07
size_aruco_square = 0.04
board = cv2.aruco.CharucoBoard((board_size[0], board_size[1]), size_chess_square, size_aruco_square, aruco_dic)

margin_size = 2  # margin in pixel wrt boarder of the img
marker_border = 1  # number of "bits" surrounding each aruco encoding

square_size = 10
img_size = (board_size[0]*square_size + margin_size*2, board_size[1]*square_size + margin_size*2)  # in pixels, better to be proportial to board size
img = board.generateImage(img_size, marginSize=margin_size, borderBits=marker_border)
print(type(img))
print(np.shape(img))

for y in range(board_size[1]):

    if y%2 == 0:
        # shift left
        img[margin_size + y*square_size:margin_size + y*square_size + square_size, margin_size: np.shape(img)[1] - margin_size - square_size] = img[margin_size + y*square_size:margin_size + y*square_size + square_size, margin_size + square_size: np.shape(img)[1] - margin_size]
        # adding black square
        img[margin_size + y*square_size:margin_size + y*square_size + square_size, np.shape(img)[1] - margin_size - square_size: np.shape(img)[1] - margin_size] = img[margin_size + y*square_size:margin_size + y*square_size + square_size, margin_size + square_size: margin_size + 2*square_size]
    else:
        #shift right
        img[margin_size + y*square_size:margin_size + y*square_size + square_size, margin_size + square_size: np.shape(img)[1] - margin_size] = img[margin_size + y*square_size:margin_size + y*square_size + square_size, margin_size: np.shape(img)[1] - margin_size - square_size]
        # adding black square
        img[margin_size + y*square_size:margin_size + y*square_size + square_size, margin_size: margin_size + square_size] = img[margin_size + y*square_size:margin_size + y*square_size + square_size, margin_size + 2*square_size: margin_size + 3*square_size]
cv2.imwrite('charuco_board_real.bmp', img)
