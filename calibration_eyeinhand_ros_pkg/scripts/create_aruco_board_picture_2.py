"""
Generates a picture to print a new aruco board. These specific parameters (resolution, size etc.) have no been tested yet.
"""

import cv2

aruco_dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board_size = (10,14)
size_chess_square = 0.07
size_aruco_square = 0.04
board = cv2.aruco.CharucoBoard_create(board_size[0], board_size[1], size_chess_square, size_aruco_square, aruco_dic)

img_size = (board_size[0]*10, board_size[1]*10)  # in pixels, better to be proportial to board size
margin_size = 10  # margin in pixel wrt boarder of the img
marker_border = 1  # number of "bits" surrounding each aruco encoding
img = board.draw(img_size, marginSize=margin_size, borderBits=marker_border)
cv2.imwrite('charuco_board.jpg', img)
