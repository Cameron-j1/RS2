import math
import numpy as np


H1_Y = 0.52
H1_X = -0.13

board_yaw = np.deg2rad(-19.5)
SQUARE_SIZE = 35
col = 5
row = 6


print(f"x_final: {H1_X + col*(SQUARE_SIZE/1000)*math.cos(board_yaw) + row*(SQUARE_SIZE/1000)*math.sin(board_yaw)}")
print(f"y_final: {H1_Y - row*(SQUARE_SIZE/1000)*math.cos(board_yaw) - col*(SQUARE_SIZE/1000)*math.cos(board_yaw)}")