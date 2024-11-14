import cv2
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace = os.path.dirname(current_dir)
data_dir = os.path.join(workspace,'dataset/class0')

for k in range(7,12):
        for i in range(-1,2):
            # flag_x = 1-i
            for j in range(-1,2):
                # flag_y = 1-j
                print(os.path.join(data_dir,f'capture_{i+1+3*k}_{j+1+3*k}_0.jpg'))
                image = cv2.imread(os.path.join(data_dir,f'capture_{i+1+3*k}_{j+1+3*k}_0.jpg'))
                cv2.imwrite(os.path.join(data_dir,f'capture{i+1+3*k}_{j+1+3*k}_0.jpg'),image)
               