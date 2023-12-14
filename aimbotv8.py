import numpy as np
import cv2
import time
import torch
import pyautogui
from ultralytics import YOLO

import win32api
import win32con
import ait
import time

# yolo8, pulling pretrained weight from yolov8, can also put own weight here
weight = 'yolov8s.pt'
model = YOLO(weight)

# test model single image object detection
# remove classes=32 if using a custom (non yolov8_.pt weight)
# can remove conf=0.05 if not using a yolov8n trained weight
#test = model('imgs/test2.jpg', save=True, conf = 0.05, classes=32)

# Runs until down arrow pressed Do not recomend running 
# since it will move your mouse pointer to the bottom left of your screen.
while True:
    t = time.time()
    img = pyautogui.screenshot(region = (0,0,1920,1200))
    #remove classes=32 for personal
    results = model(img, stream=True, classes=32)
    
    rl = True
    if np.size(results) != 0:
        for r in results:
        #if np.size(r.boxes) != 0:
            rl = r
            boxes = r.boxes
            old_distance = 1920000 #max distance squared
            bestx = 30
            besty = 30
            for box in boxes:
                xy = box.xyxy[0].tolist()
                xpos = int(1.75 * (((xy[0] + xy[2])/2) - pyautogui.position()[0])) # gets x distance needed to get to target
                ypos = int(1.60 * (((xy[1] + xy[3])/2) - pyautogui.position()[1])) # gets y distance needed to get to target
                
                # Compare to smallest current distance
                if (xpos**2 + ypos**2) < old_distance:
                    old_distance = xpos**2 + ypos**2
                    bestx = xpos
                    besty = ypos
    
            # Moves mouse to closest target location, then clicks
            win32api.mouse_event(win32con.MOUSEEVENTF_MOVE, int(bestx), int(besty))
            ait.click()
    
        # If no classifications found, move mouse down and to right 30 pixels each
    else:
        win32api.mouse_event(win32con.MOUSEEVENTF_MOVE, 30, 30)

    a = win32api.GetKeyState(0x28)  # down arrow key
    if a < 0:
        break
    
    # Outputs classifications of screen
    # recomend two monitors for this
    #im = cv2.resize(np.squeeze(rl.plot()),(1200, 800))
    #cv2.imshow('yolov8', im)
    #cv2.resize(600, 400)
    #cv2.waitKey(1)
    
    
    # time buffer for mouse movement to not over do it
    time.sleep(0.03)
    
cv2.destroyAllWindows()
