import mediapipe as mp
import cv2 as cv
import numpy as np
import autopy
import time
# from autopy.mouse import LEFT_BUTTON, RIGHT_BUTTON

# print(autopy.screen.size())
scrnWidth , scrnHeight = autopy.screen.size()
width,height = 640,480
paddingTop, paddingBottom = 50,150
cap = cv.VideoCapture(0)
cap.set(3,width)
cap.set(4,height)
cursorX, cursorY =0,0

prevX,prevY = 0,0
currX,currY = 0, 0
smoothening = 5
import time
mpHands = mp.solutions.hands
hands = mpHands.Hands(max_num_hands=1,
               min_detection_confidence=0.8,
               min_tracking_confidence=0.8)
mpDraw = mp.solutions.drawing_utils
def left_click():
    """Perform a left mouse click."""
    autopy.mouse.click()
    time.sleep(.01)
while True:
    rightx, righty, leftx, lefty = 0, 0, 2000, 1
    top_point = (0, 0)
    bottom_point = (0, 0)
    ret, img = cap.read()
    img = cv.flip(img,1)
    RGBimg = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    results = hands.process(RGBimg)
    cv.rectangle(img,(paddingTop,paddingTop),(width-paddingBottom,height - paddingBottom),(0,0,255),1)
    if results.multi_hand_landmarks:
        # for hand in results.multi_hand_landmarks:
        for hand, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
        # Determine handedness (right or left hand)

            if handedness.classification[0].label == 'Right':
                  #### USES ONLY RIGHT HAND ####

                mpDraw.draw_landmarks(img, hand, mpHands.HAND_CONNECTIONS)
                pos8 = hand.landmark[8]
                ### Index finger coordinate for mouse positioning
                cx, cy = pos8.x * width, pos8.y * height
                cursorX = np.interp(cx, (paddingTop, width - paddingBottom)
                                    , (0, scrnWidth - 1))
                cursorY = np.interp(cy, (paddingTop, height - paddingBottom),
                                    (0, scrnHeight - 1))
                # print(cursorX,cursorY)
                currX = prevX + (cursorX - prevX) / smoothening
                currY = prevY + (cursorY - prevY) / smoothening
                autopy.mouse.move(currX, currY)
                prevX, prevY = currX, currY

                ### Thumb finger (tip) and index finger (bottom)
                 #  for determining the left click
                pos4 = hand.landmark[4]
                pos5 = hand.landmark[5]
                rightx, righty = pos4.x * width, pos4.y * height
                leftx, lefty = pos5.x * width, pos5.y * height

                ### Index finger positions for right click
                pos12 = hand.landmark[12]
                pos11 = hand.landmark[11]
                presentx, presenty = int(pos12.x * width), int(pos12.y * height)
                top_point = (presentx, presenty)
                presentx, presenty = int(pos11.x * width), int(pos11.y * height)
                bottom_point = (presentx, presenty)

    if leftx -rightx<0:
        autopy.mouse.click()
        # time.sleep(0.05)
        cv.rectangle(img, (paddingTop, paddingTop), (width - paddingBottom, height - paddingBottom),
                                 (0, 255,0), 3)
        print("LEFT clicked")

    if top_point[1] > bottom_point[1]:
        autopy.mouse.click(autopy.mouse.Button.RIGHT)
        cv.rectangle(img, (paddingTop, paddingTop), (width - paddingBottom, height - paddingBottom),
                     (255, 0, 0), 3)
        print("RIGHT clicked")

    cv.imshow("Image",img)
    cv.waitKey(1)

