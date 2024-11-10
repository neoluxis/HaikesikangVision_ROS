import cv2 as cv

cap = cv.VideoCapture('/dev/video-cam2')
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv.CAP_PROP_FPS, 10)

# out = cv.VideoWriter('1110.avi',  
#                          cv.VideoWriter_fourcc(*'MJPG'), 
#                          10, (640, 480))


while True:
    _, frame = cap.read()
    if not _: 
        print("No open")
        break
    # cv.imshow("f", frame)
    # # out.write(frame)
    # key = cv.waitKey(1)
    # if key == ord('q'):
    #     break
    cv.imwrite("a.png", frame)
    print("Write")
    break

cap.release()
# out.release()