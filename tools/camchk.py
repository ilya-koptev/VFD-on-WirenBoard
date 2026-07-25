import cv2
import numpy as np

for backend, name in ((cv2.CAP_MSMF, "MSMF"), (cv2.CAP_DSHOW, "DSHOW")):
    cap = cv2.VideoCapture(1, backend)
    if not cap.isOpened():
        print(name, "не открылась")
        continue
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    for want in (-4, -7, -10, -13):
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        ok_set = cap.set(cv2.CAP_PROP_EXPOSURE, want)
        got = cap.get(cv2.CAP_PROP_EXPOSURE)
        auto = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
        fr = None
        for _ in range(8):
            r, f = cap.read()
            if r:
                fr = f
        g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
        print("%-5s want=%4d set_ok=%s got=%8.3f auto=%5.2f  frame mean=%6.2f max=%3d"
              % (name, want, ok_set, got, auto, g.mean(), g.max()))
    cap.release()
