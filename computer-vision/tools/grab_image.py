import numpy as np
import cv2


def main():
    cap = cv2.VideoCapture(1)

    n = 0
    while n < 10:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Display the resulting frame
        cv2.imshow("frame", gray)
        k = cv2.waitKey(1)
        if k & 0xFF == ord(" "):
            cv2.imwrite(f"camera_calibration_{n}.jpg", gray)
            n += 1

        if k & 0xFF == ord("q"):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
