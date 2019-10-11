import cv2
import sys

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)


def arGenerator():
    fileName = "ar.png"
    generator = aruco.drawMarker(dictionary, 0, 100)
    cv2.imwrite(fileName, generator)


def main(args):
    arGenerator()


if __name__ == '__main__':
    main(sys.argv)

