import cv2
import sys
from ORB_SLAM3.Tools import errorPrint

import ORB_SLAM3.System as System
from ORB_SLAM3.CameraModels.Pinhole import Pinhole
from ORB_SLAM3.CameraModels.KannalaBrandt8 import KannalaBrandt8

def readParameterFloat(fSettings, name, required=False):
    node = cv2.FileNode(fSettings[name])
    if node.empty():
        if required:
            sys.stderr.write(f"{name} required parameter does not exist, aborting...\n")
            sys.exit(-1)
        else:
            sys.stderr.write(f"{name} optional parameter does not exist...")
            return 0.0, False
    elif not node.isReal():
        sys.stderr.write(f"{name} parameter must be a real number, aborting...")
        exit(-1)
    else:
        return node.real(), True


def readParameterInt(fSettings, name, required=False):
    node = cv2.FileNode(fSettings[name])
    if node.empty():
        if required:
            sys.stderr.write(f"{name} required parameter does not exist, aborting...\n")
            sys.exit(-1)
        else:
            sys.stderr.write(f"{name} optional parameter does not exist...")
            return 0, False
    elif not node.isInt():
        sys.stderr.write(f"{name} parameter must be a integer number, aborting...")
        exit(-1)
    else:
        return node.int(), True


def readParameterString(fSettings, name, required=False):
    node = cv2.FileNode(fSettings[name])
    if node.empty():
        if required:
            sys.stderr.write(f"{name} required parameter does not exist, aborting...\n")
            sys.exit(-1)
        else:
            sys.stderr.write(f"{name} optional parameter does not exist...")
            return 0, False
    elif not node.isString():
        sys.stderr.write(f"{name} parameter must be a string, aborting...")
        exit(-1)
    else:
        return node.string(), True


class Settings:
    # CameraType
    PinHole = 0
    Rectified = 1
    KannalaBrandt = 2

    def __init__(self, configFile, sensor) -> None:
        self.bNeedToUndistort_ = False
        self.bNeedToRectify_ = False
        self.bNeedToResize1_ = False
        self.bNeedToResize2_ = False
        self.sensor_ = sensor

        # Open settings file
        fSettings = cv2.FileStorage(configFile, cv2.FileStorage_READ)
        if not fSettings.isOpened():
            errorPrint(f"[ERROR]: could not open configuration file at: {configFile}")
            errorPrint(f"Aborting...")
            sys.exit(-1)
        else:
            print(f"Loading settings from {configFile}")

        # Read first camera
        self.readCamera1(fSettings)
        print(f'\t-Loaded camera 1\n',end='')

        # Read second camera if stereo (not rectified)
        if (self.sensor_ == System.STEREO or self.sensor_ == System.IMU_STEREO):
            self.readCamera2(fSettings)
            print(f'\t-Loaded camera 2\n',end='')

        # Read image info
        self.readImageInfo(fSettings)
        print(f'\t-Loaded image info\n',end='')

        if (self.sensor_ == System.IMU_MONOCULAR or self.sensor_ == System.IMU_STEREO or self.sensor_ == System.IMU_RGBD):
            self.readIMU(fSettings)
            print(f'\t-Loaded IMU calibration\n',end='')

        if (self.sensor_ == System.RGBD or self.sensor_ == System.IMU_RGBD):
            self.readRGBD(fSettings)
            print(f'\t-Loaded RGB-D calibration\n',end='')

        self.readORB(fSettings)
        print(f'\t-Loaded ORB settings\n',end='')
        self.readViewer(fSettings)
        print(f'\t-Loaded viewer settings\n',end='')
        self.readLoadAndSave(fSettings)
        print(f'\t-Loaded Atlas settings\n',end='')
        self.readOtherParameters(fSettings)
        print(f'\t-Loaded misc parameters\n',end='')

        if self.bNeedToRectify_:
            self.precomputeRectificationMaps()
            print(f'\t-Computed rectification maps\n',end='')
        print(f'----------------------------------\n',end='')

    def readCamera1(self, fSettings):
        # Read camera model
        cameraModel, found = readParameterString(fSettings, "Camera.type")
        vCalibration = []
        if (cameraModel == "PinHole"):
            self.cameraType_ = self.PinHole

            # Read intrinsic parameters
            fx, found = readParameterFloat(fSettings, "Camera1.fx")
            fy, found = readParameterFloat(fSettings, "Camera1.fy")
            cx, found = readParameterFloat(fSettings, "Camera1.cx")
            cy, found = readParameterFloat(fSettings, "Camera1.cy")

            vCalibration = [fx, fy, cx, cy]

            self.calibration1_ = Pinhole(vCalibration)
            self.originalCalib1_ = Pinhole(vCalibration)

            # Check if it is a distorted PinHole
            readParameterFloat(fSettings, "Camera1.k1", found, False)
            if found:
                readParameterFloat(fSettings, "Camera1.k3", found, False)
                if found :
                    self.vPinHoleDistorsion1_.resize(5)
                    self.vPinHoleDistorsion1_[4] = readParameterFloat(fSettings, "Camera1.k3")
                else:
                    self.vPinHoleDistorsion1_.resize(4)
                self.vPinHoleDistorsion1_[0] = readParameterFloat(fSettings, "Camera1.k1")
                self.vPinHoleDistorsion1_[1] = readParameterFloat(fSettings, "Camera1.k2")
                self.vPinHoleDistorsion1_[2] = readParameterFloat(fSettings, "Camera1.p1")
                self.vPinHoleDistorsion1_[3] = readParameterFloat(fSettings, "Camera1.p2")

            # Check if we need to correct distortion from the images
            if ((self.sensor_ == System.MONOCULAR or self.sensor_ == System.IMU_MONOCULAR) and len(self.vPinHoleDistorsion1_) != 0) :
                self.bNeedToUndistort_ = True
        elif (cameraModel == "Rectified"):
            self.cameraType_ = self.Rectified
            # Read intrinsic parameters
            fx = readParameterFloat(fSettings, "Camera1.fx")
            fy = readParameterFloat(fSettings, "Camera1.fy")
            cx = readParameterFloat(fSettings, "Camera1.cx")
            cy = readParameterFloat(fSettings, "Camera1.cy")

            vCalibration = {fx, fy, cx, cy}

            self.calibration1_ = Pinhole(vCalibration)
            self.originalCalib1_ = Pinhole(vCalibration)

            # Rectified images are assumed to be ideal PinHole images (no distortion)
        elif (cameraModel == "KannalaBrandt8"):
            self.cameraType_ = self.KannalaBrandt

            # Read intrinsic parameters
            fx = readParameterFloat(fSettings, "Camera1.fx")
            fy = readParameterFloat(fSettings, "Camera1.fy")
            cx = readParameterFloat(fSettings, "Camera1.cx")
            cy = readParameterFloat(fSettings, "Camera1.cy")

            k0 = readParameterFloat(fSettings, "Camera1.k1")
            k1 = readParameterFloat(fSettings, "Camera1.k2")
            k2 = readParameterFloat(fSettings, "Camera1.k3")
            k3 = readParameterFloat(fSettings, "Camera1.k4")

            vCalibration = [fx, fy, cx, cy, k0, k1, k2, k3]

            self.calibration1_ = KannalaBrandt8(vCalibration)
            self.originalCalib1_ = KannalaBrandt8(vCalibration)

            if (self.sensor_ == System.STEREO or self.sensor_ == System.IMU_STEREO):
                colBegin = self.readParameter<int>(fSettings, "Camera1.overlappingBegin")
                colEnd = self.readParameter<int>(fSettings, "Camera1.overlappingEnd")
                vOverlapping = [colBegin, colEnd]
                self.calibration1_.mvLappingArea = vOverlapping
        else:
            errorPrint(f"Error: {cameraModel} not known")
            sys.exit(-1)

