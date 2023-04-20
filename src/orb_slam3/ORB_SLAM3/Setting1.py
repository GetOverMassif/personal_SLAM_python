import cv2
import sys
from Tools import errorPrint

class Setting:
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
        if (sensor_ == System.STEREO or sensor_ == System.IMU_STEREO)
        {
            readCamera2(fSettings)
            print(f'\t-Loaded camera 2\n',end='')
        }

        # Read image info
        readImageInfo(fSettings)
        print(f'\t-Loaded image info\n',end='')

        if (sensor_ == System.IMU_MONOCULAR or sensor_ == System.IMU_STEREO or sensor_ == System.IMU_RGBD)
        {
            readIMU(fSettings)
            print(f'\t-Loaded IMU calibration\n',end='')
        }

        if (sensor_ == System.RGBD or sensor_ == System.IMU_RGBD)
        {
            readRGBD(fSettings)
            print(f'\t-Loaded RGB-D calibration\n',end='')
        }

        readORB(fSettings)
        print(f'\t-Loaded ORB settings\n',end='')
        readViewer(fSettings)
        print(f'\t-Loaded viewer settings\n',end='')
        readLoadAndSave(fSettings)
        print(f'\t-Loaded Atlas settings\n',end='')
        readOtherParameters(fSettings)
        print(f'\t-Loaded misc parameters\n',end='')

        if (bNeedToRectify_)
            precomputeRectificationMaps()
            print(f'\t-Computed rectification maps\n',end='')

        print(f'----------------------------------\n',end='')

    def readCamera1(cv.FileStorage &fSettings)
        found

        # Read camera model
        string cameraModel = readParameter<string>(fSettings, "Camera.type", found)

        vector<float> vCalibration
        if (cameraModel == "PinHole")
        {
            cameraType_ = PinHole

            # Read intrinsic parameters
            fx = readParameter<float>(fSettings, "Camera1.fx", found)
            fy = readParameter<float>(fSettings, "Camera1.fy", found)
            cx = readParameter<float>(fSettings, "Camera1.cx", found)
            cy = readParameter<float>(fSettings, "Camera1.cy", found)

            vCalibration = {fx, fy, cx, cy}

            calibration1_ = new Pinhole(vCalibration)
            originalCalib1_ = new Pinhole(vCalibration)

            # Check if it is a distorted PinHole
            readParameter<float>(fSettings, "Camera1.k1", found, False)
            if (found)
            {
                readParameter<float>(fSettings, "Camera1.k3", found, False)
                if (found)
                {
                    vPinHoleDistorsion1_.resize(5)
                    vPinHoleDistorsion1_[4] = readParameter<float>(fSettings, "Camera1.k3", found)
                }
                else
                {
                    vPinHoleDistorsion1_.resize(4)
                }
                vPinHoleDistorsion1_[0] = readParameter<float>(fSettings, "Camera1.k1", found)
                vPinHoleDistorsion1_[1] = readParameter<float>(fSettings, "Camera1.k2", found)
                vPinHoleDistorsion1_[2] = readParameter<float>(fSettings, "Camera1.p1", found)
                vPinHoleDistorsion1_[3] = readParameter<float>(fSettings, "Camera1.p2", found)
            }

            # Check if we need to correct distortion from the images
            if ((sensor_ == System.MONOCULAR or sensor_ == System.IMU_MONOCULAR) and vPinHoleDistorsion1_.size() != 0)
            {
                bNeedToUndistort_ = True
            }
        }
        else if (cameraModel == "Rectified")
        {
            cameraType_ = Rectified

            # Read intrinsic parameters
            fx = readParameter<float>(fSettings, "Camera1.fx", found)
            fy = readParameter<float>(fSettings, "Camera1.fy", found)
            cx = readParameter<float>(fSettings, "Camera1.cx", found)
            cy = readParameter<float>(fSettings, "Camera1.cy", found)

            vCalibration = {fx, fy, cx, cy}

            calibration1_ = new Pinhole(vCalibration)
            originalCalib1_ = new Pinhole(vCalibration)

            # Rectified images are assumed to be ideal PinHole images (no distortion)
        }
        else if (cameraModel == "KannalaBrandt8")
        {
            cameraType_ = KannalaBrandt

            # Read intrinsic parameters
            fx = readParameter<float>(fSettings, "Camera1.fx", found)
            fy = readParameter<float>(fSettings, "Camera1.fy", found)
            cx = readParameter<float>(fSettings, "Camera1.cx", found)
            cy = readParameter<float>(fSettings, "Camera1.cy", found)

            k0 = readParameter<float>(fSettings, "Camera1.k1", found)
            k1 = readParameter<float>(fSettings, "Camera1.k2", found)
            k2 = readParameter<float>(fSettings, "Camera1.k3", found)
            k3 = readParameter<float>(fSettings, "Camera1.k4", found)

            vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3}

            calibration1_ = new KannalaBrandt8(vCalibration)
            originalCalib1_ = new KannalaBrandt8(vCalibration)

            if (sensor_ == System.STEREO or sensor_ == System.IMU_STEREO)
            {
                colBegin = readParameter<int>(fSettings, "Camera1.overlappingBegin", found)
                colEnd = readParameter<int>(fSettings, "Camera1.overlappingEnd", found)
                vector<int> vOverlapping = {colBegin, colEnd}

                static_cast<KannalaBrandt8 *>(calibration1_)->mvLappingArea = vOverlapping
            }
        }
        else
        {
            cerr << "Error: " << cameraModel << " not known" << endl
            exit(-1)
        }