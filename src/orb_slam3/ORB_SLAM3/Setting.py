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
        readCamera1(fSettings)
        cout << "\t-Loaded camera 1" << endl

        # Read second camera if stereo (not rectified)
        if (sensor_ == System::STEREO or sensor_ == System::IMU_STEREO)
        {
            readCamera2(fSettings)
            cout << "\t-Loaded camera 2" << endl
        }

        # Read image info
        readImageInfo(fSettings)
        cout << "\t-Loaded image info" << endl

        if (sensor_ == System::IMU_MONOCULAR or sensor_ == System::IMU_STEREO or sensor_ == System::IMU_RGBD)
        {
            readIMU(fSettings)
            cout << "\t-Loaded IMU calibration" << endl
        }

        if (sensor_ == System::RGBD or sensor_ == System::IMU_RGBD)
        {
            readRGBD(fSettings)
            cout << "\t-Loaded RGB-D calibration" << endl
        }

        readORB(fSettings)
        cout << "\t-Loaded ORB settings" << endl
        readViewer(fSettings)
        cout << "\t-Loaded viewer settings" << endl
        readLoadAndSave(fSettings)
        cout << "\t-Loaded Atlas settings" << endl
        readOtherParameters(fSettings)
        cout << "\t-Loaded misc parameters" << endl

        if (bNeedToRectify_)
        {
            precomputeRectificationMaps()
            cout << "\t-Computed rectification maps" << endl
        }

        cout << "----------------------------------" << endl

    def readCamera1(cv::FileStorage &fSettings)
        bool found;

        // Read camera model
        string cameraModel = readParameter<string>(fSettings, "Camera.type", found);

        vector<float> vCalibration;
        if (cameraModel == "PinHole")
        {
            cameraType_ = PinHole;

            // Read intrinsic parameters
            float fx = readParameter<float>(fSettings, "Camera1.fx", found);
            float fy = readParameter<float>(fSettings, "Camera1.fy", found);
            float cx = readParameter<float>(fSettings, "Camera1.cx", found);
            float cy = readParameter<float>(fSettings, "Camera1.cy", found);

            vCalibration = {fx, fy, cx, cy};

            calibration1_ = new Pinhole(vCalibration);
            originalCalib1_ = new Pinhole(vCalibration);

            // Check if it is a distorted PinHole
            readParameter<float>(fSettings, "Camera1.k1", found, false);
            if (found)
            {
                readParameter<float>(fSettings, "Camera1.k3", found, false);
                if (found)
                {
                    vPinHoleDistorsion1_.resize(5);
                    vPinHoleDistorsion1_[4] = readParameter<float>(fSettings, "Camera1.k3", found);
                }
                else
                {
                    vPinHoleDistorsion1_.resize(4);
                }
                vPinHoleDistorsion1_[0] = readParameter<float>(fSettings, "Camera1.k1", found);
                vPinHoleDistorsion1_[1] = readParameter<float>(fSettings, "Camera1.k2", found);
                vPinHoleDistorsion1_[2] = readParameter<float>(fSettings, "Camera1.p1", found);
                vPinHoleDistorsion1_[3] = readParameter<float>(fSettings, "Camera1.p2", found);
            }

            // Check if we need to correct distortion from the images
            if ((sensor_ == System::MONOCULAR || sensor_ == System::IMU_MONOCULAR) && vPinHoleDistorsion1_.size() != 0)
            {
                bNeedToUndistort_ = true;
            }
        }
        else if (cameraModel == "Rectified")
        {
            cameraType_ = Rectified;

            // Read intrinsic parameters
            float fx = readParameter<float>(fSettings, "Camera1.fx", found);
            float fy = readParameter<float>(fSettings, "Camera1.fy", found);
            float cx = readParameter<float>(fSettings, "Camera1.cx", found);
            float cy = readParameter<float>(fSettings, "Camera1.cy", found);

            vCalibration = {fx, fy, cx, cy};

            calibration1_ = new Pinhole(vCalibration);
            originalCalib1_ = new Pinhole(vCalibration);

            // Rectified images are assumed to be ideal PinHole images (no distortion)
        }
        else if (cameraModel == "KannalaBrandt8")
        {
            cameraType_ = KannalaBrandt;

            // Read intrinsic parameters
            float fx = readParameter<float>(fSettings, "Camera1.fx", found);
            float fy = readParameter<float>(fSettings, "Camera1.fy", found);
            float cx = readParameter<float>(fSettings, "Camera1.cx", found);
            float cy = readParameter<float>(fSettings, "Camera1.cy", found);

            float k0 = readParameter<float>(fSettings, "Camera1.k1", found);
            float k1 = readParameter<float>(fSettings, "Camera1.k2", found);
            float k2 = readParameter<float>(fSettings, "Camera1.k3", found);
            float k3 = readParameter<float>(fSettings, "Camera1.k4", found);

            vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

            calibration1_ = new KannalaBrandt8(vCalibration);
            originalCalib1_ = new KannalaBrandt8(vCalibration);

            if (sensor_ == System::STEREO || sensor_ == System::IMU_STEREO)
            {
                int colBegin = readParameter<int>(fSettings, "Camera1.overlappingBegin", found);
                int colEnd = readParameter<int>(fSettings, "Camera1.overlappingEnd", found);
                vector<int> vOverlapping = {colBegin, colEnd};

                static_cast<KannalaBrandt8 *>(calibration1_)->mvLappingArea = vOverlapping;
            }
        }
        else
        {
            cerr << "Error: " << cameraModel << " not known" << endl;
            exit(-1);
        }