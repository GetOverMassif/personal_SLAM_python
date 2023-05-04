import cv2
import sys, os
from dataclasses import dataclass

from ORB_SLAM3.Tools import errorPrint
from ORB_SLAM3.Viewer import Viewer
from ORB_SLAM3.Settings import Settings
from ORB_SLAM3.ORBVocabulary import ORBVocabulary
from ORB_SLAM3.KeyFrameDatabase import KeyFrameDatabase
from ORB_SLAM3.Atlas import Atlas
from ORB_SLAM3.FrameDrawer import FrameDrawer
from ORB_SLAM3.MapDrawer import MapDrawer
from ORB_SLAM3.Tracking import Tracking

# @dataclass
class Verbose:
    th = None
    def PrintMess(self, str, lev):
        if lev <= self.th:
            print(str)
    
    def SetTh(self, _th):
        self.th = _th

@dataclass
class System:
    MONOCULAR=0
    STEREO=1
    RGBD=2
    IMU_MONOCULAR=3
    IMU_STEREO=4
    IMU_RGBD=5

    TEXT_FILE=0
    BINARY_FILE=1

    mSensor: int = None
    mpViewer: Viewer = None
    # mpViewer
    
    def __init__(self, strVocFile, strSettingsFile, sensor, bUseViewer, initFr, strSequence):
        """ Step 1: Init some member params """
        self.mSensor = sensor
        # self.mpViewer = Viewer()
        self.mbReset = False
        self.mbResetActiveMap = False
        self.mbActivateLocalizationMode = False
        self.mbDeactivateLocalizationMode = False
        self.mbShutDown = False
        self.mStrLoadAtlasFromFile = ""

        # Output welcome message
        print(f"\nORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.", \
              f"\nORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.", \
              f"\nThis program comes with ABSOLUTELY NO WARRANTY", \
              f"\nThis is free software, and you are welcome to redistribute it", \
              f"\nunder certain conditions. See LICENSE.txt.\n")
        print(f"Input sensor was set to: ",end="")

        modeDict = {self.MONOCULAR:"Monocular", self.STEREO:"Stereo", self.RGBD:"RGB-D", \
                    self.IMU_MONOCULAR:"Monocular-Inertial", self.IMU_STEREO:"Stereo-Inertial", self.IMU_RGBD:"RGB-D-Inertial"}
        print(f"{modeDict[self.mSensor]}")

        # Step 2: Check settings file
        fsSettings = cv2.FileStorage(strSettingsFile, cv2.FileStorage_READ)
        if not fsSettings.isOpened():
            errorPrint(f"Failed to open settings file at: strSettingsFile")
            sys.exit(-1)

        # 查看配置文件版本，不同版本有不同处理方法
        node = fsSettings.getNode("File.version")
        if not node.empty() and node.isString() and node.string() == "1.0":
            print("Create Settings")
            settings_ = Settings(strSettingsFile,self.mSensor)

            # 保存及加载地图的名字
            self.mStrLoadAtlasFromFile = settings_.atlasLoadFile()
            self.mStrSaveAtlasToFile = settings_.atlasSaveFile()
            # cout << (*settings_) << endl

        else:
            settings_ = None
            node = fsSettings.getNode("System.LoadAtlasFromFile")
            if not node.empty() and node.isString():
                self.mStrLoadAtlasFromFile = str(node)

            node = fsSettings.getNode("System.SaveAtlasToFile")
            if not node.empty() and node.isString():
                self.mStrSaveAtlasToFile = str(node)

        # 是否激活回环，默认是开着的
        node = fsSettings.getNode("loopClosing")
        activeLC = True
        if not node.empty():
            activeLC = node.int() != 0

        self.mStrVocabularyFilePath = strVocFile

        # ORBSLAM3新加的多地图管理功能，这里加载Atlas标识符
        loadedAtlas = False

        if self.mStrLoadAtlasFromFile.isspace():
            #Load ORB Vocabulary
            print("\nLoading ORB Vocabulary. This could take a while...\n",end="")

            # 建立一个新的ORB字典
            self.mpVocabulary = ORBVocabulary()
            # 读取预训练好的ORB字典并返回成功/失败标志
            bVocLoad = mpVocabulary.loadFromTextFile(strVocFile)
            # 如果加载失败，就输出错误信息
            if not bVocLoad:
                sys.stderr.write("Wrong path to vocabulary. \n")
                sys.stderr.write("Falied to open at: {strVocFile}\n")
                sys.exit(-1)
            print("Vocabulary loaded!\n\n",end="")

            #Create KeyFrame Database
            # Step 4 创建关键帧数据库
            self.mpKeyFrameDatabase = KeyFrameDatabase(self.mpVocabulary)

            #Create the Atlas
            # Step 5 创建多地图，参数0表示初始化关键帧id为0
            print("Initialization of Atlas from scratch \n", end="")
            self.mpAtlas = Atlas(0)
        else:
            #Load ORB Vocabulary
            print("\nLoading ORB Vocabulary. This could take a while...\n",end="")

            mpVocabulary = ORBVocabulary()
            bVocLoad = mpVocabulary.loadFromTextFile(strVocFile)
            if not bVocLoad:
                sys.stderr.write("Wrong path to vocabulary. \n")
                sys.stderr.write(f"Falied to open at: {strVocFile}\n")
                sys.exit(-1)
            print("Vocabulary loaded!\n\n",end="")

            #Create KeyFrame Database
            mpKeyFrameDatabase = KeyFrameDatabase(mpVocabulary)
            print("Load File\n")

            # Load the file with an earlier session
            #clock_t start = clock()
            print(f"Initialization of Atlas from file: {self.mStrLoadAtlasFromFile}\n",end="")
            isRead = self.LoadAtlas(self.BINARY_FILE)

            if not isRead:
                print("Error to load the file, please try with other session file or vocabulary file")
                sys.exit(-1)
            #mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary)


            #cout << "KF in DB: " << mpKeyFrameDatabase.mnNumKFs << " words: " << mpKeyFrameDatabase.mnNumWords << endl

            loadedAtlas = True

            self.mpAtlas.CreateNewMap()

            #clock_t timeElapsed = clock() - start
            #unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000)
            #cout << "Binary file read in " << msElapsed << " ms" << endl

            #usleep(10*1000*1000)

        # 如果是有imu的传感器类型，设置mbIsInertial = True以后的跟踪和预积分将和这个标志有关
        if self.mSensor==self.IMU_STEREO or self.mSensor==self.IMU_MONOCULAR or self.mSensor==self.IMU_RGBD:
            self.mpAtlas.SetInertialSensor()

        # Step 6 依次创建跟踪、局部建图、闭环、显示线程
        #Create Drawers. These are used by the Viewer
        # 创建用于显示帧和地图的类，由Viewer调用
        self.mpFrameDrawer = FrameDrawer(self.mpAtlas)
        self.mpMapDrawer = MapDrawer(self.mpAtlas, strSettingsFile, settings_)

        #Initialize the Tracking thread
        #(it will live in the main thread of execution, the one that called this constructor)
        # 创建跟踪线程（主线程）,不会立刻开启,会在对图像和imu预处理后在main主线程种执行
        print(f"Seq. Name: {strSequence}")
        self.mpTracker = Tracking(self, mpVocabulary, self.mpFrameDrawer, self.mpMapDrawer,
                                self.mpAtlas, mpKeyFrameDatabase, strSettingsFile, self.mSensor, settings_, strSequence)

        # #Initialize the Local Mapping thread and launch
        # #创建并开启local mapping线程
        # mpLocalMapper = new LocalMapping(this, mpAtlas, self.mSensor==MONOCULAR or self.mSensor==IMU_MONOCULAR,
        #                                 self.mSensor==IMU_MONOCULAR or self.mSensor==IMU_STEREO or self.mSensor==IMU_RGBD, strSequence)
        # mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run,mpLocalMapper)
        # mpLocalMapper.mInitFr = initFr

        # # 设置最远3D地图点的深度值，如果超过阈值，说明可能三角化不太准确，丢弃
        # if(settings_)
        #     mpLocalMapper.mThFarPoints = settings_.thFarPoints()
        # else
        #     mpLocalMapper.mThFarPoints = fsSettings["thFarPoints"]
        # # ? 这里有个疑问,C++中浮点型跟0比较是否用精确?
        # if(mpLocalMapper.mThFarPoints!=0)
        # {
        #     cout << "Discard points further than " << mpLocalMapper.mThFarPoints << " m from current camera" << endl
        #     mpLocalMapper.mbFarPoints = True
        # }
        # else
        #     mpLocalMapper.mbFarPoints = False

        # #Initialize the Loop Closing thread and launch
        # # mSensor!=MONOCULAR and mSensor!=IMU_MONOCULAR
        # # 创建并开启闭环线程
        # mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR, activeLC) # mSensor!=MONOCULAR)
        # mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser)

        # #Set pointers between threads
        # # 设置线程间的指针
        # mpTracker.SetLocalMapper(mpLocalMapper)
        # mpTracker.SetLoopClosing(mpLoopCloser)

        # mpLocalMapper.SetTracker(mpTracker)
        # mpLocalMapper.SetLoopCloser(mpLoopCloser)

        # mpLoopCloser.SetTracker(mpTracker)
        # mpLoopCloser.SetLocalMapper(mpLocalMapper)

        # #usleep(10*1000*1000)

        # #Initialize the Viewer thread and launch
        # # 创建并开启显示线程
        # if(bUseViewer)
        # #if(False) # TODO
        # {
        #     mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile,settings_)
        #     mptViewer = new thread(&Viewer::Run, mpViewer)
        #     mpTracker.SetViewer(mpViewer)
        #     mpLoopCloser.mpViewer = mpViewer
        #     mpViewer.both = mpFrameDrawer.both
        # }

        # # Fix verbosity
        # # 打印输出中间的信息，设置为安静模式
        # Verbose::SetTh(Verbose::VERBOSITY_QUIET)


    def TrackMonocular(im, timestamp, vImuMeas, filename=""):
        print(f"hello")
    
    def LoadAtlas(self, type):
        # # 1. 加载地图文件
        # # string strFileVoc, strVocChecksum
        # isRead = False

        # pathLoadFileName = "./"
        # pathLoadFileName = pathLoadFileName.append(self.mStrLoadAtlasFromFile)
        # pathLoadFileName = pathLoadFileName.append(".osa")

        # if type == self.TEXT_FILE: # File text
        #     print("Starting to read the save text file ")

        #     std::ifstream ifs(pathLoadFileName, std::ios::binary)
        #     if not ifs.good():
        #         print("Load file not found")
        #         return False
        #     boost::archive::text_iarchive ia(ifs)
        #     ia >> strFileVoc
        #     ia >> strVocChecksum
        #     ia >> mpAtlas
        #     print("End to load the save text file ")
        #     isRead = True
        # elif type == self.BINARY_FILE: # File binary
        #     print("Starting to read the save binary file")
        #     std::ifstream ifs(pathLoadFileName, std::ios::binary)
        #     if not ifs.good():
        #         cout << "Load file not found" << endl
        #         return False
        #     boost::archive::binary_iarchive ia(ifs)
        #     ia >> strFileVoc
        #     ia >> strVocChecksum
        #     ia >> mpAtlas
        #     print("End to load the save binary file")
        #     isRead = True

        # # 2. 如果加载成功
        # if isRead:
        #     #Check if the vocabulary is the same
        #     # 校验词典是否一样
        #     strInputVocabularyChecksum = CalculateCheckSum(self.mStrVocabularyFilePath,self.TEXT_FILE)

        #     if strInputVocabularyChecksum.compare(strVocChecksum) != 0:
        #         print("The vocabulary load isn't the same which the load session was created ")
        #         print(f"-Vocabulary name: {strFileVoc}")
        #         return False # Both are differents

        #     # 加载对应数据
        #     self.mpAtlas.SetKeyFrameDababase(mpKeyFrameDatabase)
        #     self.mpAtlas.SetORBVocabulary(mpVocabulary)
        #     self.mpAtlas.PostLoad()
        #     return True
        # return False
        return True
