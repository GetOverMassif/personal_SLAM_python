import cv2
import sys
from dataclasses import dataclass

from Viewer import Viewer
from Tools import errorPrint
from Settings import Settings


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

        self.mSensor = sensor
        # self.mpViewer = Viewer()
        self.mbReset = False
        self.mbResetActiveMap = False
        self.mbActivateLocalizationMode = False
        self.mbDeactivateLocalizationMode = False
        self.mbShutDown = False


        # Output welcome message
        print(f"\nORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.", \
              f"\nORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.", \
              f"\nThis program comes with ABSOLUTELY NO WARRANTY", \
              f"\nThis is free software, and you are welcome to redistribute it", \
              f"\nunder certain conditions. See LICENSE.txt.\n")
        print(f"Input sensor was set to: ",end="")

        if self.mSensor==self.MONOCULAR:
            print(f"Monocular")             # 单目
        elif self.mSensor==self.STEREO:
            print(f"Stereo")                # 双目
        elif self.mSensor==self.RGBD:
            print(f"RGB-D")                 # RGBD相机   
        elif self.mSensor==self.IMU_MONOCULAR:
            print(f"Monocular-Inertial")    # 单目 + imu
        elif self.mSensor==self.IMU_STEREO:
            print(f"Stereo-Inertial")       # 双目 + imu
        elif self.mSensor==self.IMU_RGBD:
            print(f"RGB-D-Inertial")        # RGBD相机 + imu

        #Check settings file
        # Step 2 读取配置文件
        fsSettings = cv2.FileStorage(strSettingsFile, cv2.FileStorage_READ)
        # 如果打开失败，就输出错误信息
        if not fsSettings.isOpened():
            errorPrint(f"Failed to open settings file at: strSettingsFile")
            sys.exit(-1)

        # 查看配置文件版本，不同版本有不同处理方法
        node = fsSettings["File.version"]
        if not node.empty() and node.isString() and node.string() == "1.0":
            settings_ = Settings(strSettingsFile,self.mSensor)

            # 保存及加载地图的名字
            mStrLoadAtlasFromFile = settings_.atlasLoadFile()
            mStrSaveAtlasToFile = settings_.atlasSaveFile()
            # cout << (*settings_) << endl

        else:
            settings_ = None
            node = fsSettings["System.LoadAtlasFromFile"]
            if not node.empty() and node.isString():
                mStrLoadAtlasFromFile = str(node)

            node = fsSettings["System.SaveAtlasToFile"]
            if not node.empty() and node.isString():
                mStrSaveAtlasToFile = str(node)

        # # 是否激活回环，默认是开着的
        # node = fsSettings["loopClosing"]
        # activeLC = True
        # if not node.empty():
        #     activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0

        # mStrVocabularyFilePath = strVocFile

        # # ORBSLAM3新加的多地图管理功能，这里加载Atlas标识符
        # loadedAtlas = False

        # if mStrLoadAtlasFromFile.empty():
        # {
        #     #Load ORB Vocabulary
        #     cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl

        #     # 建立一个新的ORB字典
        #     mpVocabulary = new ORBVocabulary()
        #     # 读取预训练好的ORB字典并返回成功/失败标志
        #     bool bVocLoad = mpVocabulary.loadFromTextFile(strVocFile)
        #     # 如果加载失败，就输出错误信息
        #     if(not bVocLoad)
        #     {
        #         cerr << "Wrong path to vocabulary. " << endl
        #         cerr << "Falied to open at: " << strVocFile << endl
        #         exit(-1)
        #     }
        #     cout << "Vocabulary loaded!" << endl << endl

        #     #Create KeyFrame Database
        #     # Step 4 创建关键帧数据库
        #     mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary)

        #     #Create the Atlas
        #     # Step 5 创建多地图，参数0表示初始化关键帧id为0
        #     cout << "Initialization of Atlas from scratch " << endl
        #     mpAtlas = new Atlas(0)
        # }
        # else
        # {
        #     #Load ORB Vocabulary
        #     cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl

        #     mpVocabulary = new ORBVocabulary()
        #     bool bVocLoad = mpVocabulary.loadFromTextFile(strVocFile)
        #     if(not bVocLoad)
        #     {
        #         cerr << "Wrong path to vocabulary. " << endl
        #         cerr << "Falied to open at: " << strVocFile << endl
        #         exit(-1)
        #     }
        #     cout << "Vocabulary loaded!" << endl << endl

        #     #Create KeyFrame Database
        #     mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary)

        #     cout << "Load File" << endl

        #     # Load the file with an earlier session
        #     #clock_t start = clock()
        #     cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << endl
        #     bool isRead = LoadAtlas(FileType::BINARY_FILE)

        #     if(not isRead)
        #     {
        #         cout << "Error to load the file, please try with other session file or vocabulary file" << endl
        #         exit(-1)
        #     }
        #     #mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary)


        #     #cout << "KF in DB: " << mpKeyFrameDatabase.mnNumKFs << " words: " << mpKeyFrameDatabase.mnNumWords << endl

        #     loadedAtlas = true

        #     mpAtlas.CreateNewMap()

        #     #clock_t timeElapsed = clock() - start
        #     #unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000)
        #     #cout << "Binary file read in " << msElapsed << " ms" << endl

        #     #usleep(10*1000*1000)
        # }

        # # 如果是有imu的传感器类型，设置mbIsInertial = true以后的跟踪和预积分将和这个标志有关
        # if (self.mSensor==IMU_STEREO or self.mSensor==IMU_MONOCULAR or self.mSensor==IMU_RGBD)
        #     mpAtlas.SetInertialSensor()

        # # Step 6 依次创建跟踪、局部建图、闭环、显示线程
        # #Create Drawers. These are used by the Viewer
        # # 创建用于显示帧和地图的类，由Viewer调用
        # mpFrameDrawer = new FrameDrawer(mpAtlas)
        # mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_)

        # #Initialize the Tracking thread
        # #(it will live in the main thread of execution, the one that called this constructor)
        # # 创建跟踪线程（主线程）,不会立刻开启,会在对图像和imu预处理后在main主线程种执行
        # cout << "Seq. Name: " << strSequence << endl
        # mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
        #                         mpAtlas, mpKeyFrameDatabase, strSettingsFile, self.mSensor, settings_, strSequence)

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
        #     mpLocalMapper.mbFarPoints = true
        # }
        # else
        #     mpLocalMapper.mbFarPoints = false

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
        # #if(false) # TODO
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
