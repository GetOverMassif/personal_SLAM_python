import os, sys
MONOCULAR_INTERTIAL_DIR = os.path.dirname(os.path.realpath(__file__))
EXAMPLE_DIR = os.path.dirname(MONOCULAR_INTERTIAL_DIR)
ORB_DIR = os.path.dirname(EXAMPLE_DIR)
print(f"ORB_DIR = {ORB_DIR}")
sys.path.append(ORB_DIR)

import argparse

from ORB_SLAM3.System import System

from ORB_SLAM3.Tools import errorPrint


def LoadImagesTUMVI(strImagePath, strPathTimes, vstrImages, vTimeStamps):
    print(f"strImagePath = {strImagePath}")
    print(f"strPathTimes = {strPathTimes}")
    fTimes = open(strPathTimes, 'r')

    for s in fTimes.readlines():
        if s == "" or s[0]=='#':
            continue
        item = s.split('\n')[0].split(' ')[0]
        vstrImages.append(f"{strImagePath}/{item}.png")
        vTimeStamps.append(float(item)/1e9)
    return vstrImages, vTimeStamps

def LoadIMU(strImuPath, vTimeStamps, vAcc, vGyro):
    print(f"strPathTimes = {strImuPath}")
    fImu = open(strImuPath, 'r')
    for s in fImu.readlines():
        if s == "" or s[0]=='#':
            continue
        data = s.split('\n')[0].split(',')
        vTimeStamps.append(float(data[0])/1e9)
        vAcc.append([float(x) for x in [data[4],data[5],data[6]]])  # 加速度计
        vGyro.append([float(x) for x in [data[1],data[2],data[3]]])  # 重力计

def main():
    path_to_tum_data = f"~/data/TUM-VI/dataset-room4_512_16"
    path_to_vocabulary = f"{ORB_DIR}/Vocabulary/ORBvoc.txt"
    path_to_settings = f"{MONOCULAR_INTERTIAL_DIR}/TUM_512.yaml"
    path_to_image_folder_1 = f"{path_to_tum_data}/mav0/cam0/data"
    path_to_times_file_1 = f"{MONOCULAR_INTERTIAL_DIR}/TUM_TimeStamps/dataset-room4_512.txt"
    path_to_imu_data_1 = f"{MONOCULAR_INTERTIAL_DIR}/TUM_IMU/dataset-room4_512.txt"
    data_clip_name = f"dataset-room4_512_monoi"

# /home/nio/文档/BUAA/personal_SLAM_python/src/orb_slam3
# /home/nio/文档/BUAA/personal_SLAM_python/src/orb_slam3/Vocabulary

    argv = [f"mono_inertial_tum_vi", path_to_vocabulary, path_to_settings, \
            path_to_image_folder_1, path_to_times_file_1, path_to_imu_data_1, \
            data_clip_name, \
    ]
    argc = len(argv)
    num_seq = int((argc - 3) / 3)
    bfile_name = ((argc % 3) == 1)

    file_name = ""
    if (bfile_name):
        file_name = str(argv[argc-1])
    print(f"file name: {file_name}")

    if(argc < 6):
        errorPrint("Usage: ./mono_inertial_tum_vi path_to_vocabulary path_to_settings path_to_image_folder_1 path_to_times_file_1 path_to_imu_data_1 (path_to_image_folder_2 path_to_times_file_2 path_to_imu_data_2 ... path_to_image_folder_N path_to_times_file_N path_to_imu_data_N) (trajectory_file_name)")
        return 1
    
    # load all sequences
    vstrImagefile_names = [[] for i in range(num_seq)]
    vTimestampsCam = [[] for i in range(num_seq)]
    vTimestampsImu = [[] for i in range(num_seq)]
    vAcc = [[] for i in range(num_seq)]
    vGyro = [[] for i in range(num_seq)]
    nImages = [[] for i in range(num_seq)]
    nImu = [[] for i in range(num_seq)]
    first_imu = [0 for i in range(num_seq)]

    tot_images = 0
    for seq in range(num_seq):
        print(f"Loading images for sequence {seq}...")
        LoadImagesTUMVI(argv[3*(seq+1)], (argv[3*(seq+1)+1]), vstrImagefile_names[seq], vTimestampsCam[seq])
        print(f"LOADED!")
        print(f"Loading IMU for sequence {seq}...")
        LoadIMU(str(argv[3*(seq+1)+2]), vTimestampsImu[seq], vAcc[seq], vGyro[seq])
        print(f"LOADED!")

        nImages[seq] = len(vstrImagefile_names[seq])
        tot_images += nImages[seq]
        nImu[seq] = len(vTimestampsImu[seq])

        if nImages[seq]<=0 or nImu[seq]<=0:
            errorPrint(f"ERROR: Failed to load images or IMU for sequence{seq}")
            return 1

        # Find first imu to be considered, supposing imu measurements start first

        while vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][0]:
            first_imu[seq] += 1
        first_imu[seq] -= 1  # first imu measurement to be considered
    
    # Vector for tracking time statistics
    vTimesTrack = [0.0 for i in range(tot_images)]
    print(f"\n-------")
    # cout.precision(17)

    """cout << "Start processing sequence ..." << endl
    cout << "Images in the sequence: " << nImages << endl
    cout << "IMU data in the sequence: " << nImu << endl << endl"""

    # Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM = System(argv[1],argv[2], System.IMU_MONOCULAR, True, 0, file_name)

    # imageScale = SLAM.GetImageScale()

    # imageScale = SLAM.GetImageScale()

    # t_resize, t_track = 0.0, 0.0
    # proccIm = 0
    
#     for seq in range(num_seq):
#         # Main loop
#         cv::Mat im
#         vector<ORB_SLAM3::IMU::Point> vImuMeas
#         proccIm = 0
#         cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8))
        
#         for ni in range(len(nImages[seq])):
#             proccIm += 1
#             # Read image from file
#             im = cv::imread(vstrImagefile_names[seq][ni],cv::IMREAD_GRAYSCALE) #,cv::IMREAD_GRAYSCALE)

#             # clahe
#             clahe->apply(im,im)


#             # cout << "mat type: " << im.type() << endl
#             double tframe = vTimestampsCam[seq][ni]

#             if(im.empty())
#             {
#                 cerr << endl << "Failed to load image at: "
#                      <<  vstrImagefile_names[seq][ni] << endl
#                 return 1
#             }


#             # Load imu measurements from previous frame
#             vImuMeas.clear()

#             if(ni>0)
#             {
#                 # cout << "t_cam " << tframe << endl

#                 while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni])
#                 {
#                     vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
#                                                              vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
#                                                              vTimestampsImu[seq][first_imu[seq]]))
#                     # cout << "t_imu = " << fixed << vImuMeas.back().t << endl
#                     first_imu[seq]++
#                 }
#             }

#             if(imageScale != 1.f)
#             {
# #ifdef REGISTER_TIMES
#     #ifdef COMPILEDWITHC11
#                 std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now()
#     #else
#                 std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now()
#     #endif
# #endif
#                 int width = im.cols * imageScale
#                 int height = im.rows * imageScale
#                 cv::resize(im, im, cv::Size(width, height))
# #ifdef REGISTER_TIMES
#     #ifdef COMPILEDWITHC11
#                 std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now()
#     #else
#                 std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now()
#     #endif
#                 t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count()
#                 SLAM.InsertResizeTime(t_resize)
# #endif
#             }

#             # cout << "first imu: " << first_imu[seq] << endl
#             """cout << "first imu time: " << fixed << vTimestampsImu[first_imu] << endl
#             cout << "size vImu: " << vImuMeas.size() << endl"""
#     #ifdef COMPILEDWITHC11
#             std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now()
#     #else
#             std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now()
#     #endif

#             # Pass the image to the SLAM system
#             # cout << "tframe = " << tframe << endl
#             SLAM.TrackMonocular(im,tframe,vImuMeas) # TODO change to monocular_inertial

#     #ifdef COMPILEDWITHC11
#             std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now()
#     #else
#             std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now()
#     #endif

# #ifdef REGISTER_TIMES
#             t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count()
#             SLAM.InsertTrackTime(t_track)
# #endif

#             double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count()
#             ttrack_tot += ttrack
#             # std::cout << "ttrack: " << ttrack << std::endl

#             vTimesTrack[ni]=ttrack

#             # Wait to load the next frame
#             double T=0
#             if(ni<nImages[seq]-1)
#                 T = vTimestampsCam[seq][ni+1]-tframe
#             else if(ni>0)
#                 T = tframe-vTimestampsCam[seq][ni-1]

#             if(ttrack<T)
#                 usleep((T-ttrack)*1e6) # 1e6

#         }
#         if(seq < num_seq - 1)
#         {
#             cout << "Changing the dataset" << endl

#             SLAM.ChangeDataset()
#         }

#     }

#     # cout << "ttrack_tot = " << ttrack_tot << std::endl
#     # Stop all threads
#     SLAM.Shutdown()


#     # Tracking time statistics

#     # Save camera trajectory

#     if bfile_name:
#         const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt"
#         const string f_file =  "f_" + string(argv[argc-1]) + ".txt"
#         SLAM.SaveTrajectoryEuRoC(f_file)
#         SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file)
#     else:
#         SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt")
#         SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt")

#     sort(vTimesTrack.begin(),vTimesTrack.end())
#     float totaltime = 0
#     for ni in range(nImages[0]):
#         totaltime+=vTimesTrack[ni]
#     print(f"-------\n")
#     print(f"median tracking time: {vTimesTrack[nImages[0]/2]}")
#     print(f"mean tracking time: {totaltime/proccIm}")

#     """const string kf_file =  "kf_" + ss.str() + ".txt"
#     const string f_file =  "f_" + ss.str() + ".txt"

#     SLAM.SaveTrajectoryEuRoC(f_file)
#     SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file)"""

    return 0

if __name__=="__main__":
    main()
    # errorPrint("hello")