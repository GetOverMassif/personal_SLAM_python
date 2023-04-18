#include "ros/ros.h"
#include <k4a/k4a.h>
#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;

#define DEBUG_std_cout 0

int main(int argc,char** argv)
{
    ros::init(argc, argv, "configure_kinect_dk_node");
    ros::NodeHandle n;

    const uint32_t device_count = k4a_device_get_installed_count();
	if (device_count == 0)	
	{
		std::cout << "Error: no K4A devices found. " << std::endl;
		return EXIT_FAILURE;
	}
	else
	{
		std::cout << "Found " << device_count << " connected devices. " << std::endl;
 
		if (device_count != 1)// 超过1个设备，也输出错误信息。
		{
			std::cout << "Error: more than one K4A devices found. " << std::endl;
			return EXIT_FAILURE;
		}
		else// 该示例代码仅限对1个设备操作
		{
			std::cout << "Done: found 1 K4A device. " << std::endl;
		}		
	}
	
    // 打开（默认）设备
    k4a_device_t device = NULL;
    if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device)))
    {
        printf("Failed to open k4a device!\n");
        return 1;
    }
 
    // Get the size of the serial number
    size_t serial_size = 0;
    k4a_device_get_serialnum(device, NULL, &serial_size);

    // Allocate memory for the serial, then acquire it
    char *serial = (char*)(malloc(serial_size));
    k4a_device_get_serialnum(device, serial, &serial_size);
    printf("Opened device: %s\n", serial);
    free(serial);
 
	/*
		检索 Azure Kinect 图像数据
	*/
	// 配置并启动设备（帧率、颜色格式、分辨率、深度模式，以及是否只同步图像）
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
	config.synchronized_images_only = true;// ensures that depth and color images are both available in the capture
 
    if (K4A_FAILED(k4a_device_start_cameras(device, &config)))
    {
        printf("Failed to start cameras!\n");
        k4a_device_close(device);
        return 1;
    }
    printf("Successfully started the camera!\n");

	// 稳定化
	k4a_capture_t capture;
	// int iAuto = 0;//用来稳定，类似自动曝光
	// int iAutoError = 0;// 统计自动曝光的失败次数

    // Capture a depth frame
    switch (k4a_device_get_capture(device, &capture, 1000))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
        printf("Successfully get a capture\n");
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        printf("Timed out waiting for a capture\n");
        // continue;
        break;
    case K4A_WAIT_RESULT_FAILED:
        printf("Failed to read a capture\n");
        break;
    }
    

    k4a_capture_release(capture);
    cout << "flag1" << endl;
    k4a_image_t image = k4a_capture_get_depth_image(capture);
    cout << "flag2" << endl;
    
    if (image != NULL)
    {
        printf(" | Depth16 res:%4dx%4d stride:%5d\n",
                k4a_image_get_height_pixels(image),
                k4a_image_get_width_pixels(image),
                k4a_image_get_stride_bytes(image));

        // Release the image
        k4a_image_release(image);
    }

    // Release the capture
    k4a_capture_release(capture);


// 	while (true)
// 	{
// 		if (device.get_capture(&capture))
// 		{
// 			std::cout << iAuto << ". Capture several frames to give auto-exposure" << std::endl;
 
// 			// 跳过前 n 个（成功的数据采集）循环，用来稳定
// 			if (iAuto != 30)
// 			{
// 				iAuto++;
// 				continue;
// 			}
// 			else
// 			{
// 				std::cout << "Done: auto-exposure" << std::endl;
// 				break;// 跳出该循环，完成相机的稳定过程
// 			}
// 		}
// 		else
// 		{
// 			std::cout << iAutoError << ". K4A_WAIT_RESULT_TIMEOUT." << std::endl;
// 			if (iAutoError != 30)
// 			{
// 				iAutoError++;
// 				continue;
// 			}
// 			else
// 			{
// 				std::cout << "Error: failed to give auto-exposure. " << std::endl;
// 				return EXIT_FAILURE;
// 			}
// 		}
// 	}
// 	std::cout << "-----------------------------------" << std::endl;
// 	std::cout << "----- Have Started Kinect DK. -----" << std::endl;
// 	std::cout << "-----------------------------------" << std::endl;
 
 
// 	// 从设备获取捕获
// 	k4a::image rgbImage;
// 	k4a::image depthImage;
// 	k4a::image irImage;
 
// 	cv::Mat cv_rgbImage_with_alpha;
// 	cv::Mat cv_rgbImage_no_alpha;
// 	cv::Mat cv_depth;
// 	cv::Mat cv_depth_8U;
// 	cv::Mat cv_irImage;
// 	cv::Mat cv_irImage_8U;
 
 
// 	while (true)
// 	// for (size_t i = 0; i < 100; i++)
// 	{
// 		// if (device.get_capture(&capture, std::chrono::milliseconds(0)))
// 		if (device.get_capture(&capture))
// 		{
// 			// rgb
// 			// * Each pixel of BGRA32 data is four bytes. The first three bytes represent Blue, Green,
//      		// * and Red data. The fourth byte is the alpha channel and is unused in the Azure Kinect APIs.
// 			rgbImage = capture.get_color_image();
// #if DEBUG_std_cout == 1
// 			std::cout << "[rgb] " << "\n"
// 				<< "format: " << rgbImage.get_format() << "\n"
// 				<< "device_timestamp: " << rgbImage.get_device_timestamp().count() << "\n"
// 				<< "system_timestamp: " << rgbImage.get_system_timestamp().count() << "\n"
// 				<< "height*width: " << rgbImage.get_height_pixels() << ", " << rgbImage.get_width_pixels() 
// 				<< std::endl;
// #endif
// 			cv_rgbImage_with_alpha = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4, (void *)rgbImage.get_buffer());
// 			cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);
 
// 			// depth
// 			// * Each pixel of DEPTH16 data is two bytes of little endian unsigned depth data. The unit of the data is in
//      		// * millimeters from the origin of the camera.
// 			depthImage = capture.get_depth_image();
// #if DEBUG_std_cout == 1
// 			std::cout << "[depth] " << "\n"
// 				<< "format: " << depthImage.get_format() << "\n"
// 				<< "device_timestamp: " << depthImage.get_device_timestamp().count() << "\n"
// 				<< "system_timestamp: " << depthImage.get_system_timestamp().count() << "\n"
// 				<< "height*width: " << depthImage.get_height_pixels() << ", " << depthImage.get_width_pixels() 
// 				<< std::endl;
// #endif
// 			cv_depth = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U, (void *)depthImage.get_buffer(), static_cast<size_t>(depthImage.get_stride_bytes()));
// 			cv_depth.convertTo(cv_depth_8U, CV_8U, 1 );
			
// 			// ir
// 			// * Each pixel of IR16 data is two bytes of little endian unsigned depth data. The value of the data represents
//      		// * brightness.
// 			irImage = capture.get_ir_image();
// #if DEBUG_std_cout == 1
// 			std::cout << "[ir] " << "\n"
// 				<< "format: " << irImage.get_format() << "\n"
// 				<< "device_timestamp: " << irImage.get_device_timestamp().count() << "\n"
// 				<< "system_timestamp: " << irImage.get_system_timestamp().count() << "\n"
// 				<< "height*width: " << irImage.get_height_pixels() << ", " << irImage.get_width_pixels() 
// 				<< std::endl;
// #endif
// 			cv_irImage = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_16U, (void *)irImage.get_buffer(), static_cast<size_t>(irImage.get_stride_bytes()));
// 			cv_irImage.convertTo(cv_irImage_8U, CV_8U, 1 );
 
// 			// show image
// 			cv::imshow("color", cv_rgbImage_no_alpha);
// 			cv::imshow("depth", cv_depth_8U);
// 			cv::imshow("ir", cv_irImage_8U);
// 			cv::waitKey(1);
 
// 			std::cout << "--- test ---" << std::endl;
// 		}
// 		else
// 		{
// 			std::cout << "false: K4A_WAIT_RESULT_TIMEOUT." << std::endl;
// 		}
// 	}
 
// 	cv::destroyAllWindows();
 
// 	// 释放，关闭设备
// 	rgbImage.reset();
// 	depthImage.reset();
// 	irImage.reset();


	
 
 
	// ---------------------------------------------------------------------------------------------------------
	/*
				Test
	*/
	// 等待输入，方便显示上述运行结果
	// std::cout << "--------------------------------------------" << std::endl;
	// std::cout << "Waiting for inputting an integer: ";
	// int wd_wait;
	// std::cin >> wd_wait;
 
	// std::cout << "----------------------------------" << std::endl;
	// std::cout << "------------- closed -------------" << std::endl;
	// std::cout << "----------------------------------" << std::endl;
 
// ————————————————
// 版权声明：本文为CSDN博主「denkywu」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
// 原文链接：https://blog.csdn.net/denkywu/article/details/103305714

    k4a_device_stop_cameras(device);
    k4a_device_close(device);
    return 0;
}