//
// Created by taka on 2020/09/12.
//

#include "MVGE134GCT.hpp"

void CameraCallback(CameraHandle hCamera, BYTE *pFrameBuffer,
					tSdkFrameHead *pFrameHead, PVOID pContext)
{
	CameraDevice *c = (CameraDevice *)pContext;
	CameraImageProcess(hCamera, pFrameBuffer, c->g_pRgbBuffer, pFrameHead);
	cv::Mat matImage(
		pFrameHead->iHeight, pFrameHead->iWidth,
		pFrameHead->uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
		c->g_pRgbBuffer);
	c->imageBuff.push_front(matImage.clone());
}
CameraDevice::CameraDevice()
{
	imageBuff.resize(2);
	CameraSdkInit(1);
	//枚举设备，并建立设备列表(デバイスを列挙し、デバイスのリストを作成する)
	iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
	
	printf("state = %d\n", iStatus);
    ROS_INFO("state = %d",iStatus);
	printf("count = %d\n", iCameraCounts);
    ROS_INFO("count= %d",iStatus);
	//没有连接设备(接続されたデバイスがない)
	if (iCameraCounts == 0)
	{
    	ROS_ERROR("Not find camera");
		exit(-1);
	}

	//相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口(カメラの初期化。
	//他のカメラ関連の操作インターフェースを呼び出す前に、初期化が成功していることを確認してください。)
	iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

	//初始化失败(初期化失敗)
    ROS_INFO("state = %d",iStatus);
	if (iStatus != CAMERA_STATUS_SUCCESS)
	{
    	ROS_ERROR("Failed Initialize");
		exit(-1);
	}

	//获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
	//(カメラの特徴記述構造を取得します．
	//この構造体には、カメラで設定できる様々なパラメータの範囲に関する情報が含まれています。
	//関連する関数のパラメータを決定します。)
	CameraGetCapability(hCamera, &tCapability);
	g_pRgbBuffer =
		(unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax *
								tCapability.sResolutionRange.iWidthMax * 3);
//    g_pRgbBuffer =
//            (unsigned char *)malloc(600*
//                                    800* 3);
    ROS_INFO("%d", tCapability.iFrameSpeedDesc);
    ROS_INFO("HeightMax %d", tCapability.sResolutionRange.iHeightMax);
    ROS_INFO("HeightMin %d", tCapability.sResolutionRange.iHeightMin);
    ROS_INFO("WidthMax  %d", tCapability.sResolutionRange.iWidthMax);
    ROS_INFO("WidthMin  %d", tCapability.sResolutionRange.iWidthMin);
    //カメラのパラメータ設定
    // TODO:外部ファイルからカメラの設定を決める
    double time;
	int gain;
	//自動露光を無効化
	if (CameraSetAeState(hCamera, false)!= CAMERA_STATUS_SUCCESS)
	{
		ROS_ERROR("Failed Set Manual Exposure time.");
	}
	//露光時間を指定(us)
	if (CameraSetExposureTime(hCamera, 50 * 1000) != CAMERA_STATUS_SUCCESS)
	{
		ROS_ERROR("Failed Set Exposure time.");
	}
	//ゲインの設定(0~33の間, 計算式は不明)
	if (CameraSetAnalogGain(hCamera, 33)!= CAMERA_STATUS_SUCCESS)
	{
		ROS_ERROR("Failed Set Analog Gain.");
	}
	CameraGetExposureTime(hCamera, &time);
	CameraGetAnalogGain(hCamera, &gain);
	ROS_INFO("exposure time%f[ms]:Gain%d", time / 1000, gain);

	/*让SDK进入工作模式，开始接收来自相机发送的图像
	数据。如果当前相机是触发模式，则需要接收到
	触发帧以后才会更新图像。    */
	//カメラからの画像取得をスタート
	CameraPlay(hCamera);
	/*其他的相机参数设置
	例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
		 CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
		 CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
		 更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
	*/
	CameraGetImageResolution(hCamera,&sImageInfo);
	std::cout<<sImageInfo.iWidth<<"x"<<sImageInfo.iHeight<<std::endl;
	sImageInfo.iHeight = 600;
    sImageInfo.iWidth = 800;
    if (CameraSetImageResolution(hCamera,&sImageInfo) != CAMERA_STATUS_SUCCESS)
    {
	    ROS_ERROR("Failed Set Image Size");
    }
    CameraGetImageResolution(hCamera,&sImageInfo);
	ROS_INFO("%d x %d", sImageInfo.iWidth, sImageInfo.iHeight);
	if (tCapability.sIspCapacity.bMonoSensor)
	{
		channel = 1;
		CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
	}
	else
	{
		channel = 3;
		CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
	}
	if (CameraSetCallbackFunction(hCamera, CameraCallback, this, nullptr) != CAMERA_STATUS_SUCCESS)
	{
	    ROS_ERROR("Failed Set CallBack");
	}
}
CameraDevice::~CameraDevice()
{
	CameraUnInit(hCamera);
	//注意，现反初始化后再free
	free(g_pRgbBuffer);
}

bool CameraDevice::GetFrame(cv::Mat &img)
{
	if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 500) ==
		CAMERA_STATUS_SUCCESS)
	{
		if (iplImage)
		{
			cvReleaseImageHeader(&iplImage);
		}
		CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
		//        CameraPushFrame(hCamera, g_pRgbBuffer, &sFrameInfo);
		//画像を保存できる
		// CameraSaveImage(hCamera, (char *)"./test",g_pRgbBuffer, &sFrameInfo,
		// FILE_BMP, 100);
		//        std::cout << sFrameInfo.iWidth << ", " << sFrameInfo.iHeight
		//        << std::endl;
		iplImage = cvCreateImageHeader(
			cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight), IPL_DEPTH_8U, 3);
		//        cv::Mat matImage(
		//                sFrameInfo.iHeight,
		//                sFrameInfo.iWidth,
		//                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ?
		//                CV_8UC1 : CV_8UC3,
		//				pbyBuffer
		//        );
		cvSetData(iplImage, g_pRgbBuffer, sFrameInfo.iWidth * 3);
		img = cv::cvarrToMat(iplImage).clone();
		if (!img.empty())
		{
			// cv::imshow("rawImage", img);
			// cv::waitKey(1);
			//            img = matImage.clone();
			//            writer->write(matImage);
			//			CameraGetFrameRate(hCameqra,&frameSpeed);
			//			std::cout<<frameSpeed<<std::endl;
			//            cv::imshow("camera", matImage);
			//            cv::waitKey(1);

			//在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
			//否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
			CameraReleaseImageBuffer(hCamera, pbyBuffer);
			return TRUE;
		}
	}
	else
	{
		//画像を読み込めなかったときは空の画像を生成
		img = cv::Mat();
	}
	return FALSE;
}
bool CameraDevice::PopFrame(cv::Mat &img)
{
	// timer.Start();
	while (imageBuff.empty()) {
		// timer.Stop();
		// if(timer.GetTime(ProcessingTimer::Unit::MILLI_SECONDS)>100)
		{
            img = cv::Mat();
			return false;
		}
	}
	img = imageBuff.front().clone();
	imageBuff.pop_front();
	return true;
}