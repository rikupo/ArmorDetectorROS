//
// Created by taka on 2020/09/12.
//
/**
 * @file camera_device.h
 * @brief 画像処理用カメラに関連するクラスを定義
 */
#ifndef PHOENIXVISION_CAMERA_DEVICE_HPP
#define PHOENIXVISION_CAMERA_DEVICE_HPP
#include <ros/ros.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <boost/circular_buffer.hpp>
#include <queue>
#include <chrono>
#include "CameraApi.h"

class CameraDevice
{
	/**
	 * @brief カメラが画像を取得したら呼ばれる
	 */
	friend void CameraCallback(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead* pFrameHead,PVOID pContext);
  public:
	CameraDevice();
	~CameraDevice();
	/**
	 * @brief デバッグ用の動画から画像を取得
	 * @return 構造体FrameData(cv::Mat,フレーム数)
	 */
	bool GetFrame(cv::Mat& img);
	bool PopFrame(cv::Mat& img);

  private:
	int 				iCameraCounts = 1;
	int 				iStatus = -1;
	tSdkCameraDevInfo 	tCameraEnumList;
	int 				hCamera;
	tSdkCameraCapbility tCapability; //设备描述信息
	tSdkFrameHead 		sFrameInfo;
	tSdkImageResolution sImageInfo;
	BYTE* 				pbyBuffer;
	int 				iDisplayFrames = 10000;
	int 				channel = 3;
	unsigned char* 		g_pRgbBuffer; //处理后数据缓存区
	IplImage* 			iplImage;
	boost::circular_buffer<cv::Mat> imageBuff;
};

#endif // PHOENIXVISION_CAMERA_DEVICE_HPP