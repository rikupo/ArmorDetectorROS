#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "armor.hpp"
class ArmorDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher targets_pub_;
  Color enemyColor_;
  int GRAYTH_;
  int COLORTH_;
  //画像を膨張させるときに使うカーネル
  cv::Mat element77 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
  cv::Mat element55 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::Mat element33 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  bool ROIFLAG;
  //! ROI領域
  cv::Rect2f roiArea;
  std_msgs::Float32MultiArray targets_msg_;

public:
  ArmorDetector() : it_(nh_)
  {
    image_sub_ = it_.subscribe("/img_publisher/image", 1, &ArmorDetector::imageCallback, this);
    image_pub_ = it_.advertise("/armor_detector/output_video", 1);
    targets_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("targets", 1);
    enemyColor_ = Color::blue;
    GRAYTH_ = 105;
    COLORTH_ = 82;
    int ROIFLAG = 0;
  }
  ~ArmorDetector()
  {
  }
  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    cv::Mat image;
    try
    {
      image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    ros::WallTime callbackBegin = ros::WallTime::now();
    if (ROIFLAG > 0)
    {
      ROS_INFO("ROI\n");
      roiArea.x = 400;
      roiArea.y = 400;
      roiArea.width = 400;
      roiArea.height = 400;
      cv::Rect roi(roiArea);
      image = image(roi);
    }
    // targets_msg_ = detect2(image);
    detect(image, targets_msg_);
    ros::WallTime callbackEnd = ros::WallTime::now();
    ros::WallDuration callbackTimeDiff = callbackEnd - callbackBegin;
    ROS_INFO("get image\n");
    ROS_INFO("DetectArmor:%u.%09u\n", callbackTimeDiff.sec, callbackTimeDiff.nsec);
    sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    //認識結果の画像をパブリッシュ
    image_pub_.publish(msg_);
    //認識した装甲板の座標群をパブリッシュ
    targets_pub_.publish(targets_msg_);
  }
  double lw_rate(const cv::RotatedRect &rect)
  {
    return rect.size.height > rect.size.width ? rect.size.height / rect.size.width : rect.size.width / rect.size.height;
  }

  double areaRatio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect)
  {
    return cv::contourArea(contour) / rect.size.area();
  }
  bool isValidLightBar(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect)
  {
    return (1.2 < lw_rate(rect) && lw_rate(rect) < 5) && ((rect.size.area() < 50 && areaRatio(contour, rect) > 0.4) ||
                                                          (rect.size.area() >= 50 && areaRatio(contour, rect) > 0.6));
  }
  void detect(cv::Mat &inputImg, std_msgs::Float32MultiArray &array)
  {
    int size_of_cloud = 4;
    float x = 189.2, y = 78.2, z = 0;
    array.data.resize(3);
    array.data[0] = x;
    array.data[1] = y;
    array.data[2] = z;
  }
  void detect(cv::Mat &inputImg)
  {
    cv::Mat grayImg, colorImg, tempImg, binaryImg;
    std::vector<cv::Mat> bgr;
    std::vector<LightBar> LightBarVector;
    //バイナリー画像から輪郭抽出
    std::vector<std::vector<cv::Point>> contours;
    // BGR画像からグレースケール画像を生成
    cv::cvtColor(inputImg, grayImg, cv::COLOR_BGR2GRAY);
    // BGR画像からB,G,Rを成分ごとに分解
    cv::split(inputImg, bgr);
    if (enemyColor_ == Color::red)
    {
      subtract(bgr[2], bgr[0], colorImg);
    }
    else
    {
      subtract(bgr[0], bgr[2], colorImg);
    }
    cv::threshold(grayImg, grayImg, GRAYTH_, 255, cv::THRESH_BINARY);
    cv::threshold(colorImg, colorImg, COLORTH_, 255, cv::THRESH_BINARY);
    binaryImg = colorImg & grayImg;
    //    cv::dilate(binaryImg_,resultImg_,element55);
    //    cv::medianBlur(binaryImg_, tempImg, 3);
    cv::morphologyEx(binaryImg, binaryImg, cv::MORPH_OPEN, element55);
    cv::findContours(binaryImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    for (size_t i = 0; i < contours.size(); i++)
    {
      double aspectRatio = 0;
      double area = cv::contourArea(contours[i]);
      cv::RotatedRect rRect = cv::fitEllipse(contours[i]);
      cv::Point2f rectPoint[4];
      rRect.points(rectPoint);
      if (area > 10)
      {
        for (size_t j1 = 0; j1 < 4; j1++)
        {
          //検出した領域を紫直線で囲む
          cv::line(inputImg, rectPoint[j1], rectPoint[(j1 + 1) % 4], cv::Scalar(255, 0, 255), 2);
        }
        if (isValidLightBar(contours[i], rRect))
        {
          for (size_t j2 = 0; j2 < 4; j2++)
          {
            //ライトバー領域を青直線で囲む
            cv::line(inputImg, rectPoint[j2], rectPoint[(j2 + 1) % 4], cv::Scalar(255, 255, 0), 2);
          }
          LightBar r(rRect);
          LightBarVector.push_back(r);
        }
      }
    }
    for (size_t i = 0; i < LightBarVector.size(); i++)
    {
      for (size_t j = i + 1; j < LightBarVector.size(); j++)
      {
        Armor armor(LightBarVector[i], LightBarVector[j]);
        if (armor.isArmor() == true)
        {
          // armorList.push_back(armor.getArmorCenter());
          armor.drawArmor(inputImg);
        }
      }
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "armor_detector_node");
  ArmorDetector ArmorDetector;
  ros::spin();
  return 0;
}
