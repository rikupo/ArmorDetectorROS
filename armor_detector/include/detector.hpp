/**
 * @file armor_detector.h
 * @brief 装甲板認識に関連するクラスを定義
 */
#ifndef PHOENIXVISION_ARMOR_DETECTOR_HPP
#define PHOENIXVISION_ARMOR_DETECTOR_HPP
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <chrono>
#include <iostream>
#include <fstream>

struct FrameData{
    cv::Mat img_;
    unsigned long int frameNum_;
};

class  LightBar
{
public:
    LightBar():matched(false){}
    LightBar(const cv::RotatedRect& R){
        rect.angle  = R.angle;
        rect.center = R.center;
        rect.size   = R.size;
        matched = false;
        length = std::max(rect.size.height,rect.size.width);
    }

    cv::RotatedRect rect;
    bool matched;
    double length;
    size_t matched_index;
    float match_factor;
};
class Armor
{
public:
    Armor();
    Armor(const LightBar& L1,const LightBar& L2);
    void drawArmor(cv::Mat img);
    double getArmorSize();
    double getErrorAngle();
    cv::Point2f getArmorCenter();
    double getAspectRatio();
    bool isArmor();

private:
    LightBar lightBarR;
    LightBar lightBarL;
    float errorAngle;
    cv::Point2f armorCenter;
    cv::Rect2f armorRect;
    double armorSize;
    double aspectRatio;
    bool isArmor_;

};

class ArmorDetector {
public:
    enum class Color : int
    {
        blue,   //0
        red     //1
    };
    ArmorDetector();
    ~ArmorDetector();
    std::vector<cv::Point2f> DetectArmor(FrameData frame);
	std::vector<cv::Point2f> DetectArmor(cv::Mat &img);
	void SetFrame(FrameData frame);
    cv::Mat GetInputImg();
    cv::Mat GetBinary();
    cv::Mat GetResult();
    void setGrayThreshold(int val);
    void setColorThreshold(int val);
    int getGrayThreshold();
    int getColorThreshold();
    double lw_rate(const cv::RotatedRect &rect);
    double areaRatio(const std::vector<cv::Point> &contour,const cv::RotatedRect &rect);
    bool isValidLightBar(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect);
private:
    std::vector<cv::Point2f> armorList;
    Color enemyColor_;
	cv::Mat img_;
    cv::Mat resultImg_;
    cv::Mat binaryImg_;
    FrameData detectFrame_;
    FrameData preFrame_;
    int GRAYTH_;
    int COLORTH_;
    //画像を膨張させるときに使うカーネル
    cv::Mat element77 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
    cv::Mat element55 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::Mat element33 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
};


#endif //PHOENIXVISION_ARMOR_DETECTOR_HPP
