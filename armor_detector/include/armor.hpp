/**
 * @file armor.hpp
 * @brief 装甲板やライトバーに関連するクラスを定義
 */
#ifndef PHOENIXVISION_ARMOR_HPP
#define PHOENIXVISION_ARMOR_HPP
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <chrono>
#include <iostream>
#include <fstream>

enum class Color : int
{
    blue,   //0
    red     //1
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
#endif //PHOENIXVISION_ARMOR_HPP