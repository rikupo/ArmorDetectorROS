#include "detector.hpp"

Armor::Armor(const LightBar &L1, const LightBar &L2)
{
	isArmor_ = false;
	if (L1.rect.center.x > L2.rect.center.x)
	{
		lightBarL = L1;
		lightBarR = L2;
	}
	else if (L1.rect.center.x < L2.rect.center.x)
	{
		lightBarR = L1;
		lightBarL = L2;
	}
	//ライトバー同士の長さの比を計算
	double bothBarLengthRatio = lightBarL.length / lightBarR.length;
	cv::Point2f centers = lightBarL.rect.center - lightBarR.rect.center;
	double distanceRatio = sqrt(centers.ddot(centers)) / lightBarL.length;
	float angle_r = lightBarR.rect.size.width > lightBarR.rect.size.height
						? lightBarR.rect.angle
						: lightBarR.rect.angle - 90;
	float angle_l = lightBarL.rect.size.width > lightBarL.rect.size.height
						? lightBarL.rect.angle
						: lightBarL.rect.angle - 90;
	errorAngle = std::fabs(angle_r - angle_l);
	armorRect.width = std::fabs(L1.rect.center.x - L2.rect.center.x);
	armorRect.height = std::fabs(L1.rect.center.x - L2.rect.center.x) * 2 / 3;
	aspectRatio = armorRect.width / armorRect.height;
	armorSize = armorRect.width * armorRect.height;
	armorCenter.x = (std::fabs(L1.rect.center.x + L2.rect.center.x)) / 2;
	armorCenter.y = (std::fabs(L1.rect.center.y + L2.rect.center.y)) / 2;
	armorRect.x = armorCenter.x - armorRect.width / 2;
	armorRect.y = armorCenter.y - armorRect.height / 2;
	if (errorAngle < 20 && bothBarLengthRatio < 2.5 &&
		bothBarLengthRatio > 0.4 && distanceRatio < 5 && distanceRatio > 0.4)
	{
		isArmor_ = true;
	}
}

void Armor::drawArmor(cv::Mat img)
{
	cv::rectangle(img, armorRect, cv::Scalar(0, 255, 0), 2);
}

double Armor::getErrorAngle()
{
	return errorAngle;
}

double Armor::getArmorSize()
{
	return armorSize;
}

cv::Point2f Armor::getArmorCenter()
{
	return armorCenter;
}

double Armor::getAspectRatio()
{
	return aspectRatio;
}

bool Armor::isArmor()
{
	return isArmor_;
}

ArmorDetector::ArmorDetector()
{
	enemyColor_ = Color::blue;
}

ArmorDetector::~ArmorDetector()
{
}

void ArmorDetector::SetFrame(FrameData frame)
{
	preFrame_.img_ = detectFrame_.img_;
	preFrame_.frameNum_ = detectFrame_.frameNum_;
	detectFrame_.img_ = frame.img_.clone();
	detectFrame_.frameNum_ = frame.frameNum_;
}

double ArmorDetector::lw_rate(const cv::RotatedRect &rect)
{
	return rect.size.height > rect.size.width
			   ? rect.size.height / rect.size.width
			   : rect.size.width / rect.size.height;
}

double ArmorDetector::areaRatio(const std::vector<cv::Point> &contour,
								const cv::RotatedRect &rect)
{
	return cv::contourArea(contour) / rect.size.area();
}

bool ArmorDetector::isValidLightBar(const std::vector<cv::Point> &contour,
									const cv::RotatedRect &rect)
{
	return (1.2 < lw_rate(rect) && lw_rate(rect) < 5) &&
		   ((rect.size.area() < 50 && areaRatio(contour, rect) > 0.4) ||
			(rect.size.area() >= 50 && areaRatio(contour, rect) > 0.6));
}

std::vector<cv::Point2f> ArmorDetector::DetectArmor(cv::Mat &img)
{
	armorList.clear();
	img_ = img.clone();
	cv::Mat grayImg, colorImg, tempImg;
	std::vector<cv::Mat> bgr;
	std::vector<LightBar> LightBarVector;
	//バイナリー画像から輪郭抽出
	std::vector<std::vector<cv::Point>> contours;
	// BGR画像からグレースケール画像を生成
	cv::cvtColor(img_, grayImg, cv::COLOR_BGR2GRAY);
	// BGR画像からB,G,Rを成分ごとに分解
	cv::split(img_, bgr);
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
	binaryImg_ = colorImg & grayImg;
	//    cv::dilate(binaryImg_,resultImg_,element55);
	//    cv::medianBlur(binaryImg_, tempImg, 3);
	cv::morphologyEx(binaryImg_, resultImg_, cv::MORPH_OPEN, element55);
	cv::findContours(resultImg_, contours, cv::RETR_EXTERNAL,
					 cv::CHAIN_APPROX_NONE);
	for (size_t i = 0; i < contours.size(); i++)
	{
		double aspectRatio = 0;
		double area = cv::contourArea(contours[i]);
		//		std::cout << area << ",";
		cv::RotatedRect rRect = cv::fitEllipse(contours[i]);
		cv::Point2f rectPoint[4];
		rRect.points(rectPoint);
		if (area > 10)
		{
			for (size_t j1 = 0; j1 < 4; j1++)
			{
				//検出した領域を紫直線で囲む
				cv::line(img_, rectPoint[j1], rectPoint[(j1 + 1) % 4],
						 cv::Scalar(255, 0, 255), 2);
			}
			if (isValidLightBar(contours[i], rRect))
			{
				for (size_t j2 = 0; j2 < 4; j2++)
				{
					//ライトバー領域を青直線で囲む
					cv::line(img_, rectPoint[j2], rectPoint[(j2 + 1) % 4],
							 cv::Scalar(255, 255, 0), 2);
				}
				LightBar r(rRect);
				LightBarVector.push_back(r);
			}
		}
	}
	//	std::cout << std::endl;
	for (size_t i = 0; i < LightBarVector.size(); i++)
	{
		for (size_t j = i + 1; j < LightBarVector.size(); j++)
		{
			Armor armor(LightBarVector[i], LightBarVector[j]);
			if (armor.isArmor() == true)
			{
				armorList.push_back(armor.getArmorCenter());
				// armor.getArmorCenter().y);
				armor.drawArmor(img_);
			}
		}
	}
//	cv::imshow("result", img_);
//	cv::waitKey(1);
	//	std::cout<<timer.GetTime(ProcessingTimer::Unit::MILLI_SECONDS)<<"[ms]"<<std::endl;
	return armorList;
}
std::vector<cv::Point2f> ArmorDetector::DetectArmor(FrameData frame)
{
	SetFrame(frame);
	armorList.clear();
	cv::Mat grayImg, colorImg, tempImg;
	std::vector<cv::Mat> bgr;
	std::vector<LightBar> LightBarVector;
	//バイナリー画像から輪郭抽出
	std::vector<std::vector<cv::Point>> contours;
	// BGR画像からグレースケール画像を生成
	cv::cvtColor(detectFrame_.img_, grayImg, cv::COLOR_BGR2GRAY);
	// BGR画像からB,G,Rを成分ごとに分解
	cv::split(detectFrame_.img_, bgr);
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
	binaryImg_ = colorImg & grayImg;
	//    cv::dilate(binaryImg_,resultImg_,element55);
	//    cv::medianBlur(binaryImg_, tempImg, 3);
	cv::morphologyEx(binaryImg_, resultImg_, cv::MORPH_OPEN, element55);
	cv::findContours(resultImg_, contours, cv::RETR_EXTERNAL,
					 cv::CHAIN_APPROX_NONE);
	for (size_t i = 0; i < contours.size(); i++)
	{
		double aspectRatio = 0;
		double area = cv::contourArea(contours[i]);
		//		std::cout << area << ",";
		cv::RotatedRect rRect = cv::fitEllipse(contours[i]);
		cv::Point2f rectPoint[4];
		rRect.points(rectPoint);
		if (area > 10)
		{
			for (size_t j1 = 0; j1 < 4; j1++)
			{
				//検出した領域を紫直線で囲む
				cv::line(detectFrame_.img_, rectPoint[j1],
						 rectPoint[(j1 + 1) % 4], cv::Scalar(255, 0, 255), 2);
			}
			if (isValidLightBar(contours[i], rRect))
			{
				for (size_t j2 = 0; j2 < 4; j2++)
				{
					//ライトバー領域を青直線で囲む
					cv::line(detectFrame_.img_, rectPoint[j2],
							 rectPoint[(j2 + 1) % 4], cv::Scalar(255, 255, 0),
							 2);
				}
				LightBar r(rRect);
				LightBarVector.push_back(r);
			}
		}
	}
	//	std::cout << std::endl;
	for (size_t i = 0; i < LightBarVector.size(); i++)
	{
		for (size_t j = i + 1; j < LightBarVector.size(); j++)
		{
			Armor armor(LightBarVector[i], LightBarVector[j]);
			if (armor.isArmor() == true)
			{
				armorList.push_back(armor.getArmorCenter());
							//  armor.getArmorCenter().y);
				armor.drawArmor(detectFrame_.img_);
				//                cv::imshow("result", detectFrame_.img_);
				//                cv::waitKey(1);
			}
		}
	}
	//	std::cout<<timer.GetTime(ProcessingTimer::Unit::MILLI_SECONDS)<<"[ms]"<<std::endl;
	return armorList;
}

cv::Mat ArmorDetector::GetInputImg()
{
	return img_;
}

cv::Mat ArmorDetector::GetBinary()
{
	return binaryImg_;
}
cv::Mat ArmorDetector::GetResult()
{
	return resultImg_;
}

void ArmorDetector::setGrayThreshold(int val)
{
	if (val > 255)
	{
		val = 255;
	}
	else if (val < 0)
	{
		val = 0;
	}
	GRAYTH_ = val;
}

void ArmorDetector::setColorThreshold(int val)
{
	if (val > 255)
	{
		val = 255;
	}
	else if (val < 0)
	{
		val = 0;
	}
	COLORTH_ = val;
}

int ArmorDetector::getGrayThreshold()
{
	return GRAYTH_;
}

int ArmorDetector::getColorThreshold()
{
	return COLORTH_;
}
