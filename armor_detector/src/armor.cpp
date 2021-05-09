#include "armor.hpp"

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