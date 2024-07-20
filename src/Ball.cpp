/*
 * Ball.cpp
 *
 * Copyright (C) 2024 Alessio Zattoni
 *
 * Author: Alessio Zattoni
 * Date: 18/07/2024
 * Version: 1.0.0
 *
 * This file is part of 8BallGameAnalyzer.
 *
 * 8BallGameAnalyzer is free software: is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MyProject is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MyProject.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include "billiardAnalyzer.hpp"

namespace billiardAnalyzer
{
    Ball::Ball(cv::Point center,
               float radius,
               Classification classification)
               :
               center(center),
               radius(radius),
               classification(classification) {}

    Ball::Ball(cv::Point center, float radius)
            :
            center(center),
            radius(radius),
            classification(Classification::NONE) {}

    Ball::~Ball() = default;

    cv::Point Ball::getCenter() const
    {
        return this->center;
    }

    void Ball::setCenter(cv::Point center)
    {
        this->center = center;
    }

    void Ball::setCenter(int x, int y)
    {
        this->center.x = x;
        this->center.y = y;
    }

    float Ball::getRadius() const
    {
        return this->radius;
    }

    void Ball::setRadius(float radius)
    {
        this->radius = radius;
    }

    Ball::Classification Ball::getClassification() const
    {
        return this->classification;
    }

    void Ball::setClassification(Ball::Classification classification)
    {
        this->classification = classification;
    }

    cv::Scalar Ball::getColor() const
    {
        switch (this->classification) {
            case billiardAnalyzer::Ball::Classification::CUE_BALL:
                return CUE_BALL_COLOR;
            case billiardAnalyzer::Ball::Classification::EIGHT_BALL:
                return EIGHT_BALL_COLOR;
            case billiardAnalyzer::Ball::Classification::WHITE_STRIPES:
                return WHITE_STRIPES_COLOR;
            case billiardAnalyzer::Ball::Classification::SOLID_COLOR:
                return SOLID_COLOR_COLOR;
            case billiardAnalyzer::Ball::Classification::NONE:
                return NONE_COLOR;
            default:
                return {};
        }
    }

    bool Ball::compareBallsByClassification(const billiardAnalyzer::Ball& a, const billiardAnalyzer::Ball& b)
    {
        return static_cast<int>(a.getClassification()) < static_cast<int>(b.getClassification());
    }

    void Ball::classify(cv::Mat& image)
    {
        if (image.empty())
            throw std::invalid_argument("Image should not be empty");

        if (center.x == 0 || center.y == 0 || radius == 0)
            throw std::invalid_argument("Center coordinates and radius should not be zero");

        cv::Mat imgHsv;
        cv::cvtColor(image, imgHsv, cv::COLOR_BGR2HSV);

        cv::Mat maskWhite = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::Mat maskBlack = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::Mat maskColor = cv::Mat::zeros(image.size(), CV_8UC1);

        for (int i = -radius; i <= radius; ++i) {
            for (int j = -radius; j <= radius; ++j) {
                int x = center.x + i;
                int y = center.y + j;

                if (x >= 0 && x < image.cols && y >= 0 && y < image.rows && i * i + j * j <= radius * radius) {
                    cv::Vec3b hsvPixel = imgHsv.at<cv::Vec3b>(y, x);

                    if (Utility::isInRange(LOWER_RANGE_CUE_BALL, UPPER_RANGE_CUE_BALL, hsvPixel))
                        maskWhite.at<uchar>(y, x) = 255;
                    else if (Utility::isInRange(LOWER_RANGE_8_BALL, UPPER_RANGE_8_BALL, hsvPixel))
                        maskBlack.at<uchar>(y, x) = 255;
                    else if (Utility::isInRange(LOWER_RANGE_STRIPED_BALL, UPPER_RANGE_STRIPED_BALL, hsvPixel))
                        maskColor.at<uchar>(y, x) = 255;
                }
            }
        }

        int whitePixelCount = cv::countNonZero(maskWhite);
        int blackPixelCount = cv::countNonZero(maskBlack);
        int colorPixelCount = cv::countNonZero(maskColor);
        int totalPixels = CV_PI * radius * radius;

        billiardAnalyzer::Ball::Classification ballClassification;

        if (whitePixelCount > 0.5 * totalPixels && blackPixelCount < 0.3 * totalPixels)
            ballClassification = billiardAnalyzer::Ball::Classification::CUE_BALL;
        else if (colorPixelCount > 0 && whitePixelCount > 0.15 * totalPixels)
            ballClassification = billiardAnalyzer::Ball::Classification::WHITE_STRIPES;
        else if (blackPixelCount > 0.4 * totalPixels)
            ballClassification = billiardAnalyzer::Ball::Classification::EIGHT_BALL;
        else
            ballClassification = billiardAnalyzer::Ball::Classification::SOLID_COLOR;

        setClassification(ballClassification);
    }

    void Ball::drawBallBox(cv::Mat& image,
                           uint8_t ballBoxThickness,
                           float ballBoxInnerGradient)
    {
        int centerX = this->getCenter().x;
        int centerY = this->getCenter().y;
        float radius = this->getRadius();
        const cv::Scalar& ballColor = this->getColor();

        cv::Scalar rectColor(ballColor[0], ballColor[1], ballColor[2]);

        cv::Mat overlay;
        image.copyTo(overlay);

        cv::rectangle(overlay,
                      cv::Rect(centerX - radius , centerY - radius, radius * 2,radius * 2),
                      rectColor,
                      cv::FILLED);


        cv::addWeighted(overlay, ballBoxInnerGradient, image, 1 - ballBoxInnerGradient, 0, image);

        cv::rectangle(image,
                      cv::Rect(centerX - radius, centerY - radius, radius * 2, radius * 2),
                      ballColor ,
                      ballBoxThickness);
    }
}