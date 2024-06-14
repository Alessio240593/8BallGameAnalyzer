#include "billiardAnalyzer.hpp"

namespace billiardAnalyzer {
    //Ball
    Ball::Ball(cv::Point center,
               float radius,
               Classification classification) :
        center(center),
        radius(radius),
        classification(classification)
    {}

    Ball::Ball(cv::Point center, float radius)  :
            center(center),
            radius(radius),
            classification(Classification::NONE)
    {}

    Ball::~Ball()
    {}

    cv::Point Ball::getCenter()
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

    float Ball::getRadius()
    {
        return this->radius;
    }

    void Ball::setRadius(float radius)
    {
        this->radius = radius;
    }

    Ball::Classification Ball::getClassification()
    {
        return this->classification;
    }

    void Ball::setClassification(Ball::Classification classification)
    {
        this->classification = classification;
    }

    cv::Scalar Ball::getColor()
    {
        switch (this->classification) {
            case billiardAnalyzer::Ball::Classification::CUE_BALL: return {255, 255, 255};
            case billiardAnalyzer::Ball::Classification::EIGHT_BALL: return {0, 0, 0};
            case billiardAnalyzer::Ball::Classification::WHITE_STRIPES: return {255, 178, 102};
            case billiardAnalyzer::Ball::Classification::SOLID_COLOR: return {0, 0, 255};
            case billiardAnalyzer::Ball::Classification::NONE: return {255, 255, 255};
        }
    }

    void Ball::classify(cv::Mat& img)
    {
        if (img.empty())
            throw std::invalid_argument("Image should be not empty");

        if (center.x == 0 || center.y == 0 || radius == 0)
            throw std::invalid_argument("Radius and center should be not zero");

        // compute the ball coordinate and size
        int x = center.x - radius;
        int y = center.y - radius;
        int width = 2 * radius;
        int height = 2 * radius;

        x = std::max(0, x);
        y = std::max(0, y);
        width = std::min(width, img.cols - x);
        height = std::min(height, img.rows - y);

        // extract a region where the ball is present
        cv::Mat roi = img(cv::Rect(x, y, width, height));

        // convert to grayscale
        cv::Mat roi_gray;
        cv::cvtColor(roi, roi_gray, cv::COLOR_BGR2GRAY);

        // smoothing image preserve then border
        cv::Mat filtered;
        cv::bilateralFilter(roi_gray, filtered, 22, 75, 75);

        // mask for white pixel
        cv::Mat mask_white;
        cv::inRange(filtered, cv::Scalar(180), cv::Scalar(255), mask_white);

        // mask for balck pixel
        cv::Mat mask_black;
        cv::inRange(filtered, cv::Scalar(0), cv::Scalar(70), mask_black);

        // count white and black pixel in the roi
        int white_pixel_count = cv::countNonZero(mask_white);
        int black_pixel_count = cv::countNonZero(mask_black);
        // total pixel in the roi
        int total_pixels = width * height;

        billiardAnalyzer::Ball::Classification ballClassification;

        // classify the ball take into account the intensity of the pixel in the roi
        if (white_pixel_count > 0.10 * total_pixels)
            ballClassification = billiardAnalyzer::Ball::Classification::CUE_BALL;
        else if (black_pixel_count > 0.30 * total_pixels)
            ballClassification = billiardAnalyzer::Ball::Classification::EIGHT_BALL;
        else if (white_pixel_count > 0.01 * total_pixels)
            ballClassification = billiardAnalyzer::Ball::Classification::WHITE_STRIPES;
        else
            ballClassification = billiardAnalyzer::Ball::Classification::SOLID_COLOR;

        // set the clasification of the ball
        setClassification(ballClassification);
    }


    // Utility
    cv::Point2f Utility::computeIntersection(cv::Vec4i a, cv::Vec4i b)
    {
        cv::Point2f p(-1, -1);
        auto denominator = static_cast<float>((a[0] - a[2]) * (b[1] - b[3]) - (a[1] - a[3]) * (b[0] - b[2]));
        if (denominator != 0) {
            p.x = static_cast<float>((a[0] * a[3] - a[1] * a[2]) * (b[0] - b[2]) - (a[0] - a[2]) * (b[0] * b[3] - b[1] * b[2])) / denominator;
            p.y = static_cast<float>((a[0] * a[3] - a[1] * a[2]) * (b[1] - b[3]) - (a[1] - a[3]) * (b[0] * b[3] - b[1] * b[2])) / denominator;
        }
        return p;
    }

    void Utility::drawFieldAndBalls(cv::Mat& image, const std::vector<billiardAnalyzer::Ball>& balls, const cv::Rect& field)
    {
        // draw the field
        cv::rectangle(image, field, cv::Scalar(0, 100, 0), -1);

        // draw the balls
        for (billiardAnalyzer::Ball ball : balls) {
            cv::Point originalCenter(ball.getCenter().x + field.x, ball.getCenter().y + field.y);
            cv::circle(image, originalCenter, ball.getRadius(), ball.getColor(), -1);
        }
    }

    void Utility::createMinimap(const cv::Mat& image, const std::vector<billiardAnalyzer::Ball>& balls, cv::Rect boundingBox)
    {
        if (image.empty())
            throw std::invalid_argument("Image should be not empty");

        // minimap
        cv::Mat miniMap = cv::Mat::zeros(cv::Size( image.cols, image.rows), CV_8UC3);
        // white background
        miniMap = cv::Scalar(255, 255, 255);

        // drw fields and balls into minimap
        drawFieldAndBalls(miniMap, balls, boundingBox);

        // minimap border
        cv::rectangle(miniMap, cv::Point(0, 0), cv::Point(miniMap.cols - 1, miniMap.rows - 1), cv::Scalar(0, 0, 0), 50);

        // resize minimap
        cv::Mat resizedMiniMap;
        cv::resize(miniMap, resizedMiniMap, cv::Size(), 0.3, 0.3);

        // copy minimap into left bottom corner of the image
        cv::Rect miniMapRect(0, image.rows -  resizedMiniMap.rows, resizedMiniMap.cols, resizedMiniMap.rows);
        cv::Mat miniMapRegion = image(miniMapRect);
        resizedMiniMap.copyTo(miniMapRegion);
    }

    void Utility::filterLinesByLength(const std::vector<cv::Vec4i>& lines , std::vector<cv::Vec4i>& filteredLines)
    {
        for (auto line : lines) {
            double length = sqrt(pow(line[2] - line[0], 2) + pow(line[3] - line[1], 2));

            if (length > 175)
                filteredLines.push_back(line);
        }
    }
}