#include <opencv2/opencv.hpp>
#include <iostream>

enum class Classification {
    CUE_BALL,
    EIGHT_BALL,
    WHITE_STRIPES,
    SOLID_COLOR
};

struct Ball {
    cv::Point center;
    float radius;
    Classification classification;
};

Classification classifyBall(cv::Mat& img, cv::Point center, float radius, std::vector<Ball>& balls)
{
    int x = center.x - radius;
    int y = center.y - radius;
    int width = 2 * radius;
    int height = 2 * radius;

    x = std::max(0, x);
    y = std::max(0, y);
    width = std::min(width, img.cols - x);
    height = std::min(height, img.rows - y);

    cv::Mat roi = img(cv::Rect(x, y, width, height));

    cv::Mat roi_gray;
    cv::cvtColor(roi, roi_gray, cv::COLOR_BGR2GRAY);

    cv::Mat filtered;
    cv::GaussianBlur(roi_gray, filtered, cv::Size(3, 3), 3);

    cv::Mat mask_white;
    cv::inRange(filtered, cv::Scalar(180), cv::Scalar(255), mask_white);

    cv::Mat mask_black;
    cv::inRange(filtered, cv::Scalar(0), cv::Scalar(50), mask_black);

    int white_pixel_count = cv::countNonZero(mask_white);
    int black_pixel_count = cv::countNonZero(mask_black);
    int total_pixels = width * height;

    Classification classification;

    if (white_pixel_count > 0.30 * total_pixels)
        classification =  Classification::CUE_BALL;
    else if (black_pixel_count > 0.30 * total_pixels)
        classification = Classification::EIGHT_BALL;
    else if (white_pixel_count > 0.01 * total_pixels)
        classification = Classification::WHITE_STRIPES;
    else
        classification = Classification::SOLID_COLOR;

    balls.emplace_back(Ball{center, radius, classification});

    return classification;
}

cv::Point2f computeIntersection(cv::Vec4i a, cv::Vec4i b)
{
    cv::Point2f p(-1, -1);
    auto denominator = static_cast<float>((a[0] - a[2]) * (b[1] - b[3]) - (a[1] - a[3]) * (b[0] - b[2]));
    if (denominator != 0) {
        p.x = static_cast<float>((a[0] * a[3] - a[1] * a[2]) * (b[0] - b[2]) - (a[0] - a[2]) * (b[0] * b[3] - b[1] * b[2])) / denominator;
        p.y = static_cast<float>((a[0] * a[3] - a[1] * a[2]) * (b[1] - b[3]) - (a[1] - a[3]) * (b[0] * b[3] - b[1] * b[2])) / denominator;
    }
    return p;
}

cv::Scalar getColor(Classification classification)
{
    switch (classification) {
        case Classification::CUE_BALL: return {255, 255, 255};
        case Classification::EIGHT_BALL: return {0, 0, 0};
        case Classification::WHITE_STRIPES: return {255, 178, 102};
        case Classification::SOLID_COLOR: return {0, 0, 255};
        default: return {0, 0, 0};
    }
}

void drawFieldAndBalls(cv::Mat& image, const std::vector<Ball>& balls, const cv::Rect& boundingBox)
{
    cv::rectangle(image, boundingBox, cv::Scalar(0, 100, 0), -1);

    for (const auto& ball : balls) {
        cv::Point originalCenter(ball.center.x + boundingBox.x, ball.center.y + boundingBox.y);
        int radius = ball.radius;
        Classification classification = ball.classification;
        cv::Scalar color = getColor(classification);
        cv::circle(image, originalCenter, radius, color, -1);
    }
}

void createMinimap(const cv::Mat& image, const std::vector<Ball>& balls, cv::Rect boundingBox) {
    int frameHeight = image.rows;
    int frameWidth = image.cols;

    cv::Mat miniMap = cv::Mat::zeros(cv::Size(frameWidth, frameHeight), CV_8UC3);
    miniMap = cv::Scalar(255, 255, 255);

    drawFieldAndBalls(miniMap, balls, boundingBox);

    cv::rectangle(miniMap, cv::Point(0, 0), cv::Point(miniMap.cols - 1, miniMap.rows - 1), cv::Scalar(0, 0, 0), 50);

    cv::Mat resizedMiniMap;
    cv::resize(miniMap, resizedMiniMap, cv::Size(), 0.25, 0.25);

    int miniMapHeight = resizedMiniMap.rows;
    int miniMapWidth = resizedMiniMap.cols;

    cv::Rect miniMapRect(0, frameHeight - miniMapHeight, miniMapWidth, miniMapHeight);
    cv::Mat miniMapRegion = image(miniMapRect);
    resizedMiniMap.copyTo(miniMapRegion);
}

void processFrame(cv::Mat image)
{
    cv::Mat img = image.clone();
    int xOffset = image.cols * 0.13;
    int yOffset = image.rows * 0.13;
    int cropWidth = image.cols - 2 * xOffset;
    int cropHeight = image.rows - 2 * yOffset;

    cv::Rect roi(xOffset, yOffset, cropWidth, cropHeight);
    cv::Mat croppedImage = image(roi);

    cv::Mat gray;
    cv::cvtColor(croppedImage, gray, cv::COLOR_BGR2GRAY);

    cv::Mat filtered;
    cv::bilateralFilter(gray, filtered, 5, 75, 75);

    cv::Mat edges;
    cv::Canny(filtered, edges, 100, 200);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 100, 50, 50);

    std::vector<cv::Vec4i> filteredLines;
    for (auto line : lines) {
        double length = sqrt(pow(line[2] - line[0], 2) + pow(line[3] - line[1], 2));

        if (length > 175)
            filteredLines.push_back(line);
    }

    std::vector<cv::Point2f> points;
    for (size_t i = 0; i < filteredLines.size(); i++) {
        for (size_t j = i + 1; j < filteredLines.size(); j++) {
            cv::Point2f pt = computeIntersection(filteredLines[i], filteredLines[j]);
            if (pt.x >= 0 && pt.y >= 0 && pt.x < cropWidth && pt.y < cropHeight)
                points.push_back(pt);
        }
    }

    std::vector<cv::Point> hull;
    if (!points.empty()) {
        std::vector<cv::Point> intPoints(points.size());

        for (size_t i = 0; i < points.size(); i++)
            intPoints[i] = points[i];

        cv::convexHull(intPoints, hull);
    }

    cv::Scalar borderColor(0, 255, 255);

    int thickness = 2;

    if (!hull.empty()) {
        for (auto& pt : hull) {
            pt.x += xOffset;
            pt.y += yOffset;
        }
        std::vector<std::vector<cv::Point>> hulls(1, hull);
        cv::polylines(image, hulls, true, borderColor, thickness);

        cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::fillPoly(mask, hulls, cv::Scalar(255));

        cv::Mat result = cv::Mat::zeros(image.size(), image.type());
        image.copyTo(result, mask);

        cv::Rect boundingBox = cv::boundingRect(hull);
        cv::Mat croppedResult = result(boundingBox);

        cv::Mat img_gray;
        cv::cvtColor(croppedResult, img_gray, cv::COLOR_BGR2GRAY);

        cv::Mat croppedFiltered;
        cv::bilateralFilter(img_gray, croppedFiltered, 5, 75, 75);

        cv::Mat inverted;
        cv::bitwise_not(croppedFiltered, inverted);

        cv::Mat binarized;
        cv::threshold(inverted, binarized, 160, 255, cv::THRESH_BINARY );

        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        cv::Mat morphed;
        cv::morphologyEx(binarized, morphed, cv::MORPH_OPEN, element);

        cv::Mat element2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::Mat dilatate;
        cv::dilate(morphed, dilatate, element2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilatate, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<Ball> balls;
        for (const auto& contour : contours) {
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);

            if (radius > 10 && radius < 50) {
                cv::Point originalCenter(center.x + boundingBox.x, center.y + boundingBox.y);

                Classification classification = classifyBall(croppedResult, center, radius, balls);
                cv::Scalar color = getColor(classification);

                cv::rectangle(image, cv::Rect(originalCenter.x - radius, originalCenter.y - radius, radius * 2, radius * 2), color, 2);
            }
        }

        createMinimap(image, balls, boundingBox);

        cv::imshow("Billiard Video Analysis", image);
    }
}

int main()
{
    cv::VideoCapture cap;
    cap.open("../Images/game1_clip1.mp4");

    if (!cap.isOpened()) {
        std::cout << "Error: Unable to open video file" << std::endl;
        return -1;
    }

    double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "Resolution of the video: " << dWidth << " x " << dHeight << std::endl;

    std::string window_name = "Billiard Video Analysis";
    namedWindow(window_name, cv::WINDOW_NORMAL);

    cv::Mat frame;
    while (true) {
        bool bSuccess = cap.read(frame);
        if (!bSuccess) {
            std::cout << "Video is completed" << std::endl;
            break;
        }

        processFrame(frame);

        if (cv::waitKey(10) == 27) {
            std::cout << "Esc key is pressed by user. Stopping the video" << std::endl;
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}