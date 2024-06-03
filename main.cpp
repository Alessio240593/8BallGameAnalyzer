#include <opencv2/opencv.hpp>
#include <iostream>

std::string classifyBall(cv::Mat& img, cv::Point center, int radius)
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

    cv::Mat mask_white;
    cv::threshold(roi_gray, mask_white, 200, 255, cv::THRESH_BINARY);

    cv::Mat mask_black;
    cv::threshold(roi_gray, mask_black, 50, 255, cv::THRESH_BINARY_INV);

    int white_pixel_count = cv::countNonZero(mask_white);
    int black_pixel_count = cv::countNonZero(mask_black);

    if (white_pixel_count > 0.30 * width * height)
        return "Cue ball";
    else if (black_pixel_count > 0.30 * width * height)
        return "8 ball";
    else if (white_pixel_count > 0.05 * width * height)
        return "With stripes";
    else
        return "Solid color";
}

cv::Point2f computeIntersection(cv::Vec4i a, cv::Vec4i b)
{
    cv::Point2f p(-1, -1);
    float denominator = static_cast<float>((a[0] - a[2]) * (b[1] - b[3]) - (a[1] - a[3]) * (b[0] - b[2]));
    if (denominator != 0) {
        p.x = static_cast<float>((a[0] * a[3] - a[1] * a[2]) * (b[0] - b[2]) - (a[0] - a[2]) * (b[0] * b[3] - b[1] * b[2])) / denominator;
        p.y = static_cast<float>((a[0] * a[3] - a[1] * a[2]) * (b[1] - b[3]) - (a[1] - a[3]) * (b[0] * b[3] - b[1] * b[2])) / denominator;
    }
    return p;
}

cv::Scalar getColor(std::string classification)
{
    if (classification == "Cue ball")
        return {255, 255, 255};
    else if (classification == "8 ball")
        return {0, 0, 0};
    else if (classification == "With stripes")
        return {0, 255, 0};
    else if (classification == "Solid color")
        return {255, 0, 0};
    else
        return {255, 0, 255};
}

int main()
{
    cv::Mat image = cv::imread("../Images/frame_first2.png");
    if (image.empty()) {
        std::cerr << "Could not load the image" << std::endl;
        return -1;
    }

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
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i line = lines[i];
        double length = sqrt(pow(line[2] - line[0], 2) + pow(line[3] - line[1], 2));

        if (length > 200)
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

        cv::Mat result;
        image.copyTo(result, mask);

        cv::Rect boundingBox = cv::boundingRect(hull);

        cv::Mat croppedResult = result(boundingBox);

        cv::Mat img_gray;
        cv::cvtColor(croppedResult, img_gray, cv::COLOR_BGR2GRAY);

        cv::medianBlur(img_gray, img_gray, 5);

        cv::equalizeHist(img_gray, img_gray);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(img_gray, circles, cv::HOUGH_GRADIENT, 1, img_gray.rows / 16, 100, 10, 8, 15);

        if (!circles.empty()) {
            for (size_t i = 0; i < circles.size(); i++) {
                cv::Vec3i c = circles[i];
                cv::Point center = cv::Point(c[0], c[1]);
                int radius = c[2];

                std::string classification = classifyBall(croppedResult, center, radius);
                cv::Scalar color = getColor(classification);

                cv::Point originalCenter(center.x + boundingBox.x, center.y + boundingBox.y);
                cv::rectangle(image, cv::Rect(originalCenter.x - radius, originalCenter.y - radius, radius * 2, radius * 2), color, 2);
                cv::putText(image, classification, cv::Point(originalCenter.x - radius, originalCenter.y - radius - 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, color, 1);
            }
        }
    }

    cv::imshow("Billiard Table with Border", image);
    cv::waitKey(0);

    return 0;
}
