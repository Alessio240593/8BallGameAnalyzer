#include <iostream>
#include "billiardAnalyzer.hpp"

void processFrame(cv::Mat image)
{
    int xOffset = image.cols * 0.13;
    int yOffset = image.rows * 0.13;
    int cropWidth = image.cols - 2 * xOffset;
    int cropHeight = image.rows - 2 * yOffset;

    // crop the image to reduce the searching area
    cv::Rect roi(xOffset, yOffset, cropWidth, cropHeight);
    cv::Mat croppedImage = image(roi);

    // convert the image to grayscale
    cv::Mat gray;
    cv::cvtColor(croppedImage, gray, cv::COLOR_BGR2GRAY);

    // smoothing the image preserving the border
    cv::Mat filtered;
    cv::bilateralFilter(gray, filtered, 5, 75, 75);

    // calculate the edges
    cv::Mat edges;
    cv::Canny(filtered, edges, 100, 200);

    // find the lines in the image
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 100, 50, 50);

    // filter the lines by length
    std::vector<cv::Vec4i> filteredLines;
    billiardAnalyzer::Utility::filterLinesByLength(lines, filteredLines);

    // compute intersection
    std::vector<cv::Point> points;
    for (size_t i = 0; i < filteredLines.size(); i++) {
        for (size_t j = i + 1; j < filteredLines.size(); j++) {
            cv::Point2f pt = billiardAnalyzer::Utility::computeIntersection(filteredLines[i], filteredLines[j]);
            if (pt.x >= 0 && pt.y >= 0 && pt.x < cropWidth && pt.y < cropHeight)
                points.push_back(pt);
        }
    }

    // calculate the convex hull
    std::vector<cv::Point> hull;
    if (!points.empty())
        cv::convexHull(points, hull);

    // table color and lines thickness
    cv::Scalar borderColor(0, 255, 255);
    int thickness = 2;

    if (!hull.empty()) {
        // adjust the points
        for (auto& pt : hull) {
            pt.x += xOffset;
            pt.y += yOffset;
        }
        // draw the field as polygon
        std::vector<std::vector<cv::Point>> hulls(1, hull);
        cv::polylines(image, hulls, true, borderColor, thickness);

        // obtain only the table image
        cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::fillPoly(mask, hulls, cv::Scalar(255));

        cv::Mat result = cv::Mat::zeros(image.size(), image.type());
        image.copyTo(result, mask);

        cv::Rect boundingBox = cv::boundingRect(hull);
        cv::Mat croppedResult = result(boundingBox);

        // convert field to grayscale
        cv::Mat img_gray;
        cv::cvtColor(croppedResult, img_gray, cv::COLOR_BGR2GRAY);

        // smoothing the image preserving the border
        cv::Mat croppedFiltered;
        cv::bilateralFilter(img_gray, croppedFiltered, 5, 75, 75);

        // invert the intensity of the pixel
        cv::Mat inverted;
        cv::bitwise_not(croppedFiltered, inverted);

        // threshold the image to bynarize the image
        cv::Mat binarized;
        cv::threshold(inverted, binarized, 160, 255, cv::THRESH_BINARY );

        cv::Mat edges;
        cv::Canny(binarized, edges, 10, 50);

        // fill the gap created by opening
        cv::Mat element2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::Mat dilatate;
        cv::dilate(edges, dilatate, element2);

        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::Mat morphed;
        cv::morphologyEx(dilatate, morphed, cv::MORPH_OPEN, element);

        // find the balls contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(morphed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // for each balls let's find the center, radius and classification
        std::vector<billiardAnalyzer::Ball> balls;
        for (const auto& contour : contours) {
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);

            if (radius > 10 && radius < 50) {
                auto* pBall = new billiardAnalyzer::Ball(center, radius);
                pBall->classify(croppedResult);

                balls.emplace_back(*pBall);

                cv::Point originalCenter(center.x + boundingBox.x, center.y + boundingBox.y);
                cv::rectangle(image, cv::Rect(originalCenter.x - radius, originalCenter.y - radius, radius * 2, radius * 2), pBall->getColor(), 2);

                delete pBall;
            }
        }

        // create the minimap of the game
        billiardAnalyzer::Utility::createMinimap(image, balls, boundingBox);

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