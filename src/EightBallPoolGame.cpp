/*
 * EightBallPoolGame.cpp
 *
 * Copyright (C) 2024 Alessio Zattoni
 *
 * Author: Alessio Zattoni
 * Date: TODO
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


#include <utility>
#include <fstream>
#include "billiardAnalyzer.hpp"

namespace billiardAnalyzer
{
    EightBallPoolGame::EightBallPoolGame(std::vector<Ball> balls, PoolTable& poolTable)
            :
            balls(std::move(balls)),
            poolTable(poolTable) {}

    EightBallPoolGame::EightBallPoolGame() = default;

    EightBallPoolGame::~EightBallPoolGame() = default;

    std::vector<Ball> EightBallPoolGame::getBalls() const
    {
        return this->balls;
    }

    void EightBallPoolGame::setBalls(std::vector<Ball> balls)
    {
        this->balls = balls;
    }

    PoolTable& EightBallPoolGame::getPoolTable()
    {
        return this->poolTable;
    }

    void EightBallPoolGame::setPoolTable(PoolTable& poolTable)
    {
        this->poolTable = poolTable;
    }

    void EightBallPoolGame::printBallsInformations()
    {
        if (!balls.empty())
        {
            std::sort(balls.begin(), balls.end(), Ball::compareBallsByClassification);

            for (billiardAnalyzer::Ball& ball : balls) {
                int radius = std::ceil(ball.getRadius());
                std::cout << (ball.getCenter().x) - radius << " " << (ball.getCenter().y) - radius << " " << radius * 2 << " " << radius * 2 << " " << static_cast<int>(ball.getClassification()) << std::endl;
            }

            std::cout << std::endl;
        }
        else
            std::cout << "Nothing to display about balls" << std::endl;
    }

    void EightBallPoolGame::processVideo(const std::string& path,
                                         bool showTableDetection,
                                         bool showBallsDetection,
                                         bool showSegmentation,
                                         bool showMinimap)
    {
        cv::VideoCapture cap;
        cap.open(path);

        if (!cap.isOpened()) {
            std::cout << "Error: Unable to open video" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

        std::cout << "Resolution of the video: " << dWidth << " x " << dHeight << std::endl;

        std::string windowName = "Billiard Video Analysis";
        cv::namedWindow(windowName, cv::WINDOW_NORMAL);

        cv::Mat frame;
        auto eightBallPoolGame = std::make_unique<EightBallPoolGame>();

        cv::Mat previousFrame;
        std::map<int, cv::Point2f> previousPositions;
        cv::Mat mask;

        while (true) {
            bool bSuccess = cap.read(frame);
            if (!bSuccess) {
                std::cout << "Video is completed" << std::endl;
                break;
            }

            executeAnalysis(frame, previousFrame, previousPositions, mask, eightBallPoolGame.get(), showTableDetection, showBallsDetection, showSegmentation, showMinimap);

            cv::imshow(windowName, frame);

            previousFrame = frame.clone();

            if (cv::waitKey(10) == 27) {
                std::cout << "Esc key is pressed by user. Stopping the video" << std::endl;
                break;
            }
        }

        cap.release();
        cv::destroyAllWindows();
    }

    void EightBallPoolGame::processImage(const std::string& path,
                                         bool showTableDetection,
                                         bool showBallsDetection,
                                         bool showSegmentation,
                                         bool showMinimap)
    {
        cv::Mat frame = cv::imread(path);

        if (frame.empty()){
            std::cout << "Error: Unable to open the images" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::string windowName = "Billiard Video Analysis";
        cv::namedWindow(windowName, cv::WINDOW_NORMAL);

        auto eightBallPoolGame = std::make_unique<EightBallPoolGame>();
        std::map<int, cv::Point2f> previous_positions;

        cv::Mat mask;
        cv::Mat previousFrame;

        executeAnalysis(frame, previousFrame, previous_positions, mask, eightBallPoolGame.get(), showTableDetection, showBallsDetection, showSegmentation, showMinimap);

        cv::imshow(windowName, frame);
        cv::waitKey(0);
    }

    void EightBallPoolGame::executeAnalysis(cv::Mat& frame,
                                            cv::Mat& previousFrame,
                                            std::map<int, cv::Point2f>& previousPositions,
                                            cv::Mat& mask,
                                            EightBallPoolGame* eightBallPoolGame,
                                            bool showTableDetection,
                                            bool showBallsDetection,
                                            bool showSegmentation,
                                            bool showMinimap)
    {
        billiardAnalyzer::PoolTable table = eightBallPoolGame->getPoolTable();

        if (table.getVertices().empty())
            eightBallPoolGame->detectPoolTable(frame);

        std::vector<billiardAnalyzer::Ball> balls = eightBallPoolGame->detectBalls(frame, 85, 3);

        /* pool table detection */
        if (showTableDetection)
            eightBallPoolGame->getPoolTable().drawPoolTable(frame);

        /* ball detection */
        if (showBallsDetection)
        {
            for (billiardAnalyzer::Ball ball : balls)
                ball.drawBallBox(frame);
        }

        /* segmentation */
        if (showSegmentation)
            eightBallPoolGame->segmentsImage(frame);

        /* trajectory */
        if (mask.empty())
            mask = cv::Mat::zeros(frame.size(), frame.type());

        if (!previousFrame.empty())
            eightBallPoolGame->calculateBallsTrajectory(frame,
                                                        previousFrame,
                                                        mask,
                                                        previousPositions);
        /* minimap */
        if (showMinimap)
            billiardAnalyzer::Utility::createMinimap(frame, mask, *eightBallPoolGame);

        /* info */
//        eightBallPoolGame->printBallsInformations();
//
//        /* calculate mIoU */
//        cv::Mat groundTruth = cv::imread("../Resources/masks/frame_last.png", cv::IMREAD_GRAYSCALE);
//        float mIoU = Utility::computeMIoU(frame, groundTruth, eightBallPoolGame);
//        std::cout << "Mean IoU: " << std::fixed << std::setprecision(2) << mIoU * 100<< std::endl;
//
//        /* calculate mAP */
//        std::ifstream inputFile("../Resources/gt/frame_last_bbox.txt");
//        if (!inputFile) {
//            std::cerr << "Errore nell'apertura del file." << std::endl;
//            return;
//        }
//
//       std::vector<Detection> groundTruthBalls, predictedBalls;
//
//       Utility::prepareDetectionStructure(inputFile,
//                                          groundTruthBalls,
//                                          balls,
//                                          predictedBalls);
//
//        float mAP = Utility::calculatemAP(groundTruthBalls, predictedBalls);
//
//        std::cout << "Mean Average Precision (mAP): " << std::fixed << std::setprecision(2) << mAP * 100 << "%" << std::endl;
    }

    void EightBallPoolGame::analyzingEightBallPoolGame(int argc, char** argv)
    {
        if (argc == 1)
        {
            Utility::printUsage();
            throw std::invalid_argument("An images or video is required, use path option for this purpose");
        }
        else
        {
            std::string path;
            bool showTableDetection = false;
            bool showBallsDetection = false;
            bool showSegmentation = false;
            bool showMinimap = false;

            billiardAnalyzer::Utility::parseCommandLineArguments(argc, argv, path, showTableDetection, showBallsDetection, showSegmentation, showMinimap);

            analyzingEightBallPoolGame(path, showTableDetection, showBallsDetection, showSegmentation, showMinimap);
        }
    }

    void EightBallPoolGame::analyzingEightBallPoolGame(const std::string& path,
                                                       bool showTableDetection,
                                                       bool showBallsDetection,
                                                       bool showSegmentation,
                                                       bool showMinimap)
    {
        if (billiardAnalyzer::Utility::isImageFile(path))
            processImage(path, showTableDetection, showBallsDetection, showSegmentation, showMinimap);

        else if (billiardAnalyzer::Utility::isVideoFile(path))
            processVideo(path, showTableDetection, showBallsDetection, showSegmentation, showMinimap);
        else
            throw std::invalid_argument("The path isn't an images or video");
    }

    std::vector<std::vector<cv::Point>> EightBallPoolGame::filterContours(const std::vector<std::vector<cv::Point>>& contours, double alpha)
    {
        std::vector<std::vector<cv::Point>> filteredContours;

        for (const auto & ctr : contours) {
            cv::RotatedRect rotRect = cv::minAreaRect(ctr);
            double w = rotRect.size.width;
            double h = rotRect.size.height;
            double area = cv::contourArea(ctr);

            if ((h * alpha < w) || (w * alpha < h)) {
                continue;
            }

            if (area < 100.0 || area > 1000.0) {
                continue;
            }

            filteredContours.push_back(ctr);
        }

        return filteredContours;
    }

    cv::Vec3b EightBallPoolGame::findModeColor(const cv::Mat& image, const std::vector<cv::Point>& tableVertices)
    {
        if (image.empty())
            throw std::invalid_argument("Image should not be empty");

        if (tableVertices.size() < 4)
            throw std::invalid_argument("Table vertices should contain at least 4 points");

        cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);

        std::vector<std::vector<cv::Point>> hulls(1, tableVertices);
        cv::fillPoly(mask, hulls, cv::Scalar(255));

        cv::Mat validPixels;
        image.copyTo(validPixels, mask);

        std::vector<cv::Mat> bgrPlanes;
        cv::split(validPixels, bgrPlanes);

        int histSize = 256;
        float range[] = {0, 256};
        const float* histRange = {range};
        cv::Mat bHist, gHist, rHist;

        cv::Mat nonZeroMask = (bgrPlanes[0] > 20) & (bgrPlanes[1] > 20) & (bgrPlanes[2] > 20);

        cv::calcHist(&bgrPlanes[0], 1, nullptr, nonZeroMask, bHist, 1, &histSize, &histRange, true, false);
        cv::calcHist(&bgrPlanes[1], 1, nullptr, nonZeroMask, gHist, 1, &histSize, &histRange, true, false);
        cv::calcHist(&bgrPlanes[2], 1, nullptr, nonZeroMask, rHist, 1, &histSize, &histRange, true, false);

        // Trova il valore moda per ciascun canale
        cv::Point maxLocB, maxLocG, maxLocR;
        cv::minMaxLoc(bHist, nullptr, nullptr, nullptr, &maxLocB);
        cv::minMaxLoc(gHist, nullptr, nullptr, nullptr, &maxLocG);
        cv::minMaxLoc(rHist, nullptr, nullptr, nullptr, &maxLocR);

        // Combina i valori moda dei singoli canali
        cv::Vec3b modeColor(maxLocB.y, maxLocG.y, maxLocR.y);

        this->getPoolTable().setColor(cv::Scalar (maxLocB.y, maxLocG.y, maxLocR.y));

        return modeColor;
    }

    std::vector<cv::Point> EightBallPoolGame::detectPoolTable(cv::Mat& image,
                                                              int tolerance)
    {
        if (image.empty())
            throw std::invalid_argument("Image should not be empty");

        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        cv::Mat binary;
        cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        if (contours.size() < 2)
            throw std::runtime_error("Not enough contours found");

        std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a, false) > cv::contourArea(b, false);
        });

        int largestContourIdx = 0;

        std::vector<cv::Point> convexHullPoints;
        cv::convexHull(contours[largestContourIdx], convexHullPoints, true);

        this->getPoolTable().setVertices(convexHullPoints);

        refinePoolTable(image, tolerance, 3);

        return convexHullPoints;
    }

    std::vector<cv::Point>  EightBallPoolGame::refinePoolTable(cv::Mat& image,
                                                               uint8_t rangeTolerance,
                                                               uint8_t morphCloseSize)
    {
        if (image.empty())
            throw std::invalid_argument("Image should not be empty");

        if (this->getPoolTable().getVertices().empty())
            throw std::invalid_argument("Not found vertices to refine");

        auto tableVertices = this->getPoolTable().getVertices();
        auto table = this->getPoolTable().getPoolTableImage(image);

        cv::Mat transformed_blur;
        cv::GaussianBlur(image, transformed_blur, cv::Size(3, 3), 2);
        cv::Mat hsv;
        cv::cvtColor(transformed_blur, hsv, cv::COLOR_BGR2HSV);

        cv::Vec3b exactColorBGR = this->findModeColor(table, tableVertices);

        cv::Mat exactColorBGRMat(1, 1, CV_8UC3, exactColorBGR);
        cv::cvtColor(exactColorBGRMat, exactColorBGRMat, cv::COLOR_BGR2HSV);
        cv::Vec3b exactColorHSV = exactColorBGRMat.at<cv::Vec3b>(0, 0);

        cv::Mat mask;
        cv::Scalar lowerBound(exactColorHSV[0] - rangeTolerance, exactColorHSV[1] - rangeTolerance, exactColorHSV[2] - rangeTolerance);
        cv::Scalar upperBound(exactColorHSV[0] + rangeTolerance, exactColorHSV[1] + rangeTolerance, exactColorHSV[2] + rangeTolerance);
        cv::inRange(hsv, lowerBound, upperBound, mask);

        cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphCloseSize, morphCloseSize));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel2);

        cv::Mat maskInv;
        cv::threshold(mask, maskInv, 5, 255, cv::THRESH_BINARY_INV);

        cv::Mat maskedImg;
        image.copyTo(maskedImg, maskInv);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(maskInv, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        std::size_t largestContourIdx = -1;
        double maxArea = 0;
        std::size_t secondLargestContourIdx = -1;
        double secondMaxArea = 0;

        for (std::size_t i = 0; i < contours.size(); ++i) {
            double area = contourArea(contours[i]);

            if (area > maxArea) {
                secondLargestContourIdx = largestContourIdx;
                secondMaxArea = maxArea;
                largestContourIdx = i;
                maxArea = area;
            } else if (area > secondMaxArea) {
                secondLargestContourIdx = i;
                secondMaxArea = area;
            }
        }

        std::vector<cv::Point> convexHullPoints;
        convexHull(contours[secondLargestContourIdx], convexHullPoints, true);

        this->getPoolTable().setVertices(convexHullPoints);

        return convexHullPoints;
    }

    std::vector<billiardAnalyzer::Ball> EightBallPoolGame::detectBalls(cv::Mat & image,
                                                                       uint8_t rangeTolerance,
                                                                       uint8_t morphCloseSize)
    {
        if (image.empty())
            throw std::invalid_argument("Image should not be empty");

        if (poolTable.getVertices().empty())
            throw std::invalid_argument("Table vertices should not be empty");

        auto tableVertices = poolTable.getVertices();
        auto table = poolTable.getPoolTableImage(image);

        cv::Mat filteredImg;
        cv::bilateralFilter(table, filteredImg, 5, 75, 75);

        cv::Mat hsv;
        cv::cvtColor(filteredImg, hsv, cv::COLOR_BGR2HSV);

        cv::Vec3b exactColorBGR = this->findModeColor(table, tableVertices);

        cv::Mat exactColorBGRMat(1, 1, CV_8UC3, exactColorBGR);
        cv::cvtColor(exactColorBGRMat, exactColorBGRMat, cv::COLOR_BGR2HSV);
        cv::Vec3b exactColorHSV = exactColorBGRMat.at<cv::Vec3b>(0, 0);

        cv::Mat mask;
        cv::Scalar lowerBound(exactColorHSV[0] - rangeTolerance, exactColorHSV[1] - rangeTolerance, exactColorHSV[2] - rangeTolerance);
        cv::Scalar upperBound(exactColorHSV[0] + rangeTolerance, exactColorHSV[1] + rangeTolerance, exactColorHSV[2] + rangeTolerance);
        cv::inRange(hsv, lowerBound, upperBound, mask);

        cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphCloseSize, morphCloseSize));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel2);

        cv::Mat maskInv;
        cv::threshold(mask, maskInv, 0, 255, cv::THRESH_BINARY_INV);

        cv::Mat maskedImg;
        table.copyTo(maskedImg, maskInv);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(maskInv, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat contoursImg = cv::Mat::zeros(maskedImg.size(), CV_8UC3);

        std::vector<std::vector<cv::Point>> filtered;
        filtered = filterContours(contours);

        std::vector<billiardAnalyzer::Ball> balls;
        int minX = std::numeric_limits<int>::max();
        int minY = std::numeric_limits<int>::max();
        int maxX = std::numeric_limits<int>::min();
        int maxY = std::numeric_limits<int>::min();

        for (const auto& vertex : tableVertices)
        {
            minX = std::min(minX, vertex.x);
            minY = std::min(minY, vertex.y);
            maxX = std::max(maxX, vertex.x);
            maxY = std::max(maxY, vertex.y);
        }

        cv::Rect tableRect(minX, minY, maxX - minX, maxY - minY);

        for (const auto & ballsContour : filtered)
        {
            cv::Point2f center;
            float radius;

            cv::minEnclosingCircle(ballsContour, center, radius);

            cv::Point2f globalCenter(center.x + tableRect.x, center.y + tableRect.y);

            auto* pball = new billiardAnalyzer::Ball(globalCenter, radius);
            pball->classify(image);

            balls.emplace_back(*pball);
            delete pball;
        }

        this->setBalls(balls);
        return balls;
    }

    void EightBallPoolGame::segmentsImage(cv::Mat& image)
    {
        Utility::drawPoolTableAndBalls(image, *this);
        poolTable.drawPoolTable(image);
    }

    void EightBallPoolGame::createSegmentationMask(cv::Mat& image, cv::Mat& destImage)
    {
        if (image.empty())
            throw std::invalid_argument("Image should not be empty");

        if (this->getPoolTable().getVertices().empty())
            throw std::invalid_argument("Field polygon is empty");

        if (this->getBalls().empty())
            throw std::invalid_argument("Balls are empty");

        destImage = cv::Mat::zeros(image.size(), CV_8UC1);

        cv::fillConvexPoly(destImage, this->getPoolTable().getVertices(), 5);

        for (Ball ball : this->getBalls())
            cv::circle(destImage, ball.getCenter(), ball.getRadius(), static_cast<int>(ball.getClassification()), -1);
    }

    void EightBallPoolGame::calculateBallsTrajectory(cv::Mat& frame,
                                                     cv::Mat& previousFrame,
                                                     cv::Mat& mask,
                                                     std::map<int, cv::Point2f>& previousPositions,
                                                     cv::TermCriteria criteria,
                                                     cv::Size winSize,
                                                     int maxLevel,
                                                     cv::Scalar trajectoryColor,
                                                     float movementThreshold)
    {
        cv::Mat frameGray, previousFrameGray;
        cv::cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(previousFrame, previousFrameGray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> previousBallsPositions;
        for (const Ball& ball : this->getBalls())
            previousBallsPositions.emplace_back(ball.getCenter());

        std::vector<cv::Point2f> newBallPositions;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(previousFrameGray, frameGray, previousBallsPositions, newBallPositions, status, err, winSize, maxLevel, criteria);

        std::set<int> assignedIndices;
        for (size_t i = 0; i < newBallPositions.size(); i++) {
            if (status[i] == 1) {
                double distance = cv::norm(newBallPositions[i] - previousBallsPositions[i]);
                if (distance > movementThreshold) {
                    int closestBallIndex = -1;
                    double minDistance = std::numeric_limits<double>::max();
                    for (size_t j = 0; j < previousBallsPositions.size(); j++) {
                        if (assignedIndices.find(j) != assignedIndices.end()) continue;
                        double dist = cv::norm(newBallPositions[i] - previousBallsPositions[j]);
                        if (dist < minDistance) {
                            minDistance = dist;
                            closestBallIndex = j;
                        }
                    }

                    if (closestBallIndex != -1) {
                        assignedIndices.insert(closestBallIndex);

                        if (previousPositions.find(closestBallIndex) != previousPositions.end())
                            cv::line(mask, previousPositions[closestBallIndex], newBallPositions[i], trajectoryColor, 2);

                        previousPositions[closestBallIndex] = newBallPositions[i];
                    }
                }
            }
        }
    }
}