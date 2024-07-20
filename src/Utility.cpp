/*
 * Utility.cpp
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


#include <fstream>
#include "billiardAnalyzer.hpp"

namespace billiardAnalyzer
{
    bool Utility::isInRange(const cv::Scalar& lowerRange,
                            const cv::Scalar& upperRange,
                            cv::Vec3b hsvPixel)

    {
        int h = hsvPixel[0];
        int s = hsvPixel[1];
        int v = hsvPixel[2];

        return h >= lowerRange[0] && h <= upperRange[0] &&
               s >= lowerRange[1] && s <= upperRange[1] &&
               v >= lowerRange[2] && v <= upperRange[2];
    }

    bool Utility::isVideoFile(const std::string& path)
    {
        cv::VideoCapture cap(path);
        return cap.isOpened();
    }

    bool Utility::isImageFile(const std::string& path)
    {
        cv::Mat image = cv::imread(path);
        return !image.empty();
    }

    bool Utility::stringToBool(const std::string& str)
    {
        return str == "1" || str == "true" || str == "True";
    }

    void Utility::printUsage()
    {
        std::cout << "Usage: ./your_program_name [options]" << std::endl;
        std::cout << "Options:" << std::endl;
        std::cout << "  -path <value>                   Specify the path to the images or video." << std::endl;
        std::cout << "  -showTable <true/false>         Show or hide table detection (default: false)." << std::endl;
        std::cout << "  -showBalls <true/false>         Show or hide ball detection (default: false)." << std::endl;
        std::cout << "  -showSegmentation <true/false>  Show or hide segmentation (default: false)." << std::endl;
        std::cout << "  -showMinimap <true/false>       Show or hide minimap (default: false)." << std::endl;
        std::cout << "  -mAP <value>                    Specify the path to the ground truth file." << std::endl;
        std::cout << "  -mIoU <value>                   Specify the path to the mask image." << std::endl;
        std::cout << std::endl;
    }

    void Utility::parseCommandLineArguments(int argc,
                                            char **argv,
                                            std::string& path,
                                            bool& showTableDetection,
                                            bool& showBallsDetection,
                                            bool& showSegmentation,
                                            bool& showMinimap,
                                            std::string& pathToGtmAP,
                                            std::string& pathToMaskmIoU)
    {
        for (int i = 1; i < argc; i += 2) {
            std::string arg = argv[i];

            if (arg == "-path" && i + 1 < argc) {
                std::string value = argv[i + 1];
                path = value;
            }
            else if (arg == "-showTable" && i + 1 < argc) {
                std::string value = argv[i + 1];
                showTableDetection = stringToBool(value);
            }
            else if (arg == "-showBalls" && i + 1 < argc) {
                std::string value = argv[i + 1];
                showBallsDetection = stringToBool(value);
            }
            else if (arg == "-showSegmentation" && i + 1 < argc) {
                std::string value = argv[i + 1];
                showSegmentation = stringToBool(value);
            }
            else if (arg == "-showMinimap" && i + 1 < argc) {
                std::string value = argv[i + 1];
                showMinimap = stringToBool(value);
            }
            else if (arg == "-mAP" && i + 1 < argc) {
                std::string value = argv[i + 1];
                pathToGtmAP = value;
            }
            else if (arg == "-mIoU" && i + 1 < argc) {
                std::string value = argv[i + 1];
                pathToMaskmIoU = value;
            }
            else {
                printUsage();
                throw std::invalid_argument(arg + " isn't valid argument");
            }
        }
    }

    void Utility::createMinimap(const cv::Mat& image,
                                cv::Mat& mask,
                                billiardAnalyzer::EightBallPoolGame& eightBallPoolGame)
    {
        if (image.empty())
            throw std::invalid_argument("Image should not be empty");

        if (mask.empty()) {
            std::cerr << "Warning: The mask is empty." << std::endl;
            // Optionally create a blank mask
            mask = cv::Mat::zeros(image.size(), CV_8UC3);
        }

        cv::Mat miniMap = cv::Mat::zeros(cv::Size(image.cols, image.rows), CV_8UC3);
        miniMap = cv::Scalar(255, 255, 255);

        drawPoolTableAndBalls(miniMap, eightBallPoolGame);

        cv::rectangle(miniMap,
                      cv::Point(0, 0),
                      cv::Point(miniMap.cols - 1, miniMap.rows - 1),
                      cv::Scalar(0, 0, 0),
                      50);

        cv::Mat resizedMiniMap;
        cv::resize(miniMap, resizedMiniMap, cv::Size(), 0.3, 0.3);

        cv::Rect miniMapRect(0, image.rows - resizedMiniMap.rows, resizedMiniMap.cols, resizedMiniMap.rows);
        cv::Mat miniMapRegion = image(miniMapRect);

        cv::Mat resizedMask;
        cv::resize(mask, resizedMask, resizedMiniMap.size());

        cv::add(resizedMiniMap, resizedMask, miniMapRegion);
    }

    void Utility::drawPoolTableAndBalls(cv::Mat& image, EightBallPoolGame& eightBallPoolGame)
    {
        if (image.empty())
            throw std::invalid_argument("Image should not be empty");

        if (eightBallPoolGame.getPoolTable().getVertices().empty())
            throw std::invalid_argument("Pool table vertexes are empty are empty");

        if (eightBallPoolGame.getBalls().empty())
            throw std::invalid_argument("Balls are empty");

        cv::Scalar poolTableColor =  eightBallPoolGame.getPoolTable().getColor();

        if (poolTableColor == cv::Scalar(0, 0, 0, 0))
            poolTableColor = PoolTable::DEFAULT_POOL_TABLE_COLOR;

        cv::fillConvexPoly(image, eightBallPoolGame.getPoolTable().getVertices(), poolTableColor);

        for (Ball ball : eightBallPoolGame.getBalls()) {
            cv::Point originalCenter(ball.getCenter().x, ball.getCenter().y);
            cv::circle(image, originalCenter, ball.getRadius(), ball.getColor(), -1);
        }
    }

    double Utility::computeIoUByMask(const cv::Mat& groundTruth,
                                     const cv::Mat& predictedBall,
                                     int classificationID)
    {
        if (groundTruth.empty() || predictedBall.empty()) {
            throw std::invalid_argument("Input masks must not be empty.");
        }

        if (groundTruth.size() != predictedBall.size()) {
            throw std::invalid_argument("Input masks must have the same size.");
        }

        cv::Mat gt = (groundTruth == classificationID);
        cv::Mat pred = (predictedBall == classificationID);

        gt.convertTo(gt, CV_8U);
        pred.convertTo(pred, CV_8U);

        cv::Mat intersection = gt & pred;
        cv::Mat unionArea = gt | pred;

        int intersectionCount = cv::countNonZero(intersection);
        int unionCount = cv::countNonZero(unionArea);

        return (unionCount == 0) ? 0.0 : static_cast<double>(intersectionCount) / unionCount;
    }

    float Utility::calculateIoU(Detection pred, Detection gt)
    {
        float x1Inter = std::max(pred.getX(), gt.getX());
        float y1Inter = std::max(pred.getY(), gt.getY());
        float x2Inter = std::min(pred.getX() + pred.getWidth(), gt.getX() + gt.getWidth());
        float y2Inter = std::min(pred.getY() + pred.getHeight(), gt.getY() + gt.getHeight());

        float widthInter = std::max(0.0f, x2Inter - x1Inter);
        float heightInter = std::max(0.0f, y2Inter - y1Inter);

        float intersection = widthInter * heightInter;

        float areaPred = pred.getWidth() * pred.getHeight();
        float areaGt = gt.getWidth() * gt.getHeight();

        float unionArea = areaPred + areaGt - intersection;

        if (unionArea == 0) return 0.0f;

        return intersection / unionArea;
    }

    float Utility::computeMIoU(cv::Mat& predictedBall,
                               cv::Mat& groundTruth,
                               EightBallPoolGame* eightBallPoolGame)
    {
        cv::Mat seg;
        eightBallPoolGame->createSegmentationMask(predictedBall, seg);

        double mIoU = 0.0;

        for (int classificationID = 0; classificationID < EightBallPoolGame::TOTAL_SEGMENTATION_CLASSES; ++classificationID) {
            double iou = computeIoUByMask(groundTruth, seg, classificationID);
            mIoU += iou;
            std::cout << "IoU for class " << Detection::fromSegmentationCategoryToString(classificationID) << ": " << std::fixed << std::setprecision(2) << iou * 100 << std::endl;
        }

        return mIoU / EightBallPoolGame::TOTAL_SEGMENTATION_CLASSES;
    }

    void Utility::prepareDetectionStructure(std::ifstream& inputFile,
                                            std::vector<Detection>& groundTruthBalls,
                                            std::vector<Ball> balls,
                                            std::vector<Detection>& predictedBalls)
    {
        int x, y, width, height, classification;

        while (inputFile >> x >> y >> width >> height >> classification) {
            groundTruthBalls.emplace_back(x,
                                          y,
                                          width,
                                          height,
                                          classification);
        }

        for(Ball& ball : balls) {
            predictedBalls.emplace_back(ball.getCenter().x - ball.getRadius(),
                                        ball.getCenter().y - ball.getRadius(),
                                        static_cast<int>(ball.getRadius() * 2),
                                        static_cast<int>(ball.getRadius() * 2),
                                        static_cast<int>(ball.getClassification()));
        }
    }

    void Utility::calculatePrecisionRecall(std::vector<Detection>& predictedBalls,
                                           std::vector<Detection>& groundTruths,
                                           int categoryID,
                                           std::vector<float>& precision,
                                           std::vector<float>& recall)
   {
        std::vector<Detection> classDetections;
        std::vector<Detection> classGroundTruths;
        for (auto& det : predictedBalls) {
            if (det.getCategory() == categoryID) {
                classDetections.push_back(det);
            }
        }
        for (auto& gt : groundTruths) {
            if (gt.getCategory() == categoryID) {
                classGroundTruths.push_back(gt);
            }
        }

        std::sort(classDetections.begin(), classDetections.end(), [](Detection a, Detection b) {
            return a.getConfidence() > b.getConfidence();
        });

        std::vector<int> tp(classDetections.size(), 0);
        std::vector<int> fp(classDetections.size(), 0);
        std::vector<bool> gtMatched(classGroundTruths.size(), false);

        for (size_t i = 0; i < classDetections.size(); ++i) {
            bool matchFound = false;
            for (size_t j = 0; j < classGroundTruths.size(); ++j) {
                if (!gtMatched[j] && Utility::calculateIoU(classDetections[i], classGroundTruths[j]) >= 0.5) {
                    tp[i] = 1;
                    gtMatched[j] = true;
                    matchFound = true;
                    break;
                }
            }
            if (!matchFound) {
                fp[i] = 1;
            }
        }

        for (size_t i = 1; i < classDetections.size(); ++i) {
            tp[i] += tp[i - 1];
            fp[i] += fp[i - 1];
        }

        precision.resize(classDetections.size());
        recall.resize(classDetections.size());
        for (size_t i = 0; i < classDetections.size(); ++i) {
            precision[i] = static_cast<float>(tp[i]) / (tp[i] + fp[i]);
            recall[i] = static_cast<float>(tp[i]) / classGroundTruths.size();
        }
    }

   float Utility::calculateAP(const std::vector<float>& precision, const std::vector<float>& recall)
   {
       float ap = 0.0;
       for (float t = 0.0; t <= 1.0; t += 0.1) {
           float maxPrec = 0.0;

           for (size_t i = 0; i < precision.size(); ++i) {
               if (recall[i] >= t) {
                   maxPrec = std::max(maxPrec, precision[i]);
               }
           }
           ap += maxPrec;
       }
       return ap / 11.0;
   }

   float Utility::calculatemAP(std::vector<Detection> groundTruthBalls, std::vector<Detection> predictedBalls)
   {
       std::map<int, float> classesAP;
       for (int classificationID = 1; classificationID <= 4; ++classificationID) {
           std::vector<float> precision, recall;
           calculatePrecisionRecall(predictedBalls, groundTruthBalls, classificationID, precision, recall);

           float ap = calculateAP(precision, recall);
           classesAP[classificationID] = ap;

           std::cout << "AP for class " << Detection::fromSegmentationCategoryToString(classificationID) << ": " << std::fixed << std::setprecision(2) << ap * 100 << "%" << std::endl;
       }

       float sumAp = std::accumulate(classesAP.begin(), classesAP.end(), 0.0,
                                     [](float sum, const std::pair<int, float>& p) {
                                          return sum + p.second;
                                      });

       return sumAp / classesAP.size();
   }
}