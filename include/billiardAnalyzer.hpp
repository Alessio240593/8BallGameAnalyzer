/*
 * billiardAnalyzer.hpp
 *
 * Copyright (C) 2024  Alessio Zattoni
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


#ifndef BILLIARDANALYZER_H
#define BILLIARDANALYZER_H

#include "opencv2/opencv.hpp"
#include <numeric>


namespace billiardAnalyzer {
    /**
    * This class represent a ball in the eight ball pool game
    */
    class Ball {
    public:
        enum class Classification {
            NONE,
            CUE_BALL,
            EIGHT_BALL,
            SOLID_COLOR,
            WHITE_STRIPES
        };

        /* ball box */
        static inline const uint8_t DEFAULT_BALL_BOX_THICKNESS = 2;
        static inline const float DEFAULT_BALL_BOX_INNER_GRADIENT = 0.3f;
        static inline const uint8_t BALLS_CLASSIFICATION_NUMBER = 4;

        /* classifications colors */
        static inline const cv::Scalar NONE_COLOR = cv::Scalar(0, 0, 0);
        static inline const cv::Scalar CUE_BALL_COLOR = cv::Scalar(255, 255, 255);
        static inline const cv::Scalar EIGHT_BALL_COLOR = cv::Scalar(0, 0, 0);
        static inline const cv::Scalar SOLID_COLOR_COLOR = cv::Scalar(0, 0, 255);
        static inline const cv::Scalar WHITE_STRIPES_COLOR = cv::Scalar(255, 0, 85);

        /* balls HSV ranges */
        static inline const cv::Scalar LOWER_RANGE_CUE_BALL = cv::Scalar(0, 0, 120);
        static inline const cv::Scalar UPPER_RANGE_CUE_BALL = cv::Scalar(100, 100, 255);
        static inline const cv::Scalar LOWER_RANGE_8_BALL = cv::Scalar(0, 0, 0);
        static inline const cv::Scalar UPPER_RANGE_8_BALL = cv::Scalar(250, 250, 90);
        static inline const cv::Scalar LOWER_RANGE_STRIPED_BALL = cv::Scalar(100, 100, 100);
        static inline const cv::Scalar UPPER_RANGE_STRIPED_BALL = cv::Scalar(150, 255, 255);

        /**
         * Constructor
         *
         * @param center - center of the ball
         * @param radius - radius of the ball
         * @param classification  - category of the ball
         * @return
         */
        Ball(cv::Point center, float radius, Classification classification);

        /**
         * Constructor
         *
         * @param center - center of the ball
         * @param radius - radius of the ball
         * @return
         */
        Ball(cv::Point center, float radius);

        /**
         * Destructor
         *
         * @param
         * @return
         */
        ~Ball();

        /**
         * Gets the ball center
         *
         * @param
         * @return the center of the ball
         */
        [[nodiscard]] cv::Point getCenter() const;

        /**
         * Sets the center of the ball
         *
         * @param center - center of the ball
         * @return
         */
        void setCenter(cv::Point center);

        /**
         * Sets the center of the ball
         *
         * @param x - x coordinate of the ball
         * @param y - y coordinate of the ball
         * @return
         */
        void setCenter(int x, int y);

        /**
         * Gets the ball radius
         *
         * @param
         * @return the radius of the ball
         */
        [[nodiscard]] float getRadius() const;

        /**
         * Sets the radius of the ball
         *
         * @param radius - radius of the ball
         * @return
         */
        void setRadius(float radius);

        /**
         * Gets the ball category
         *
         * @param
         * @return the category of the ball
         */
        [[nodiscard]] Classification getClassification() const;

        /**
         * Sets the category of the ball
         *
         * @param classification - category of the ball
         * @return
         */
        void setClassification(Classification classification);

        /**
         * Returns the color of the ball as cv::Scalar, based on its category
         *
         * @param
         * @return the color of the balls as cv:Scalar
         */
        [[nodiscard]] cv::Scalar getColor() const;

        /**
         * Compares the balls with respect to their category (see Ball::Classification)
         *
         * @param a - first ball category
         * @param b - second ball category
         * @return true if the first category comes before the second ones, false otherwise
         */
        static bool compareBallsByClassification(const billiardAnalyzer::Ball& a, const billiardAnalyzer::Ball& b);

        /**
         * Classifies the balls with respect to their category (see Ball::Classification)
         *
         * @param image - images that contains the balls
         * @return
         */
        void classify(cv::Mat& image);

        /**
         * Draws the boxes on the balls
         *
         * @param image - images on which draw the boxes
         * @param ballBoxThickness - the thickness of the ball box borders
         * @param ballBoxInnerGradient - the percentage of the ball box inner color gradient
         * @return
         */
        void drawBallBox(cv::Mat& image,
                         uint8_t ballBoxThickness = DEFAULT_BALL_BOX_THICKNESS,
                         float ballBoxInnerGradient = DEFAULT_BALL_BOX_INNER_GRADIENT);
    private:
        /* Center of the ball */
        cv::Point center;
        /* Radius of the ball */
        float radius;
        /* Classification of the ball */
        Classification classification;
    };

    /**
     * This class represent the pool table in the eight ball pool game
     */
    class PoolTable {
    public:
        static inline const cv::Scalar DEFAULT_POOL_TABLE_COLOR = cv::Scalar(0, 100, 0);
        static inline const cv::Scalar DEFAULT_POOL_TABLE_BORDER_COLOR = cv::Scalar(0, 255, 255);
        static inline const uint8_t DEFAULT_POOL_TABLE_THICKNESS = 2;

        /**
         * Constructor
         *
         * @param vertices - vertex of the pool table
         * @return
         */
        PoolTable(std::vector<cv::Point> vertices);

        /**
         * Constructor
         *
         * @param
         * @return
         */
        PoolTable();

        /**
         * Destructor
         *
         * @param
         * @return
         */
        ~PoolTable();

        /**
         * Gets the pool table vertices
         *
         * @param
         * @return the pool table vertices
         */
        [[nodiscard]] std::vector<cv::Point> getVertices() const;

        /**
         * Sets the pool table vertices
         *
         * @param vertices - vertices that will be set to the pool table
         * @return
         */
        void setVertices(std::vector<cv::Point> vertices);

        /**
        * Gets the pool table vertices
        *
        * @param
        * @return the pool table color
        */
        [[nodiscard]] cv::Scalar getColor() const;

        /**
         * Sets the pool table color
         *
         * @param vertices - color that will be set to the pool table
         * @return
         */
        void setColor(cv::Scalar color);

        /**
         * Creates an images that will contain only the pool table
         *
         * @param image - images from which the pool table will be extracted
         * @return the images that represent the pool table
         */
        cv::Mat getPoolTableImage(cv::Mat &image);

        /**
         * Draws the pool table boundaries into the images
         *
         * @param image - images where the boundaries will be drawn
         * @param borderColor  - color of the boundaries
         * @param borderThickness - thickness of the boundaries
         * @return
         */
        void drawPoolTable(cv::Mat& image,
                           const cv::Scalar& borderColor = DEFAULT_POOL_TABLE_BORDER_COLOR,
                           uint8_t borderThickness = DEFAULT_POOL_TABLE_THICKNESS);
    private:
        /* Vertices of the pool table */
        std::vector<cv::Point> vertices;
        /* Color of the pool table */
        cv::Scalar color;
    };

    /**
     * This class represent the eight ball pool game
     */
    class EightBallPoolGame {
    public:
        /* range used by ball detector */
        static inline const uint8_t RANGE_TOLERANCE_MODE_COLOR = 75;
        /* value used by ball detector */
        static inline const uint8_t MORPH_CLOSE_KERNEL_SIZE = 7;
        /* balls category + background + pool table */
        static inline const uint8_t TOTAL_SEGMENTATION_CLASSES = Ball::BALLS_CLASSIFICATION_NUMBER + 2;
        /* threshold used in trajectory calculations */
        static inline const float MOVEMENT_THRESHOLD  = 4.5f;
        /* trajectory default color */
        static inline const cv::Scalar TRAJECTORY_DEFAULT_COLOR = cv::Scalar (205, 74, 170);

        /**
         * Constructor
         *
         * @param balls - the balls
         * @param poolTable - the pool table
         * @return
         */
        EightBallPoolGame(std::vector<Ball> balls, PoolTable& poolTable);

        /**
         * Constructor
         *
         * @return
         */
        EightBallPoolGame();

        /**
         * Destructor
         *
         * @return
         */
        ~EightBallPoolGame();

        /**
         * Gets the balls
         *
         * @return the balls in the game
         */
        [[nodiscard]] std::vector<Ball> getBalls() const;

        /**
         * Sets the balls
         *
         * @param balls - the balls
         */
        void setBalls(std::vector<Ball> balls);

        /**
         * Gets the pool table
         *
         * @return the pool table in the game
         */
        PoolTable& getPoolTable();

        /**
         * Sets the pool table
         *
         * @param poolTable - the pool table
         */
        void setPoolTable(PoolTable& poolTable);

        /**
         * Finds the mode of the images
         *
         * @note this method is suitable to find the most present color in the images
         *       so with this color you could segment the table and then find the balls
         *
         * @param image - images of which find the mode
         * @param tableVertex - the vertices of the pool table
         * @return
         */
        cv::Vec3b findModeColor(const cv::Mat& image, const std::vector<cv::Point>& tableVertex);

        /**
         * Analyzes the eight ball pool game
         *
         * @param argc - number of the command line argument
         * @param argv - the command line arguments
         * @return
         */
        static void analyzingEightBallPoolGame(int argc, char** argv);

        /**
         * Analyzes the eight ball pool game
         *
         * @param path - the path of the video/images
         * @param showTableDetection - the option to show the table in the video/images
         * @param showBallsDetection - the option to show the balls in the video/images
         * @param showSegmentation - the option to show the segmentation in the video/images
         * @param showMinimap - the option to show the minimap in the video/images
         * @param pathToGtmAP - the option to calculate the mAp whit given gt file
         * @param pathToMaskmIoU - the option to calculate mIoU with give mask image
         * @return
         */
        static void analyzingEightBallPoolGame(const std::string& path,
                                               bool showTable,
                                               bool showBalls,
                                               bool showSegmentation,
                                               bool showMinimap,
                                               const std::string& pathToGtmAP = "",
                                               const std::string& pathToMaskmIoU = "");

        /**
         * Create a gray scale segmentation mask with following pixel value:
         * 0 - background
         * 1 - cue ball
         * 2 - eight ball
         * 3 - stripped ball
         * 4 - color ball
         * 5 - pool table
         *
         * @param srcImage - source images
         * @param destImage - destination images
         */
        void createSegmentationMask(cv::Mat& srcImage,
                                    cv::Mat& destImage);
    private:
        /* balls */
        std::vector<Ball> balls;
        /* pool table */
        PoolTable poolTable;

        /**
         * Prints the balls information:
         * 1)ordinate of the top left corner
         * 2)abscissa of the top left corner
         * 3)the box width
         * 4)the box height
         * 5)the ball category
         *
         * @param balls - the balls of which you would like to print the information
         *@return
         */
        void printBallsInformation();

        /**
         * Filters the contours in order to keep only the balls ones
         *
         * @param contours - the contours found in the images
         * @param alpha - parameter used as tolerance
         * @return the filtered contour
         */
        static std::vector<std::vector<cv::Point>> filterContours(const std::vector<std::vector<cv::Point>>& contours, double alpha = 3.445);

        /**
         * Process a video of the eight ball pool game
         *
         * @param path - the path of the video
         * @param showTableDetection - the option to show the table in the video
         * @param showBallsDetection - the option to show the balls in the video
         * @param showSegmentation - the option to show the segmentation in the video
         * @param showMinimap - the option to show the minimap in the video
         * @param pathToGtmAP - the option to calculate the mAp whit given gt file
         * @param pathToMaskmIoU - the option to calculate mIoU with give mask image
         */
        static void processVideo(const std::string& path,
                                 bool showTableDetection,
                                 bool showBallsDetection,
                                 bool showSegmentation,
                                 bool showMinimap,
                                 const std::string& pathToGtmAP,
                                 const std::string& pathToMaskmIoU);

        /**
         * Process a images of the eight ball pool game
         *
         * @param path - the path of the images
         * @param showTableDetection - the option to show the table in the images
         * @param showBallsDetection - the option to show the balls in the images
         * @param showSegmentation - the option to show the segmentation in the images
         * @param showMinimap - the option to show the minimap in the images
         * @param pathToGtmAP - the option to calculate the mAp whit given gt file
         * @param pathToMaskmIoU - the option to calculate mIoU with give mask image
         */
        static void processImage(const std::string& path,
                                 bool showTableDetection,
                                 bool showBallsDetection,
                                 bool showSegmentation,
                                 bool showMinimap,
                                 const std::string& pathToGtmAP,
                                 const std::string& pathToMaskmIoU);

        /**
         * Executes the analysis of the eight ball pool game
         *
         * @param frame - the current frame
         * @param previousFrame - the previous frame
         * @param previousPositions - the position of the ball in the previous frame
         * @param mask - the mask with the trajectory
         * @param eightBallPoolGame - the eight ball pool game object
         * @param showTableDetection - the option to show the table in the images
         * @param showBallsDetection - the option to show the balls in the images
         * @param showSegmentation - the option to show the segmentation in the images
         * @param showMinimap - the option to show the minimap in the images
         * @param pathToGtmAP - the option to calculate the mAp whit given gt file
         * @param pathToMaskmIoU - the option to calculate mIoU with give mask image
         */
        static void executeAnalysis(cv::Mat& frame,
                                    cv::Mat& previousFrame,
                                    std::map<int, cv::Point2f>& previousPositions,
                                    cv::Mat& mask,
                                    EightBallPoolGame* eightBallPoolGame,
                                    bool showTableDetection,
                                    bool showBallsDetection,
                                    bool showSegmentation,
                                    bool showMinimap,
                                    const std::string& pathToGtmAP,
                                    const std::string& pathToMaskmIoU);

        /**
         * Tries to detect the pool table in the frame
         *
         * @param image - images in which the table is present
         * @param tolerance - the range of tolerance for the search of the pool table mode
         * @return the points that depict the table
         */
        std::vector<cv::Point> detectPoolTable(cv::Mat &image, int tolerance = RANGE_TOLERANCE_MODE_COLOR);

        /**
         * Refines the pool table starting to a raw representation of the table found by detectPoolTable
         *
         * @param image - images on which refine the pool table
         * @param rangeTolerance - tolerance in the range of the table
         * @param morphCloseSize - size of the squared kernel used in morphological closure in the method
         * @note rangeTolerance controls the segmentations process, higher the value more pixel will be included in the
         *       same category of the table so the images result in few regions.
         *       morphCloseSize instead controls the closure that will be applied in the method in order to isolate the contours, so
         *       greater the value greater the isolation of the balls but the shape will begin to warp
         * @return the improved points that represent the pool table
         */
        std::vector<cv::Point> refinePoolTable(cv::Mat& image,
                                               uint8_t rangeTolerance = RANGE_TOLERANCE_MODE_COLOR - 15,
                                               uint8_t morphCloseSize = MORPH_CLOSE_KERNEL_SIZE - 4);

        /**
         * Tries to find the balls in the frame
         *
         * @param image - images in which the balls are present
         * @param rangeTolerance - tolerance in the range of the table
         * @param morphCloseSize - size of the squared kernel used in morphological closure in the method
         * @note rangeTolerance controls the segmentations process, higher the value more pixel will be included in the
         *       same category of the table so the images result in few regions.
         *       morphCloseSize instead controls the closure that will be applied in the method in order to isolate the contours, so
         *       greater the value greater the isolation of the balls but the shape will begin to warp
         * @return the balls
         */
        std::vector<billiardAnalyzer::Ball> detectBalls(cv::Mat & image,
                                                        uint8_t rangeTolerance = RANGE_TOLERANCE_MODE_COLOR,
                                                        uint8_t morphCloseSize = MORPH_CLOSE_KERNEL_SIZE);

        /**
         * Segments the area inside the pool table
         *
         * @param image - the images where show the segmentations
         */
        void segmentsImage(cv::Mat& image);

        /**
         * Calculates the trajectory of the balls in the game frame by frame
         *
         * @param frame - the current frame
         * @param previousFrame - the previous frame
         * @param mask - the mask with trajectory
         * @param previousPositions - previous ball center positions
         * @param criteria - calcOpticalFlowPyrLK algorithm criteria
         * @param winSize - calcOpticalFlowPyrLK window size
         * @param maxLevel - calcOpticalFlowPyrLK max level
         * @param trajectoryColor - trajectory color
         * @param movementThreshold - threshold used to avoid false positive trajectory
         */
        void calculateBallsTrajectory(cv::Mat& frame,
                                      cv::Mat& previousFrame,
                                      cv::Mat& mask,
                                      std::map<int, cv::Point2f>& previousPositions,
                                      cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10, 0.03),
                                      cv::Size winSize = cv::Size(20, 20),
                                      int maxLevel = 2,
                                      cv::Scalar trajectoryColor = TRAJECTORY_DEFAULT_COLOR,
                                      float movementThreshold = MOVEMENT_THRESHOLD);
    };

    /**
     * This class is used to compute some statistics, it represents the coordinate and the size of the box
     */
    class Detection
    {
    public:
        /**
         * Constructor
         */
        Detection();

        /**
         * Constructor
         *
         * @param x - top left corner ordinate of the box
         * @param y - top left corner abscissa of the box
         * @param width - width of the box
         * @param height - height of the box
         * @param category - category of the box
         */
        Detection(int x,
                  int y,
                  int width,
                  int height,
                  int category);

        /**
         * Destructor
         */
        ~Detection();

        /**
         * Gets the ordinate of the top left box corner
         *
         * @return the ordinate of the top left box corner
         */
        [[nodiscard]] int getX() const;

        /**
         * Sets the ordinate of the top left box corner
         *
         * @param x - the ordinate of the top left box corner
         * @return
         */
        void setX(int x);

        /**
         * Gets the abscissa of the top left box corner
         *
         * @return the abscissa of the top left box corner
         */
        [[nodiscard]] int getY() const;

        /**
         * Sets the abscissa of the top left box corner
         *
         * @param y - the abscissa of the top left box corner
         * @return
         */
        void setY(int y);

        /**
         * Gets the width of the box
         *
         * @return the width of the box
         */
        [[nodiscard]] int getWidth() const;

       /**
       * Sets the width of the box
       *
        * @param width - the width of the box
       * @return
       */
        void setWidth(int width);

        /**
         * Gets the height of the box
         *
         * @return the height of the box
         */
        [[nodiscard]] int getHeight() const;

        /**
        * Sets the height of the box
        *
         * @param height - the height of the box
        * @return
        */
        void setHeight(int height);

        /**
         * Gets the category of the detection
         *
         * @return the category of the detection
         */
        [[nodiscard]] int getCategory() const;

        /**
        * Sets the category of the detection
        *
         * @param category - the category of the prediction
        * @return
        */
        void setCategory(int category);

        /**
         * Gets the confidence of the prediction
         *
         * @return the confidence of the prediction
         */
        [[nodiscard]] float getConfidence() const;

        /**
        * Sets the confidence of the prediction
        *
         * @param confidence - the confidence of the prediction
        * @return
        */
        void setConfidence(float confidence);

        /**
         * Converts the category into the correspondent string
         *
         * @param classificationID - the id of the category
         * @return the representation as string of the category
         */
        static std::string fromSegmentationCategoryToString(int classificationID);

    private:
        /* ordinate of the top left corner of the box */
        int x;
        /* abscissa of the top left corner of the box */
        int y;
        /* width of the box */
        int width;
        /* height of the box */
        int height;
        /* category of the detection */
        int category;
        /* the value of confidence in the detection */
        float confidence;
    };

    /**
    * This class is a collection of utility methods
    */
    class Utility {
    public:
        /**
         * Checks if hsvPixel is in the ranges passed as parameters
         *
         * @param lowerRange - lower range used in the comparison
         * @param upperRange - upper range used in the comparison
         * @param hsvPixel - the pixel to control
         * @return true if the pixel is in the range, false otherwise
         */
        static bool isInRange(const cv::Scalar& lowerRange,
                              const cv::Scalar& upperRange,
                              cv::Vec3b hsvPixel);

        /**
         * Checks if the path is related to a video
         *
         * @param path - path to the video
         * @return true if the path refers to a video, false otherwise
         */
        static bool isVideoFile(const std::string& path);

        /**
         * Checks if the path is related to an images
         *
         * @param path - path to the images
         * @return true if the path refers to an images, false otherwise
         */
        static bool isImageFile(const std::string& path);

        /**
         * Converts string to bool
         *
         * @param str - string to convert
         * @return the boolean representation of string
         */
        static bool stringToBool(const std::string& str);

        /**
         * Prints the help of the application
         *
         * @param
         * @return
         */
        static void printUsage();

        /**
         * Parses the command line arguments and set the application options
         *
         * @param argc - number of command line arguments
         * @param argv - the command line arguments
         * @param path - the path of the video/images
         * @param showTableDetection - the option to show the table in the video/images
         * @param showBallsDetection - the option to show the balls in the video/images
         * @param showSegmentation - the option to show the segmentation in the video/images
         * @param showMinimap - the option to show the minimap in the video/images
         * @param pathToGtmAP - the option to calculate the mAp whit given gt file
         * @param pathToMaskmIoU - the option to calculate mIoU with give mask image
         */
        static void parseCommandLineArguments(int argc,
                                              char **argv,
                                              std::string& path,
                                              bool& showTableDetection,
                                              bool& showBallsDetection,
                                              bool& showSegmentation,
                                              bool& showMinimap,
                                              std::string& pathToGtmAP,
                                              std::string& pathToMaskmIoU);

        /**
         * Creates and draws the minimap that represent the play
         *
         * @param image - the images where draw the minimap
         * @param mask - the mask with the trajectories
         * @param eightBallPoolGame - the object that represent the game
         * @return
         */
        static void createMinimap(const cv::Mat& image,
                                  cv::Mat& mask,
                                  billiardAnalyzer::EightBallPoolGame& eightBallPoolGame);

        /**
         * Draws the pool table boundaries and the balls
         *
         * @param image - images where draw the balls
         * @param eightBallPoolGame - the object that represent the game
         */
        static void drawPoolTableAndBalls(cv::Mat& image,
                                          EightBallPoolGame& eightBallPoolGame);

        /**
         * Compute IoU (intersection over union) given two gray scale mask
         *
         * @param groundTruth - ground truth gray scale mask
         * @param predictedBall - predictedBall gray scale mask
         * @param classificationID - category of which calculate IoU
         * @return the IoU of the category identify by classificationID value
         */
        static double computeIoUByMask(const cv::Mat& groundTruth,
                                       const cv::Mat& predictedBall,
                                       int classificationID);

        /**
         * Compute IoU (intersection over union)
         *
         * @param pred - the structure that contains the predicted box position and size
         * @param gt - the structure that contains the position and dimension of the ground thruth box
         * @return the IoU
         */
        static float calculateIoU(Detection pred, Detection gt);

        /**
         * Compute the mIoU (mean of the intersection over union)
         *
         * @param predictedBall - the images on which calculate mIoU
         * @param groundTruth - the ground truth images as gray scale mask
         * @param eightBallPoolGame - the contest of the game
         * @return the mean of the intersection over union of all the categories
         */
        static float computeMIoU(cv::Mat& predictedBall,
                                 cv::Mat& groundTruth,
                                 EightBallPoolGame* eightBallPoolGame);

        /**
         * Converts the ground truths and the predicted balls into a class used in the calculation of the statistics
         *
         * @param inputFile - the input file stream of the ground truths file
         * @param groundTruthBalls - the vector of ground truths values used for the statistics
         * @param balls - the vector of predicted balls
         * @param predictedBalls - the vector of predicted balls used for the statistics
         */
        static void prepareDetectionStructure(std::ifstream& inputFile,
                                              std::vector<Detection>& groundTruthBalls,
                                              std::vector<Ball> balls,
                                              std::vector<Detection>& predictedBalls);

        /**
         * Calculate the precision and the recall of the category identify by categoryID
         *
         * @param predictedBalls - the vector of the predicted balls
         * @param groundTruths - the balls ground truths
         * @param categoryID - the category of which calculate precision and recall
         * @param precision - the vector of precisions
         * @param recall - the vector of recalls
         * @return
         */
        static void calculatePrecisionRecall(std::vector<Detection>& predictedBalls,
                                             std::vector<Detection>& groundTruths,
                                             int categoryID,
                                             std::vector<float>& precision,
                                             std::vector<float>& recall);

        /**
         * Calculate the AP (average precision)
         *
         * @param precision - the precision of the category
         * @param recall - the recall of the category
         * @return the AP of the category
         */
        static float calculateAP(const std::vector<float>& precision, const std::vector<float>& recall);

        /**
         * Calculate the mAP (mean of average precision) given the predicted balls and the relative ground truth
         *
         * @param groundTruthBalls - the vector of the ground truth values
         * @param predictedBalls - the vector of the predicted balls
         * @return the value of mAP
         */
        static float calculatemAP(std::vector<Detection> groundTruthBalls, std::vector<Detection> predictedBalls);
    };
}

#endif