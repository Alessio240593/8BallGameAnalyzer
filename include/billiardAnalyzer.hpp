#ifndef BILLIARDANALYZER_H
#define BILLIARDANALYZER_H

#include "opencv2/opencv.hpp"

namespace billiardAnalyzer {
    class Ball {
    public:
        enum class Classification {
            NONE,
            CUE_BALL,
            EIGHT_BALL,
            WHITE_STRIPES,
            SOLID_COLOR
        };

        Ball(cv::Point, float, Classification);
        Ball(cv::Point, float);
        cv::Point getCenter();
        void setCenter(cv::Point);
        void setCenter(int, int);
        float getRadius();
        void setRadius(float);
        Classification getClassification();
        void setClassification(Classification);
        cv::Scalar getColor();
        void classify(cv::Mat&);
        ~Ball();
    private:
        cv::Point center;
        float radius;
        Classification classification;
    };

    class Utility {
    public:
        static cv::Point2f computeIntersection(cv::Vec4i, cv::Vec4i);
        static void drawFieldAndBalls(cv::Mat&, const std::vector<billiardAnalyzer::Ball>&, const cv::Rect&);
        static void createMinimap(const cv::Mat&, const std::vector<billiardAnalyzer::Ball>&, cv::Rect);
        static void filterLinesByLength(const std::vector<cv::Vec4i>& , std::vector<cv::Vec4i>&);
    };
}

#endif