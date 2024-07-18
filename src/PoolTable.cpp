/*
 * PoolTable.cpp
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


#include "billiardAnalyzer.hpp"

namespace billiardAnalyzer {
    PoolTable::PoolTable(std::vector<cv::Point> vertices) : vertices(std::move(vertices)) {}

    PoolTable::PoolTable() = default;

    PoolTable::~PoolTable() = default;

    std::vector<cv::Point> PoolTable::getVertices() const
    {
        return this->vertices;
    }

    void PoolTable::setVertices(std::vector<cv::Point> vertices)
    {
        this->vertices = vertices;
    }

    cv::Scalar PoolTable::getColor() const
    {
        return this->color;
    }

    void PoolTable::setColor(cv::Scalar color)
    {
        this->color = color;
    }

    cv::Mat PoolTable::getPoolTableImage(cv::Mat& image)
    {
        if (image.empty())
            throw std::invalid_argument("Image should not be empty");

        if (this->vertices.empty())
            throw std::invalid_argument("Table vertex should not be empty");

        std::vector<std::vector<cv::Point>> hulls(1, this->vertices);

        cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::fillPoly(mask, hulls, cv::Scalar(255));

        cv::Mat result = cv::Mat::zeros(image.size(), image.type());
        image.copyTo(result, mask);

        cv::Rect boundingBox = cv::boundingRect(this->vertices);
        if (boundingBox.x < 0 || boundingBox.y < 0 ||
            boundingBox.x + boundingBox.width > image.cols ||
            boundingBox.y + boundingBox.height > image.rows) {
            return {};
        }
        else {
            cv::Mat table = result(boundingBox);
            return table;
        }
    }

    void PoolTable::drawPoolTable(cv::Mat& image,
                                  const cv::Scalar& borderColor,
                                  uint8_t borderThickness)
    {
        if (image.empty())
            throw std::invalid_argument("Image should not be empty");

        if (this->vertices.empty())
            throw std::invalid_argument("Table vertices should not be empty");

        std::vector<std::vector<cv::Point>> hulls(1, this->vertices);
        cv::polylines(image, hulls, true, borderColor, borderThickness);
    }
}