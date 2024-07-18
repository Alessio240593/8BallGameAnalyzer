/*
 * Ball.cpp
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

namespace billiardAnalyzer
{
    Detection::Detection() = default;

    Detection::Detection(int x,
                         int y,
                         int width,
                         int height,
                         int category)
                         :
                         x(x), y(y), width(width), height(height), category(category), confidence(0.0) {}

    Detection::~Detection() = default;

    int Detection::getX() const
    {
        return x;
    }

    void Detection::setX(int x)
    {
        this->x = x;
    }

    int Detection::getY() const
    {
        return y;
    }

    void Detection::setY(int y)
    {
        this->y = y;
    }

    int Detection::getWidth() const
    {
        return width;
    }

    void Detection::setWidth(int width)
    {
        this->width = width;
    }

    int Detection::getHeight() const
    {
        return height;
    }

    void Detection::setHeight(int height)
    {
        this->height = height;
    }

    int Detection::getCategory() const
    {
        return category;
    }

    void Detection::setCategory(int category)
    {
        this->category = category;
    }

    float Detection::getConfidence() const
    {
        return confidence;
    }

    void Detection::setConfidence(float confidence)
    {
        this->confidence = confidence;
    }

    std::string Detection::fromSegmentationCategoryToString(int classificationID)
    {
        switch (classificationID) {
            case 0:
                return "background";
            case 1:
                return "cue ball";
            case 2:
                return "eight ball";
            case 3:
                return "solid color";
            case 4:
                return "white stripes";
            case 5:
                return "pool table";
            default:
                return "";
        }
    }
}