//-----------------------------------------------------------------------------------
// MIT License

// Copyright (c) 2023 Sakuya Ono

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//-----------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
//----------------------------------- Include ----------------------------------------
//------------------------------------------------------------------------------------
#ifndef ROS2_RS_TRACK__RS_TRACK_COMPONENT_HPP_
#define ROS2_RS_TRACK__RS_TRACK_COMPONENT_HPP_
#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <ctime>

//Particle type definition
class Particle {
    public:
    cv::Point2d pos;//position
    cv::Point2d vel;//velocity
    double like;//likelihood
    double wgt;//weight
    bool keep;//keep flag

    Particle(cv::Point2d pos, cv::Point2d vel, double l, double w, bool k);

        bool operator<(const Particle& right) const {
            return like < right.like;
        }
};

class RsTrackComponent : public rclcpp::Node
{
    public:
        RsTrackComponent();
       
        

    private:
        void timer_callback(const sensor_msgs::msg::Image::SharedPtr track_msg);
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
        size_t count_;
};

#endif 