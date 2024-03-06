
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

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <string>

//------------------------------------------------------------------------------------
//---------------------------- Include Thrid Party -----------------------------------
//------------------------------------------------------------------------------------

#include "ros2_pso_track/rs_track_component.hpp"

using sensor_msgs::msg::Image;
using namespace cv;
using namespace std::chrono_literals;



RsTrackComponent::RsTrackComponent() : Node("rs_track")
{
    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/color/image_raw",
        10,
        std::bind(&RsTrackComponent::timer_callback, this, std::placeholders::_1)
    );
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "track_image",
        10
    );

    }

    Particle::Particle(cv::Point2d p, cv::Point2d v, double l, double w, bool k)
    {
        pos = p;
        vel = v;
        like =  l;
        wgt = w;
        keep = k;
    }
    
    void RsTrackComponent::timer_callback(const sensor_msgs::msg::Image::SharedPtr img)
    {

        cv_bridge::CvImagePtr cv_image;
        try{
            cv_image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        auto track_msg = sensor_msgs::msg::Image();
        if (cv_image -> image.empty()) return;
        cv::waitKey(1);
        cv::Mat img_src, img_hsv;
        std::vector<cv::Mat> vec_hsv(3);
        std::vector<Particle> P;
        cv::RNG rng((unsigned int)time(NULL));
        int num = 1000;// Particle volume

        img_src = cv_image->image;

        
        //Initialization of particles, uniformly distributed across the screen
        //Initial likelihood 1.0, initial weight 1.0
        for (int i = 0; i < num; i++){

            cv::Point2d pt(
                rng.uniform(0, img_src.cols), rng.uniform(0, img_src.rows)
            );
            Particle p(pt, cv::Point2d(0.0, 0.0), 1.0, 0.0, false);
            P.push_back(p);
        }

        cv::Point2d center(img_src.cols / 2, img_src.rows / 2);

            //prediction
            for (int i = 0; i < P.size(); i++){
                P.at(i).pos += P.at(i).vel;
                 }

            img_src = cv_image->image;
        cv::cvtColor(img_src, img_hsv, cv::COLOR_BGR2HSV);//hsv conversion, h from 0~180,s and v from 0~255
        cv::split(img_hsv, vec_hsv);//hsv separation

        //Calculate and update the likelihood from the hue and saturation values
        for (int i = 0; i < P.size(); i++) {
        if (0 < P.at(i).pos.x && P.at(i).pos.x < img_src.cols
        && 0 < P.at(i).pos.y && P.at(i).pos.y < img_src.rows
        )
        {
            int h = vec_hsv[0].at<unsigned char>(P.at(i).pos);//Obtain hue values for particle pixels
            int s = vec_hsv[1].at<unsigned char>(P.at(i).pos);// Get the saturation value of the particle pixel

            double len_h = abs(70 - h);// h=Distance from 70
            double len_s = abs(200 - s);// s=Distance from 70
            double like = (len_h /180)*0.8 + (len_s / 255)*0.2;//likelihood function
            P.at(i).like = 1 - like;//Maximum likelihood set as 1
            }

            else{
                P.at(i).like = 0;
            }
        }

        //Sort likelihood in ascending order
        sort(P.begin(), P.end());


        //Leave only particles with high likelihood and eliminate particles with low likelihood
        double thresh_like = 0.9;//likelihood threshold
        int thresh_keep = P.size() / 100;//% of particles keeping
        for (int i = 0; i < P.size(); i++){
            if (P.at(i).like > thresh_like || i > (P.size() - thresh_keep)) P.at(i).keep = true;
            else P.at(i).keep = false;
        }
        std::vector<Particle>::iterator it = P.begin();
        while ( it != P.end())
        {

            if ((*it).keep) it++;
            else it = P.erase(it);
        }


        //Counting of particles with high likelihood, sum of likelihoods
        int count = P.size();
        double l_sum = 0.0;
        for (int i = 0; i < P.size(); i++){
            l_sum += P.at(i).like;
        }
        //Calculate normalized weights
        for (int i = 0; i < P.size(); i++)
        {
            P.at(i).wgt = P.at(i).like / l_sum;

        }
        //resampling
        std::vector<Particle> Pnew;
        for (int i = 0; i < P.size(); i++){

            int num_new = P.at(i).wgt * (num - P.size());
            for (int j = 0; j < num_new; j++){

                double r = rng.gaussian(img_src.rows + img_src.cols) * (1 - P.at(i).like);
                double ang = rng.uniform(-M_PI, M_PI);
                cv::Point2d pt(r*cos(ang) + P.at(i).pos.x, r*sin(ang) + P.at(i).pos.y);
                Particle p(pt, pt - P.at(i).pos, P.at(i).like, P.at(i).wgt, false);

                //Assuming constant velocity linear motion of particles
                Pnew.push_back(p);
            }
        }

        std::copy(Pnew.begin(), Pnew.end(), std::back_inserter(P));
        
        //Drawing particles
        for (int i = 0; i < P.size(); i++) {

            if (0 < P.at(i).pos.x && P.at(i).pos.x < img_src.cols && 0 < P.at(i).pos.y && P.at(i).pos.y < img_src.rows){

                cv::circle(img_src, P.at(i).pos, 2, cv::Scalar(0, 0, 255));

            }
        }
        
        //Draw additional particles
        for (int i = 0; i < Pnew.size(); i++){
            cv::circle(img_src, Pnew.at(i).pos, 2, cv::Scalar(255, 0, 0));
        }

        //Particle center of gravity
        for (int i = 0; i < P.size(); i++){
            center += P.at(i).pos;

        }

        center *= 1.0 / P.size();
        cv::line(img_src, cv::Point(center.x, 0), cv::Point(center.x, img_src.rows), cv::Scalar(0, 255, 255), 3);

        cv::line(img_src, cv::Point(0, center.y), cv::Point(img_src.cols, center.y), cv::Scalar(0, 255, 255), 3);
        cv::Mat img_rgb;
        cv::cvtColor(img_src, img_rgb, cv::COLOR_BGR2RGB);
        cv::imshow("track", img_rgb);


        //Option : If you using ROS 2 Topic for visualization, please activate the below.
        //cv_bridge::CvImage cv_img;
        //cv_img.encoding = "bgr8";

        //img_src.copyTo(cv_img.image);
        //cv_img.header.stamp = this ->now();
        //cv_img.toImageMsg(track_msg);
        //publisher_->publish(std::move(track_msg));



        }

        


    

    