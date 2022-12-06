#include <stdio.h>
#include <string.h>
#include <sl/Camera.hpp>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include "shelf-pack.hpp"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <algorithm>

#include <image_transport/image_transport.h>
#include <bits/stdc++.h>

#include <csignal>
#include <thread>

#include <chrono>
#include <algorithm>
#include <set>
#include <deque>
#include <queue>

using namespace std;
using namespace mapbox;
using namespace cv;

typedef std::pair<cv::Rect, cv::Rect> roi_pair;
typedef std::pair<cv::Point2f, cv::Point2f> point_pair;
typedef std::vector<cv::Rect> roi_list;

#define DEBUG_GOOD  0 // Green
#define DEBUG_MAYBE 1 // Blue
#define DEBUG_BAD   2 // Red
#define DEBUG_NONE  3 // Purple

// ROS Params
std::string camera_id_;
cv::Scalar low_HSV_, high_HSV_;
double background_resize_;
double z_k2, z_k1, z_k0, xy_k2, xy_k1, xy_k0;
bool print_diagnostics_;
double guess_max_area_diff_, guess_max_square_diff_, guess_max_dist_diff_;
double lockin_max_dist_, lockin_wait_;
bool debug_minimal_;
double debug_resize_;
bool largest_moving_center_;
bool is_merged_;
double merged_percent_;
double debug_publish_period_;

// Buffer
int max_history = 20;
std::vector<sl::Mat> sl_img_buffer(max_history);
std::deque<Mat> img_buffer;

// Global
bool lockedIn;
roi_pair prev_ball_ROIs;
int candidate_debug = DEBUG_NONE; // For coloring the debug image on similar guess-candidates
sensor_msgs::CameraInfoPtr info_msg_;

// Constant
double fx, fy, cx, cy, base_line;
cv::Ptr<cv::BackgroundSubtractorMOG2> pBackSub_, pBackSub_left_, pBackSub_right_;
std::vector<double> covariance;

// Publisher / Subsciber
ros::Publisher pose_pub_;
image_transport::CameraPublisher debug_pub_;

void fillCamInfo(sl::Camera& zed) {
    // Get Resol
    int cam_width = zed.getCameraInformation().camera_configuration.resolution.width;
    int cam_height = zed.getCameraInformation().camera_configuration.resolution.height;
    int v_w = static_cast<int>(cam_width * debug_resize_);
    int v_h = static_cast<int>(cam_height * debug_resize_);
    sl::Resolution mat_resol = sl::Resolution(v_w, v_h);

    // Get Params
    sl::CalibrationParameters zedParam;
    zedParam = zed.getCameraInformation(mat_resol).calibration_parameters;
    float baseline = zedParam.getCameraBaseline();
    info_msg_->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info_msg_->D.resize(5);
    info_msg_->D[0] = zedParam.left_cam.disto[0];  // k1
    info_msg_->D[1] = zedParam.left_cam.disto[1];  // k2
    info_msg_->D[2] = zedParam.left_cam.disto[4];  // k3
    info_msg_->D[3] = zedParam.left_cam.disto[2];  // p1
    info_msg_->D[4] = zedParam.left_cam.disto[3];  // p2
    info_msg_->K.fill(0.0);
    info_msg_->K[0] = static_cast<double>(zedParam.left_cam.fx);
    info_msg_->K[2] = static_cast<double>(zedParam.left_cam.cx);
    info_msg_->K[4] = static_cast<double>(zedParam.left_cam.fy);
    info_msg_->K[5] = static_cast<double>(zedParam.left_cam.cy);
    info_msg_->K[8] = 1.0;
    info_msg_->R.fill(0.0);

    for (size_t i = 0; i < 3; i++) {
        info_msg_->R[i + i * 3] = 1;     // identity
    }

    info_msg_->P.fill(0.0);
    info_msg_->P[0] = static_cast<double>(zedParam.left_cam.fx);
    info_msg_->P[2] = static_cast<double>(zedParam.left_cam.cx);
    info_msg_->P[5] = static_cast<double>(zedParam.left_cam.fy);
    info_msg_->P[6] = static_cast<double>(zedParam.left_cam.cy);
    info_msg_->P[10] = 1.0;

    info_msg_->width = static_cast<uint32_t>(mat_resol.width);
    info_msg_->height = static_cast<uint32_t>(mat_resol.height);
    info_msg_->header.frame_id = "camera_" + camera_id_ + "_left_camera_optical_frame";
}

Mat slMat2cvMat(const sl::Mat& input) {
    return Mat(input.getHeight(), input.getWidth(), CV_8UC4, input.getPtr<sl::uchar1>(sl::MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

ros::Time slTime2Ros(sl::Timestamp t) {
  uint32_t sec = static_cast<uint32_t>(t.getNanoseconds() / 1000000000);
  uint32_t nsec = static_cast<uint32_t>(t.getNanoseconds() % 1000000000);
  return ros::Time(sec, nsec);
}

double getDepth(double left, double right) {
    // Z = focal length * baseline / disparity
    double depth = fx * base_line / (left - right);
    return depth;
}

geometry_msgs::Point getCartesianCoord(double x, double y, double depth) {
  // Got expression from:
  // https://answers.ros.org/question/190076/how-to-retrieve-xyz-co-ordinates-from-a-raw-depth-image/
  geometry_msgs::Point position;
  position.x = depth / fx * (x - cx);
  position.y = depth / fy * (y - cy);
  position.z = depth;
  return position;
}

geometry_msgs::PoseWithCovarianceStamped createEstimateMsg(const geometry_msgs::Point& position, const ros::Time& time_taken) {
  // Create Pose Estimate
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = time_taken;
  pose_msg.header.frame_id = "camera_" + camera_id_ + "_center_camera_optical";

  pose_msg.pose.pose.position = position;
  pose_msg.pose.pose.orientation.w = 1;

  // Check
  double r = sqrt(pow(position.x, 2) + pow(position.y, 2) + pow(position.z, 2));
  if (r == 0 || position.z <= 0) {
    ROS_ERROR_STREAM_THROTTLE(0.5, "Ball range or position.x == 0... Something is wrong!");
  }

  // Covariance x, y, z diag (dim: 6x6) (0, 6+1, 12+2)
  double d = position.z;
  covariance[0] = xy_k2*pow(d, 2) + xy_k1*d + xy_k0;; //0.01
  covariance[7] = xy_k2*pow(d, 2) + xy_k1*d + xy_k0;  //0.01
  covariance[14] = z_k2*pow(d, 2) + z_k1*d + z_k0;    //0.05
  std::copy(covariance.begin(), covariance.end(), pose_msg.pose.covariance.begin());

  return pose_msg;
}

void signalHandler( int signum ) {
    cout << "Interrupt signal (" << signum << ") received.\n";
    cout << "Freeing Image Memory. \n";
    for (auto& image : sl_img_buffer) { image.free(); }
    exit(signum);
}

void image_buffer_bookkeeping(const Mat& frame) {
    // Have the latest <max_history> number of image saved in rolling queue
    img_buffer.push_back(frame);
    if (img_buffer.size() <= max_history) { return; }
    else { img_buffer.pop_front(); }
}


roi_list findMovingCandidates(const Mat& frame) {
    auto start = ros::Time::now();
    static Mat moving_small;

    // Resize image, perform background subtraction, convert to CPU
    cv::resize(frame, moving_small, cv::Size(), background_resize_, background_resize_, cv::INTER_AREA);
    pBackSub_->apply(moving_small, moving_small, -1);


    // Filter and resize ROIs
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(moving_small, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    std::deque<cv::Rect> ROIs;
    cv::Rect frame_roi(0, 0, frame.cols, frame.rows);
    for (const auto& contour : contours) {
        // Remove too small or too big
        auto area = cv::contourArea(contour);
        if (area <= 1 || moving_small.size().area() / 4.0 < area) {
            continue;
        }

        // Find ROI, scale it to orginal,  make sure it fits in frame
        cv::Rect roi_small = cv::boundingRect(contour);

        int pad = 2;  // Make sure to capture surrounding area
        cv::Rect roi;
        roi.x = (roi_small.x-pad) / background_resize_;
        roi.y = (roi_small.y-pad) / background_resize_;
        roi.width = (roi_small.width+2*pad) / background_resize_;
        roi.height = (roi_small.height+2*pad) / background_resize_;
        roi &= frame_roi;

        ROIs.push_back(roi);
    }

    // Merge all ROI that overlap
    bool overlap = ROIs.size() > 1;
    while(overlap) {
        // Get front and pop
        cv::Rect curr_roi = ROIs.front();
        ROIs.pop_front();

        // Check if curr roi is within another roi (if so then remove)
        // Else try to merge with any overlapping roi
        bool within = false;
        for (const auto& roi : ROIs) {
            if ((curr_roi | roi) == roi) {
                within = true;
            }else if ((curr_roi & roi).area() > 0) {
                curr_roi = (curr_roi | roi) & frame_roi;
            }
        }
        if (!within) {
            ROIs.push_back(curr_roi);
        }

        // Check if any overlap
        overlap = false;
        for (int i = 0; i < ROIs.size() && !overlap; i++) {
            for (int j = i+1; j < ROIs.size() && !overlap; j++) {
                if ((ROIs[i] & ROIs[j]).area() > 0) {
                    overlap = true;
                }
            }
        }
    }
    roi_list unique_ROIs;
    for (const auto& roi : ROIs) {
        unique_ROIs.push_back(roi);
    }

    is_merged_ = false;

    if (print_diagnostics_)
        ROS_INFO_STREAM_THROTTLE(0.5,"Find Candidates: " << (ros::Time::now() - start).toSec() << " sec");

    return unique_ROIs;
}

void searchClosestCandidates(const Mat& frame, roi_list& ROIs, roi_pair& ball_ROIs) {
    // If previous ball is empty then leave
    if (prev_ball_ROIs.first.empty() || prev_ball_ROIs.second.empty()) { return; }

     // Just double-checking
    auto start = ros::Time::now();

    auto left_prev_center = (prev_ball_ROIs.first.br() + prev_ball_ROIs.first.tl()) * 0.5;
    auto right_prev_center = (prev_ball_ROIs.second.br() + prev_ball_ROIs.second.tl()) * 0.5;
    cv::Rect left_closest, right_closest;
    int left_min = INT_MAX, right_min = INT_MAX;

    for (const auto& roi: ROIs) {
        auto roi_center = (roi.br() + roi.tl()) * 0.5;
        double left_dist = cv::norm(left_prev_center - roi_center);
        double right_dist = cv::norm(right_prev_center - roi_center);

        if (roi.br().x < frame.cols / 2 && left_dist < left_min) {
            left_min = left_dist;
            left_closest = roi;
        } else if (roi.tl().x > frame.cols / 2 && right_dist < right_min) {
            right_min = right_dist;
            right_closest = roi;
        }
    }
    if (left_closest.empty() || right_closest.empty()) { return; }

    // Idk why this is faster than doing individually
    // Copying is much faster than performing other CV functions so just add to one image
    cv::Rect left_mask(0,0,left_closest.width, left_closest.height);
    cv::Rect right_mask(left_closest.width,0,right_closest.width, right_closest.height);

    Mat combined(max(left_closest.height, right_closest.height), left_closest.width + right_closest.width, frame.type());
    frame(left_closest).copyTo(combined(left_mask));               // Cheap
    frame(right_closest).copyTo(combined(right_mask));             // Cheap

    cv::cvtColor(combined, combined, cv::COLOR_BGR2HSV, 0);  // Very Expensive
    cv::inRange(combined, low_HSV_, high_HSV_, combined);    // Very Expensive
    int left_detect = cv::countNonZero(combined(left_mask));
    int right_detect = cv::countNonZero(combined(right_mask));


    // Evulate the guesses for left and right
    auto evaluate_guess = [&](cv::Rect& prev_roi, cv::Rect& closest_roi, cv::Rect& ball_roi, int detection) {
        if (detection > 0) {             // If have color then go for it
            ball_roi = closest_roi;
            candidate_debug = DEBUG_GOOD;
            is_merged_ |= (double) detection / closest_roi.area() < merged_percent_;
        } else {                                    // Else see if "similar"
            double area_diff = abs(prev_roi.area()-closest_roi.area()) / (double) closest_roi.area();
            double square_diff = abs(closest_roi.width-closest_roi.height) / (double) closest_roi.height;
            double dist_diff = cv::norm(prev_roi.tl() - closest_roi.tl());

            if (area_diff < guess_max_area_diff_
                    && square_diff < guess_max_square_diff_
                    && dist_diff < guess_max_dist_diff_) {
                ball_roi = closest_roi;
                candidate_debug = DEBUG_MAYBE;
            } else {
                candidate_debug = DEBUG_BAD;
            }
        }
    };
    evaluate_guess(prev_ball_ROIs.second, right_closest, ball_ROIs.second, right_detect);
    evaluate_guess(prev_ball_ROIs.first, left_closest, ball_ROIs.first, left_detect);   // Left has to be second for debug

    if (print_diagnostics_)
        ROS_INFO_STREAM_THROTTLE(0.5,"Evaluate Closest Candidate: " << (ros::Time::now() - start).toSec() << " sec. "
        << "Success: " << (!ball_ROIs.first.empty() && !ball_ROIs.second.empty()) );
}

void searchCandidates(const Mat& frame, roi_list& ROIs, roi_pair& ball_ROIs) {
    // I just waited before this so I am going to block....
    auto start = ros::Time::now();

    // Create 2D composite grid
    std::vector<Bin> bins;
    for (const auto& roi: ROIs) {
        bins.emplace_back(bins.size(), roi.width, roi.height);
    }
    ShelfPack::ShelfPackOptions options;
    options.autoResize = true;
    ShelfPack sprite(10, 10, options);
    std::vector<Bin*> results = sprite.pack(bins);

    // Copy data to shelf
    Mat shelf((int)sprite.height(), (int)sprite.width(), frame.type());
    for (const auto& bin : results) {
        cv::Rect shelf_roi(cv::Point2f(bin->x, bin->y), ROIs[bin->id].size());
        frame(ROIs[bin->id]).copyTo(shelf(shelf_roi));
    }

    // HSV + Threshold + Detect
    cv::cvtColor(shelf, shelf, cv::COLOR_BGR2HSV, 0);
    cv::inRange(shelf, low_HSV_, high_HSV_, shelf);

    // Find non-zeros
    std::vector<cv::Point2f> detections;
    cv::findNonZero(shelf, detections);

    // Find largest colored left and right
    int left_max = INT_MIN, right_max = INT_MIN;
    int left_cnt = INT_MIN, right_cnt = INT_MIN;
    for (const auto& bin : results) {
        cv::Rect roi = ROIs[bin->id];
        cv::Rect bin_roi(cv::Point2f(bin->x, bin->y), roi.size());

        int count = 0;
        for (const auto& detection : detections) {
            count += bin_roi.contains(detection) ? 1 : 0;
        }
        bool colored = count > 1;

        if (colored) {
            int area = roi.area();
            if (roi.br().x < frame.cols / 2 && area > left_max) {        // Left
                left_max = area;
                left_cnt = count;
                ball_ROIs.first = roi;
            } else if (roi.tl().x > frame.cols / 2 && area > right_max){ // Right
                right_max = area;
                right_cnt = count;
                ball_ROIs.second = roi;
            }
        }
    }

    if(!ball_ROIs.first.empty() && !ball_ROIs.second.empty()) {
        is_merged_ = min((double)left_cnt / left_max, (double)right_cnt / right_max) < merged_percent_;
    }

    if (print_diagnostics_)
        ROS_INFO_STREAM_THROTTLE(0.5,"Evaluate All Candidates: " << (ros::Time::now() - start).toSec() << " sec");
}

void findCandidateCenter(const Mat& frame, roi_pair& ball_ROIs, point_pair& center) {
     // Just double checking
    auto start = ros::Time::now();

    // Find the largest moving contour using history buffer
    if (largest_moving_center_ && !is_merged_) {
        // High-Res small-ROI moving area (non-blocking)
        static Mat left_moving, right_moving;
        static cv::Mat left_moving_cpu, right_moving_cpu;
        for (const auto& old_frame : img_buffer) {
            pBackSub_left_->apply(old_frame(ball_ROIs.first), left_moving, -1);
            pBackSub_right_->apply(old_frame(ball_ROIs.second), right_moving, -1);
        }

        // Get center for image
        auto calc_center = [&](const cv::Mat& img, cv::Point2f parent_tl, cv::Point2f& center, cv::Rect& ball_roi) {
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
            std::vector<cv::Point> max_contour;
            double max_area = INT_MIN;
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (max_area < area) {
                    max_area = area;
                    max_contour = contour;
                }
            }
            if (!max_contour.empty()) {
                cv::Moments m = cv::moments(max_contour, true);
                center = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00) + parent_tl;
            }
        };
        calc_center(left_moving, ball_ROIs.first.tl(), center.first, ball_ROIs.first);
        calc_center(right_moving, ball_ROIs.second.tl(), center.second, ball_ROIs.second);

    } else {
        auto&[left_roi, right_roi] = ball_ROIs;
        cv::Rect left_mask(0,0,left_roi.width, left_roi.height);
        cv::Rect right_mask(left_roi.width,0,right_roi.width, right_roi.height);

        // Could use the one from evalution but oh well, maybe in the future
        Mat combined(max(left_roi.height, right_roi.height), left_roi.width + right_roi.width, frame.type());
        frame(left_roi).copyTo(combined(left_mask));               // Cheap
        frame(right_roi).copyTo(combined(right_mask));             // Cheap
        cv::cvtColor(combined, combined, cv::COLOR_BGR2HSV, 0);  // Very Expensive
        cv::inRange(combined, low_HSV_, high_HSV_, combined);    // Very Expensive

        auto calc_center = [&](const cv::Mat& img, cv::Point2f parent_tl, cv::Point2f& center, cv::Rect& ball_roi) {
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
            std::vector<cv::Point> max_contour;
            double max_area = INT_MIN;
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (max_area < area) {
                    max_area = area;
                    max_contour = contour;
                }
            }
            if (!max_contour.empty()) {
                cv::Moments m = cv::moments(max_contour, true);
                center = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00) + parent_tl;
            }
        };
        calc_center(combined(left_mask), left_roi.tl(), center.first, left_roi);
        calc_center(combined(right_mask), right_roi.tl(), center.second, right_roi);
    }

    if (print_diagnostics_)
        ROS_INFO_STREAM_THROTTLE(0.5,"Find Center: " << (ros::Time::now() - start).toSec() << " sec");
}

void publishCenter(point_pair& centers, ros::Time& time_taken) {
    // Publish Pose
    auto left_center = centers.first;
    auto right_center = centers.second;
    right_center.x -= img_buffer.front().cols / 2; // Since SideBySide, need to remove cols from right
    double depth = getDepth(left_center.x, right_center.x);
    if (isnan(depth) || depth <= 0) {  return; }
    auto position = getCartesianCoord(left_center.x, left_center.y, depth);

    // Check for jumps
    static bool lockedIn = false;
    static ros::Time last_position_time;
    static geometry_msgs::Point prev_position;

    // Only publish if (not currently locked into position), (previous lockin reset), or (locked in and within distance)
    double dist = sqrt(pow(position.x-prev_position.x,2)+pow(position.y-prev_position.y,2)+pow(position.z-prev_position.z,2));
    if (!lockedIn
            || (ros::Time::now() - last_position_time) > ros::Duration(lockin_wait_)
            || (lockedIn && dist < lockin_max_dist_)) {
        // I kinda don't trust the first one, might be a fluke
        if (lockedIn) {
            auto pose_msg = createEstimateMsg(position, time_taken);
            pose_pub_.publish(pose_msg);
        }
        last_position_time = ros::Time::now();
        prev_position = position;
        lockedIn = true;
    }
}

void publishDebug(const Mat& frame, roi_list& ROIs, roi_pair& ball_ROIs, point_pair& centers, ros::Time time) {
    // Only publish at fixed rate
    static ros::Time last_publish_time = ros::Time::now();
    if (ros::Time::now() - last_publish_time < ros::Duration(debug_publish_period_)) {
        return;
    }
    last_publish_time = ros::Time::now();

    cv::Rect left_roi(0, 0, frame.cols/2, frame.rows);
    static Mat debug_img(frame(left_roi).size(), frame(left_roi).type());

    if (!debug_minimal_) {
        // Copy current image
        frame(left_roi).copyTo(debug_img);

        // Draw all moving candidates
        for (const auto& roi : ROIs) {
            if (left_roi.contains(roi.br())) {
                cv::add(debug_img(roi & left_roi),  cv::Scalar(0, 50, 75), debug_img(roi & left_roi), cv::noArray(), -1);
            }
        }

        // Draw candidate
        cv::Rect ball_roi = ball_ROIs.first;
        if (!ball_roi.empty()) {
            cv::Scalar color = candidate_debug == DEBUG_GOOD  ? cv::Scalar( 0, 75, 0) :
                              (candidate_debug == DEBUG_MAYBE ? cv::Scalar(75, 0,  0) :
                              (candidate_debug == DEBUG_BAD   ? cv::Scalar( 0, 0, 75) :
                                                                cv::Scalar(75, 0, 75)));
            candidate_debug = DEBUG_NONE;
            cv::add(frame(ball_roi & left_roi),  color, debug_img(ball_roi & left_roi), cv::noArray(), -1);
        }

        // Draw green dot
        cv::Point2f center = centers.first;
        if (center != cv::Point2f() && left_roi.contains(center)) {
            int size = 7;
            cv::Rect center_roi(center - cv::Point2f((size-1)/2, (size-1)/2), cv::Size(size,size));
            cv::Scalar color = is_merged_ ? cv::Scalar(255, 50, 255) : cv::Scalar(255, 50, 50);
            debug_img(center_roi & left_roi).setTo(color);
        }

        cv::resize(debug_img, debug_img, cv::Size(), debug_resize_, debug_resize_, cv::INTER_AREA);

    } else {
        // Resize debug image
        cv::resize(frame(left_roi), debug_img, cv::Size(), debug_resize_, debug_resize_, cv::INTER_AREA);
        cv::Rect resized_left_roi(0, 0, debug_img.cols, debug_img.rows);

        // Draw green dot
        cv::Point2f center = centers.first;
        if (center != cv::Point2f() && left_roi.contains(center)) {
            cv::Rect roi(cv::Point2f(center.x * debug_resize_, center.y * debug_resize_),  cv::Size(10, 10));
            debug_img(roi & resized_left_roi).setTo(cv::Scalar(255, 50, 50));
        }
    }

    info_msg_->header.stamp = time;
    debug_pub_.publish(cv_bridge::CvImage(info_msg_->header, "bgra8", debug_img).toImageMsg(), info_msg_);
}

// search closest, seperation, lock in
void detectBall(int image_idx) {
    // sl::Mat -> cv::Mat (No Copy!)
    Mat frame = slMat2cvMat(sl_img_buffer[image_idx]);
    ros::Time time_taken = slTime2Ros(sl_img_buffer[image_idx].timestamp);

    // Image queue book-keeping
    image_buffer_bookkeeping(frame);

    // Get moving ROI candidates in shrunk image
    roi_pair ball_ROIs;
    roi_list ROIs = findMovingCandidates(frame); // Blocking

    // If have previous ball, find closest candidate and check if colored or very similar to previous (fast heuristic)
    searchClosestCandidates(frame, ROIs, ball_ROIs); // Blocking

    // If no ball candidate selected, pack candidates into new minimum area image and find largest colored candidate
    if (ball_ROIs.first.empty() || ball_ROIs.second.empty()) {
        searchCandidates(frame, ROIs, ball_ROIs);  // Blocking
    }
    prev_ball_ROIs = ball_ROIs;

    // If found candidates, perform background subtraction at high-resolution on candidate
    // If colored and moving parts strongly overlap, find center of moving part, else, colored part
    point_pair centers;
    if (!ball_ROIs.first.empty() && !ball_ROIs.second.empty()) {
        // Get High-Res moving ball
        findCandidateCenter(frame, ball_ROIs, centers); // Blocking
        if (centers.first != cv::Point2f() && centers.second != cv::Point2f()) {
            publishCenter(centers, time_taken); // Blocking
        }
    }

    // Create debug image
    if (debug_pub_.getNumSubscribers() != 0) {
        publishDebug(frame, ROIs, ball_ROIs, centers, time_taken); // (Non-Blocking)
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ball_detection_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Initial ZED Params
    sl::InitParameters init_parameters;
    init_parameters.sdk_verbose = true;
    // init_parameters.input.setFromSVOFile("~/demo_day_1_drg.svo");
    init_parameters.camera_resolution= sl::RESOLUTION::HD1080;
    init_parameters.coordinate_units = sl::UNIT::METER;
    init_parameters.depth_mode = sl::DEPTH_MODE::NONE;
    init_parameters.enable_image_enhancement = false;
    init_parameters.depth_stabilization = false;

    // Open the ZED camera
    sl::Camera zed;
    auto returned_state = zed.open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        ROS_WARN_STREAM("Camera Open " << returned_state << " Exit program.");
        return EXIT_FAILURE;
    }

    // Be sure that depth is not computed
    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.enable_depth = false;

    // Dynamically adjust exposure / gain
    int gain, exposure;
    nhp.param("gain", gain, 15);
    nhp.param("exposure", exposure, 75);
    zed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, exposure);
    zed.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, gain);

    // Parameters
    nhp.getParam("camera_id", camera_id_);
    nhp.param("low_H", low_HSV_[0], 10.0);
    nhp.param("low_S", low_HSV_[1], 170.0);
    nhp.param("low_V", low_HSV_[2], 170.0);
    nhp.param("high_H", high_HSV_[0], 35.0);
    nhp.param("high_S", high_HSV_[1], 255.0);
    nhp.param("high_V", high_HSV_[2], 255.0);
    nhp.param("print_diagnostics", print_diagnostics_, false);
    nhp.param("z_k2", z_k2, 0.002);
    nhp.param("z_k1", z_k1, 0.001);
    nhp.param("z_k0", z_k0, 0.01);
    nhp.param("xy_k2", xy_k2, 0.0003);
    nhp.param("xy_k1", xy_k1, 0.0004);
    nhp.param("xy_k0", xy_k0, 0.01);
    nhp.param("guess_max_area_diff", guess_max_area_diff_, 1.5);
    nhp.param("guess_max_square_diff", guess_max_square_diff_, 2.0);
    nhp.param("guess_max_dist_diff", guess_max_dist_diff_, -1.0);
    nhp.param("lockin_wait", lockin_wait_, 0.1);
    nhp.param("lockin_max_dist", lockin_max_dist_, 2.0);
    nhp.param("background_resize", background_resize_, 0.4);
    nhp.param("debug_minimal", debug_minimal_, false);
    nhp.param("debug_resize", debug_resize_, 0.4);
    nhp.param("largest_moving_center", largest_moving_center_,true);
    nhp.param("merged_percent", merged_percent_, 0.05);
    nhp.param("debug_publish_period", debug_publish_period_, 0.01);

    // Get camera information
    auto camera_info = zed.getCameraInformation();
    cout << endl;
    cout <<"ZED Model                 : "<< camera_info.camera_model << endl;
    cout <<"ZED Camera Resolution     : "<< camera_info.camera_configuration.resolution.width<<"x"<<camera_info.camera_configuration.resolution.height << endl;
    cout <<"ZED Camera FPS            : "<< zed.getInitParameters().camera_fps << endl;

    fx = camera_info.calibration_parameters.left_cam.fx;
    cx = camera_info.calibration_parameters.left_cam.cx;
    fy = camera_info.calibration_parameters.left_cam.fy;
    cy = camera_info.calibration_parameters.left_cam.cy;
    base_line = camera_info.calibration_parameters.stereo_transform.getTranslation()[0];
    
    info_msg_.reset(new sensor_msgs::CameraInfo());
    fillCamInfo(zed);

    // Subscribers / Publishers
    pose_pub_ = nhp.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ball_" + camera_id_ + "_pose", 1);
    image_transport::ImageTransport it(nhp);
    debug_pub_ = it.advertiseCamera("debug_img", 1);
    ROS_INFO_STREAM("Advertised on topic " << debug_pub_.getTopic());
    ROS_INFO_STREAM("Advertised on topic " << debug_pub_.getInfoTopic());

    pBackSub_ = createBackgroundSubtractorMOG2(100, 50, false);
    pBackSub_left_ = createBackgroundSubtractorMOG2(100, 20, false);
    pBackSub_right_ = createBackgroundSubtractorMOG2(100, 20, false);

    covariance = std::vector<double>(36, 0.0);

    ros::Time prev_loop_time = ros::Time::now();

    std::signal(SIGINT, signalHandler); // Free memory so don't get error
    std::thread thd;
    int i = 0;

    while (ros::ok()) {
        // Grab a image
        auto grab_start = ros::Time::now();
        returned_state = zed.grab(runtime_parameters);
        auto grab_end = ros::Time::now();

        if (returned_state == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
            ROS_WARN_STREAM("SVO end has been reached. Looping back!");
            zed.setSVOPosition(0);
        }else if (returned_state != sl::ERROR_CODE::SUCCESS) {
            ROS_ERROR_STREAM("Error during capture: " << returned_state);
            break;
        }

        // Convert retrieve imag
        auto retrieve_start = ros::Time::now();
        zed.retrieveImage(sl_img_buffer[i], sl::VIEW::SIDE_BY_SIDE, sl::MEM::CPU); // Need to keep them in memory
        auto retrieve_end = ros::Time::now();

        // Detect Ball
        auto detect_start = ros::Time::now();
        // detectBall(i);
        // Move detectBall to another thread so can wait to grab in mean time
        if (thd.joinable()) { thd.join(); }
        thd = std::move(std::thread(detectBall, i));
        auto detect_end = ros::Time::now();

        i = (i + 1) % max_history;

        if (print_diagnostics_) {
            ros::Time curr_time = ros::Time::now();
            ROS_INFO_STREAM_THROTTLE(1, "--- Diagnostics CPU: " << camera_id_ << " ---");
            ROS_INFO_STREAM_THROTTLE(1, "Grab Elapsed: " << (grab_end - grab_start).toSec() << " sec");
            ROS_INFO_STREAM_THROTTLE(1, "Preprocessing Elapsed: " << (retrieve_end - retrieve_start).toSec() << " sec");
            ROS_INFO_STREAM_THROTTLE(1, "Processing Elapsed: " << (detect_end - detect_start).toSec() << " sec");
            ROS_INFO_STREAM_THROTTLE(1, "Loop Speed: " << (curr_time - prev_loop_time).toSec() << " sec");
            prev_loop_time = curr_time;
        }
    }

    // Exit
    if (thd.joinable()) { thd.join(); }
    for (auto& image : sl_img_buffer) { image.free(); }
    zed.close();
    return EXIT_SUCCESS;
}
