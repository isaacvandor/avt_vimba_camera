/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <avt_vimba_camera/mono_camera.h>

#define DEBUG_PRINTS 1

static const std::string OPENCV_WINDOW = "Image window";
namespace avt_vimba_camera {

MonoCamera::MonoCamera(ros::NodeHandle& nh, ros::NodeHandle& nhp) : nh_(nh), nhp_(nhp), it_(nhp), cam_(ros::this_node::getName()) {
  // Prepare node handle for the camera
  // TODO use nodelets with getMTNodeHandle()

  // Start Vimba & list all available cameras
  api_.start();

  // Set the image publisher before the streaming
  pub_  = it_.advertiseCamera("image_raw",  1);

  sub_ = it_.subscribe("image_raw", 1, &MonoCamera::imageCb, this);

  image_pub_ = it_.advertise("/image_converter/output_video", 1);

  cv::namedWindow(OPENCV_WINDOW);

  // Set the frame callback
  cam_.setCallback(boost::bind(&avt_vimba_camera::MonoCamera::frameCallback, this, _1));

  // Set the params
  nhp_.param("ip", ip_, std::string(""));
  nhp_.param("guid", guid_, std::string(""));
  nhp_.param("camera_info_url", camera_info_url_, std::string(""));
  std::string frame_id;
  std::string log_directory;
  nhp_.param("log_directory", log_directory, std::string(""));
  nhp_.param("frame_id", frame_id, std::string(""));
  nhp_.param("show_debug_prints", show_debug_prints_, false);

  // Set camera info manager
  info_man_  = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nhp_, frame_id, camera_info_url_));

  // Start dynamic_reconfigure & run configure()
  reconfigure_server_.setCallback(boost::bind(&avt_vimba_camera::MonoCamera::configure, this, _1, _2));
}

MonoCamera::~MonoCamera(void) {
  cam_.stop();
  pub_.shutdown();
  cv::destroyWindow(OPENCV_WINDOW);
}

void MonoCamera::frameCallback(const FramePtr& vimba_frame_ptr) {
  ros::Time ros_time = ros::Time::now();
  if (pub_.getNumSubscribers() > 0) {
    sensor_msgs::Image img;
    if (api_.frameToImage(vimba_frame_ptr, img)) {
      sensor_msgs::CameraInfo ci = info_man_->getCameraInfo();
      ci.header.stamp = img.header.stamp = ros_time;
      img.header.frame_id = ci.header.frame_id;
      pub_.publish(img, ci);
    } else {
      ROS_WARN_STREAM("Function frameToImage returned 0. No image published.");
    }
  }
  // updater_.update();
}

void MonoCamera::imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    /*
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
*/
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    std::string log_directory;
    nhp_.getParam("log_directory", log_directory);
    //const char dir_path[] ="/home/ivandor/sentry_cam_data/logdir/";
    boost::filesystem::path dir_target(log_directory);
    boost::filesystem::path path_folder = dir_target.parent_path();
    if (!boost::filesystem::exists(path_folder)) {
        boost::filesystem::create_directories(path_folder);
        ROS_ERROR_STREAM("log directory created");
    }

    static int img_count = 0;
    time_t rawtime;
    struct tm* timeinfo;
    char time_buf [80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(time_buf,80,"sentry.%Y%m%d.%H%M%S.", timeinfo); //Sentry vehicle, %i for usecs, %f for framecount
    //ROS_ERROR_STREAM("filename is: "<<buffer);
    std::stringstream ss;
    ss<<time_buf<<img_count<<".tiff";
    auto str_target = log_directory + ss.str();
    cv::imwrite(str_target, cv_ptr->image);
    img_count++;
    ROS_INFO_STREAM("Writing image "<<img_count);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}

/** Dynamic reconfigure callback
*
*  Called immediately when callback first defined. Called again
*  when dynamic reconfigure starts or changes a parameter value.
*
*  @param newconfig new Config values
*  @param level bit-wise OR of reconfiguration levels for all
*               changed parameters (0xffffffff on initial call)
**/
void MonoCamera::configure(Config& newconfig, uint32_t level) {
  try {
    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id == "") {
      newconfig.frame_id = "camera";
    }
    // The camera already stops & starts acquisition
    // so there's no problem on changing any feature.
    if (!cam_.isOpened()) {
      cam_.start(ip_, guid_, show_debug_prints_);
    }

    Config config = newconfig;
    cam_.updateConfig(newconfig);
    updateCameraInfo(config);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Error reconfiguring mono_camera node : " << e.what());
  }
}

void MonoCamera::updateCameraInfo(const avt_vimba_camera::AvtVimbaCameraConfig& config) {

  // Get camera_info from the manager
  sensor_msgs::CameraInfo ci = info_man_->getCameraInfo();

  // Set the frame id
  ci.header.frame_id = config.frame_id;

  std::string logdir;
  logdir = config.log_directory;
  std::string log_directory;
  nhp_.getParam("log_directory", log_directory);
  if (log_directory != logdir) {
      ROS_ERROR_STREAM("setting log directory to: "<<logdir);
      nhp_.setParam("log_directory", logdir);
  }

  // Set the operational parameters in CameraInfo (binning, ROI)
  int binning_or_decimation_x = std::max(config.binning_x, config.decimation_x);
  int binning_or_decimation_y = std::max(config.binning_y, config.decimation_y);

  // Set the operational parameters in CameraInfo (binning, ROI)
  ci.height    = config.height;
  ci.width     = config.width;
  ci.binning_x = binning_or_decimation_x;
  ci.binning_y = binning_or_decimation_y;

  // ROI in CameraInfo is in unbinned coordinates, need to scale up
  ci.roi.x_offset = config.roi_offset_x;
  ci.roi.y_offset = config.roi_offset_y;
  ci.roi.height   = config.roi_height;
  ci.roi.width    = config.roi_width;

  // set the new URL and load CameraInfo (if any) from it
  std::string camera_info_url;
  nhp_.getParam("camera_info_url", camera_info_url);
  if (camera_info_url != camera_info_url_) {
    info_man_->setCameraName(config.frame_id);
    if (info_man_->validateURL(camera_info_url)) {
      info_man_->loadCameraInfo(camera_info_url);
      ci = info_man_->getCameraInfo();
    } else {
      ROS_WARN_STREAM("Camera info URL not valid: " << camera_info_url);
    }
  }

  bool roiMatchesCalibration = (ci.height == config.roi_height
                              && ci.width == config.roi_width);
  bool resolutionMatchesCalibration = (ci.width == config.width
                                   && ci.height == config.height);
  // check
  ci.roi.do_rectify = roiMatchesCalibration || resolutionMatchesCalibration;
  //ci.roi.do_rectify = false;
  // push the changes to manager
  info_man_->setCameraInfo(ci);
}

};
