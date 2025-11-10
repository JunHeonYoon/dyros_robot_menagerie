#pragma once

// --- neutralize common conflicting macros before including OpenCV ---
#if defined(EPS)
  #pragma push_macro("EPS")
  #undef EPS
  #define IMAGE_IO_RESTORE_EPS 1
#endif
#if defined(MAX_ITER)
  #pragma push_macro("MAX_ITER")
  #undef MAX_ITER
  #define IMAGE_IO_RESTORE_MAX_ITER 1
#endif

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

// --- restore macros if they existed ---
#ifdef IMAGE_IO_RESTORE_EPS
  #pragma pop_macro("EPS")
  #undef IMAGE_IO_RESTORE_EPS
#endif
#ifdef IMAGE_IO_RESTORE_MAX_ITER
  #pragma pop_macro("MAX_ITER")
  #undef IMAGE_IO_RESTORE_MAX_ITER
#endif

inline sensor_msgs::msg::Image::SharedPtr
toImageMsg(const cv::Mat& img, const std::string& encoding, const std_msgs::msg::Header& header)
{
  // encoding 예: "bgr8", "rgb8", "mono8", "32FC1"
  auto cv_ptr = cv_bridge::CvImage(header, encoding, img);
  return cv_ptr.toImageMsg(); // 보통 데이터 복사 없이 공유(빌드/버전별 내부 구현 다름)
}

inline sensor_msgs::msg::Image::SharedPtr
toImageMsg(const cv::Mat& img, const std::string& enc)
{
  std_msgs::msg::Header header(rosidl_runtime_cpp::MessageInitialization::ALL);
  auto cv_ptr = cv_bridge::CvImage(header, enc, img);
  return cv_ptr.toImageMsg();
}

inline cv::Mat
fromImageMsg(const sensor_msgs::msg::Image::ConstSharedPtr& msg, std::string* out_encoding = nullptr)
{
  auto cv_ptr = cv_bridge::toCvShare(msg, msg->encoding); // zero-copy 공유
  if (out_encoding) *out_encoding = msg->encoding;
  return cv_ptr->image;  // shallow (참조) 반환
}
