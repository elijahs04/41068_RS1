#include "hotspotDetection_pyrosens/hotspotDetector.hpp"
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace thermdetect {
  HotspotDetector::HotspotDetector(const DetectorConfig& cfg) 
: cfg_(cfg) {}

void HotspotDetector::setConfig(const DetectorConfig& cfg) {
  cfg_ = cfg;
}

const DetectorConfig& HotspotDetector::getConfig() const {
  return cfg_;
}

HotspotResult HotspotDetector::detectHottestPixel(const cv::Mat& thermal_img) const
{
  HotspotResult result;

  if (thermal_img.empty() || thermal_img.cols <= 0 || thermal_img.rows <= 0) {
    return result; // u,v stay -1
  }

  // Ensure single-channel 8-bit or 16-bit
  if (thermal_img.type() != CV_8UC1 && thermal_img.type() != CV_16UC1) {
    return result;
  }

  // Find the location of the maximum pixel value
  double minVal, maxVal;
  cv::Point minLoc, maxLoc;
  cv::minMaxLoc(thermal_img, &minVal, &maxVal, &minLoc, &maxLoc);

  // Fill in result
  result.u = maxLoc.x;
  result.v = maxLoc.y;

  // Raw value as double
  result.raw = maxVal;

  // Convert to Celsius using linear scale
  result.temp_c = cfg_.temp_gain * result.raw + cfg_.temp_offset;


  return result;
}
} // namespace thermdetect