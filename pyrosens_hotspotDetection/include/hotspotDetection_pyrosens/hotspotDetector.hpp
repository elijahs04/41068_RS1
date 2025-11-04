#pragma once

#include <opencv2/core.hpp>

namespace thermdetect {

// A detected hotspot region (arbitrary shape grouped as a blob)
struct HotspotRegion {
  cv::Rect  bbox_px;           // x,y,w,h
  cv::Point2f centroid_px;     // region centroid
  int        area_px = 0;      // number of pixels in region
  double     max_temp_c = 0.0; // max temperature in region
  double     mean_temp_c = 0.0;// mean temperature in region
  double     max_raw = 0.0;    // (optional) raw max for debugging
};

// Configuration for detector
struct DetectorConfig {
  double temp_gain = 1.0;   // temp = gain * raw + offset
  double temp_offset = 0.0;
  int blur_kernel = 0;      // 0 means disabled
  double min_temp_c = 0.0;
  int min_area_px = 0;
  int max_area_px = 0;
  double hot_temp_c = 200.0;  // classify pixels >= this as hot
  bool use_percentile = false; // optional adaptive mode
  double hot_percentile = 98.0;  // if adaptive, keep top X% hottest
  int morphology_kernel = 0;  // 0 disables, otherwise odd size (e.g., 3,5)
};

class HotspotDetector {
public:
  HotspotDetector() = default;
  explicit HotspotDetector(const DetectorConfig& cfg);

  // Update configuration at runtime
  void setConfig(const DetectorConfig& cfg);
  const DetectorConfig& getConfig() const;

  // Returns regions sorted by descending area
  std::vector<HotspotRegion> detectHotspots(const cv::Mat& thermal_img) const;

private:
  DetectorConfig cfg_;
};

double percentileRawMono8(const cv::Mat& img, double percentile);

} // namespace thermdetect
