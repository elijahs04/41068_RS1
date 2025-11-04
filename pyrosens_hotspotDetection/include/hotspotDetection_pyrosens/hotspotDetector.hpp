#pragma once

#include <opencv2/core.hpp>

namespace thermdetect {

// Result of a single-frame hotspot query (detected hotspot)
struct HotspotResult {
  int u = -1;          // pixel column
  int v = -1;          // pixel row
  double raw = 0.0;    // raw sensor value at (u,v)
  double temp_c = 0.0; // converted temperature in Celsius
};

// Configuration for detector
struct DetectorConfig {
  double temp_gain = 1.0;   // temp = gain * raw + offset
  double temp_offset = 0.0;

  // Reserved for later expansion (blob mode)
  int blur_kernel = 0;      // 0 means disabled
  double min_temp_c = 0.0;
  int min_area_px = 0;
  int max_area_px = 0;
};

class HotspotDetector {
public:
  HotspotDetector() = default;
  explicit HotspotDetector(const DetectorConfig& cfg);

  // Update configuration at runtime
  void setConfig(const DetectorConfig& cfg);
  const DetectorConfig& getConfig() const;

  // Step 1: return the hottest pixel in the frame
  // Expects a single-channel CV_8UC1 or CV_16UC1 thermal image
  HotspotResult detectHottestPixel(const cv::Mat& thermal_img) const;

  // Placeholder for later: blob-based hotspot detection
  // HotspotResult detectBlobCentroid(const cv::Mat& thermal_img) const;

private:
  DetectorConfig cfg_;
};

} // namespace thd
