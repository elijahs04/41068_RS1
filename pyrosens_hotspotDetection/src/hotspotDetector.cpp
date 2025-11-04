#include "hotspotDetection_pyrosens/hotspotDetector.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <numeric>

namespace thermdetect {
  HotspotDetector::HotspotDetector(const DetectorConfig& cfg) 
: cfg_(cfg) {}

void HotspotDetector::setConfig(const DetectorConfig& cfg) {
  cfg_ = cfg;
}

const DetectorConfig& HotspotDetector::getConfig() const {
  return cfg_;
}


std::vector<HotspotRegion> HotspotDetector::detectHotspots(const cv::Mat& thermal_img) const
{
  std::vector<HotspotRegion> out;

  // 1) Validate input
  if (thermal_img.empty() || thermal_img.type() != CV_8UC1) {
    return out; // only mono8 here (your sim uses mono8)
  }

  // 2) Decide raw threshold
  double thr_raw = 0.0;
  if (cfg_.use_percentile) {
    thr_raw = thermdetect::percentileRawMono8(thermal_img, cfg_.hot_percentile);
  } else {
    if (cfg_.temp_gain <= 0.0) {
      // Fallback: if gain is invalid, assume raw directly represents temperature
      thr_raw = cfg_.hot_temp_c;
    } else {
      thr_raw = (cfg_.hot_temp_c - cfg_.temp_offset) / cfg_.temp_gain;
    }
    thr_raw = std::clamp(thr_raw, 0.0, 255.0);
  }

  // 3) Threshold: keep pixels >= thr_raw
  cv::Mat hotMask;
  // cv::threshold uses >, so use (thr_raw - 1) to include == thr_raw
  const double t = std::max(0.0, thr_raw - 1.0);
  cv::threshold(thermal_img, hotMask, t, 255.0, cv::THRESH_BINARY);

  // 4) Optional morphology (denoise / fill small gaps)
  if (cfg_.morphology_kernel > 0) {
    int k = cfg_.morphology_kernel;
    if ((k % 2) == 0) k += 1; // ensure odd
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    // Light open then close is a good default
    cv::morphologyEx(hotMask, hotMask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(hotMask, hotMask, cv::MORPH_CLOSE, kernel);
  }

  // 5) Find connected components via contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(hotMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  out.reserve(contours.size());

  for (const auto& c : contours) {
    // Area filter
    const double area = cv::contourArea(c);
    if (area <= 0.0) continue;
    if (area < static_cast<double>(cfg_.min_area_px)) continue;
    if (cfg_.max_area_px > 0 && area > static_cast<double>(cfg_.max_area_px)) continue;

    // Bounding box
    const cv::Rect bbox = cv::boundingRect(c);

    // Centroid from moments
    cv::Moments m = cv::moments(c);
    cv::Point2f centroid(static_cast<float>(bbox.x + bbox.width * 0.5f),
                         static_cast<float>(bbox.y + bbox.height * 0.5f));
    if (m.m00 != 0.0) {
      centroid.x = static_cast<float>(m.m10 / m.m00);
      centroid.y = static_cast<float>(m.m01 / m.m00);
    }

    // Build a per-region mask (within bbox) to compute mean/max efficiently
    cv::Mat roi = thermal_img(bbox);
    cv::Mat regionMask(bbox.height, bbox.width, CV_8U, cv::Scalar(0));
    // Shift contour points into bbox-local coords
    std::vector<std::vector<cv::Point>> cont_local(1);
    cont_local[0].reserve(c.size());
    for (const auto& p : c) cont_local[0].push_back(cv::Point(p.x - bbox.x, p.y - bbox.y));
    cv::drawContours(regionMask, cont_local, 0, cv::Scalar(255), cv::FILLED);

    // Mean and max raw within region
    cv::Scalar mean_raw_s = cv::mean(roi, regionMask);
    double minValDummy = 0.0, maxRaw = 0.0;
    cv::minMaxLoc(roi, &minValDummy, &maxRaw, nullptr, nullptr, regionMask);

    // Convert to °C
    double mean_temp_c = (cfg_.temp_gain > 0.0)
                         ? cfg_.temp_gain * mean_raw_s[0] + cfg_.temp_offset
                         : mean_raw_s[0];
    double max_temp_c  = (cfg_.temp_gain > 0.0)
                         ? cfg_.temp_gain * maxRaw + cfg_.temp_offset
                         : maxRaw;

    HotspotRegion r;
    r.bbox_px      = bbox;
    r.centroid_px  = centroid;
    r.area_px      = static_cast<int>(std::lround(area));
    r.max_temp_c   = max_temp_c;
    r.mean_temp_c  = mean_temp_c;
    r.max_raw      = maxRaw;
    out.push_back(r);
  }

  // 6) Sort regions (largest first). Alternatively, sort by max_temp_c.
  std::sort(out.begin(), out.end(),
            [](const HotspotRegion& a, const HotspotRegion& b){
              return a.area_px > b.area_px;
            });

    return out;
  }

  double percentileRawMono8(const cv::Mat& img_u8, double pct /*0..100*/) {
    // Build histogram (256 bins)
    int histSize = 256;
    float range[] = {0.f, 256.f};
    const float* ranges[] = { range };
    cv::Mat hist;
    cv::calcHist(&img_u8, 1, 0, cv::Mat(), hist, 1, &histSize, ranges, true, false);

    // Cumulative from low to high
    double total = static_cast<double>(img_u8.total());
    double target = std::clamp(pct, 0.0, 100.0) * 0.01 * total;

    double cum = 0.0;
    for (int v = 0; v < 256; ++v) {
      cum += hist.at<float>(v);
      if (cum >= target) {
        return static_cast<double>(v);
      }
    }
    return 255.0; // fallback
  }

} // namespace thermdetect