// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <openMVG/matching/svg_matches.hpp>
#include <openMVG/features/feature.hpp>
#include "third_party/vectorGraphics/svgDrawer.hpp"

namespace openMVG {
namespace matching {

// Convert HUE color to RGB
inline float hue2rgb(float p, float q, float t)
{
  if (t < 0) t += 1;
  if (t > 1) t -= 1;
  if (t < 1.f/6.f) return p + (q - p) * 6.f * t;
  if (t < 1.f/2.f) return q;
  if (t < 2.f/3.f) return p + (q - p) * (2.f/3.f - t) * 6.f;
  return p;
}

//
// Converts an HSL color value to RGB. Conversion formula
// adapted from http://en.wikipedia.org/wiki/HSL_color_space.
// Assumes h, s, and l are contained in the set [0, 1] and
// returns r, g, and b in the set [0, 255].
void hslToRgb
(
  float h,
  float s,
  float l,
  uint8_t & r,
  uint8_t & g,
  uint8_t & b
)
{
  if (s == 0)
  {
    r = g = b = static_cast<uint8_t>(l * 255.f); // achromatic
  }
  else
  {
    const float q = l < 0.5f ? l * (1 + s) : l + s - l * s;
    const float p = 2.f * l - q;
    r = static_cast<uint8_t>(hue2rgb(p, q, h + 1.f/3.f) * 255.f);
    g = static_cast<uint8_t>(hue2rgb(p, q, h) * 255.f);
    b = static_cast<uint8_t>(hue2rgb(p, q, h - 1.f/3.f) * 255.f);
  }
}

std::string Matches2SVGString
(
  const std::string & left_image_path,
  const std::pair<size_t,size_t> & left_image_size,
  const features::PointFeatures & left_features,
  const std::string & right_image_path,
  const std::pair<size_t,size_t> & right_image_size,
  const features::PointFeatures & right_features,
  const matching::IndMatches & matches,
  const bool b_vertical_display,
  const bool b_relative_lines,
  const double feature_circle_radius,
  const double stroke_size
)
{
  // Scale the "right" image up if needed, assuming video images are more likely to be on the RHS
  float svg_ratio_y = left_image_size.second / float(right_image_size.second);
  float svg_ratio_x = left_image_size.first / float(right_image_size.first);
  float svg_ratio_image = std::max(std::min(svg_ratio_x, svg_ratio_y), 1.0f);

  size_t svg_w, svg_h; // width and height (in pixels) of the image
  size_t svg_offset_x, svg_offset_y; // Offset (in pixels) of the "right" image (left image is at origin)
  if (b_vertical_display) // "right" image is below the "left" image
  {
    svg_w = std::max(left_image_size.first, size_t(right_image_size.first*svg_ratio_image));
    svg_h = left_image_size.second + right_image_size.second * svg_ratio_image;
    svg_offset_x = 0;
    svg_offset_y = left_image_size.second;
  }
  else // Side by side display
  {
    svg_w = left_image_size.first + right_image_size.first * svg_ratio_image;
    svg_h = std::max(left_image_size.second, size_t(right_image_size.second*svg_ratio_image));
    svg_offset_x = left_image_size.first;
    svg_offset_y = 0;
  }
  float svg_ratio_x_lines, svg_ratio_y_lines;
  int svg_offset_x_lines, svg_offset_y_lines;
  if (b_relative_lines) // Relative mode - show the direction of movement
  {
    svg_ratio_x_lines = svg_ratio_image;
    svg_ratio_y_lines = svg_ratio_image;
    svg_offset_x_lines = (right_image_size.first*svg_ratio_image - left_image_size.first) * -0.5f;
    svg_offset_y_lines = (right_image_size.second*svg_ratio_image - left_image_size.second) * -0.5f;
  }
  else // Absolute mode - connect each corresponding point
  {
    svg_ratio_x_lines = svg_ratio_image;
    svg_ratio_y_lines = svg_ratio_image;
    svg_offset_x_lines = svg_offset_x;
    svg_offset_y_lines = svg_offset_y;
  }
  std::cout << "#Ratio=" << svg_ratio_image << " FirstImage=" << left_image_size.first << "*" << left_image_size.second <<
   " SecondImage=" << right_image_size.first << "*" << right_image_size.second << "\n";
  
  svg::svgDrawer svgStream(svg_w, svg_h);

  // Draw image side by side
  svgStream.drawImage(left_image_path, left_image_size.first, left_image_size.second, 0, 0);
  svgStream.drawImage(right_image_path,
    right_image_size.first * svg_ratio_image,
    right_image_size.second * svg_ratio_image,
    svg_offset_x, svg_offset_y);

  std::vector<std::string> colors;
  colors.reserve(matches.size());
  // Draw corresponding matches
  // Perform two loop (it helps to recognize the scene when there is many matches):
  // 1. First draw lines
  for (const auto match_it : matches) {
    // Get back linked features
    const features::PointFeature & L = left_features[match_it.i_];
    const features::PointFeature & R = right_features[match_it.j_];
    { // Compute a flashy colour for the correspondence
      std::ostringstream osCol;
      uint8_t r, g, b;
      hslToRgb( (rand()%360) / 360., 1.0, .5, r, g, b);
      osCol << "rgb(" << (int)r <<',' << (int)g << ',' << (int)b <<")";
      colors.push_back(osCol.str());
    }
    // Draw the line between the corresponding feature positions
    svgStream.drawLine(
      L.x(), L.y(),
      R.x() * svg_ratio_x_lines + svg_offset_x_lines,
      R.y() * svg_ratio_y_lines + svg_offset_y_lines,
      svg::svgStyle().stroke(colors.back(),
      stroke_size));
  }
  // 2. Then display features circles
  for (size_t i = 0; i < matches.size(); ++i) {
    // Get back linked features
    const features::PointFeature & L = left_features[matches[i].i_];
    const features::PointFeature & R = right_features[matches[i].j_];
    // Draw the features (circle)
    svgStream.drawCircle(
      L.x(), L.y(), feature_circle_radius,
      svg::svgStyle().stroke(colors[i], stroke_size));
    svgStream.drawCircle(
      R.x() * svg_ratio_image + svg_offset_x,
      R.y() * svg_ratio_image + svg_offset_y,
      feature_circle_radius,
      svg::svgStyle().stroke(colors[i], stroke_size));
  }
  return svgStream.closeSvgFile().str();
}

bool Matches2SVG
(
  const std::string & left_image_path,
  const std::pair<size_t,size_t> & left_image_size,
  const features::PointFeatures & left_features,
  const std::string & right_image_path,
  const std::pair<size_t,size_t> & right_image_size,
  const features::PointFeatures & right_features,
  const matching::IndMatches & matches,
  const std::string & svg_filename,
  const bool b_vertical_display,
  const bool b_relative_lines,
  const double feature_circle_radius,
  const double stroke_size
)
{
  const std::string svg_content =
    Matches2SVGString
    (
      left_image_path,
      left_image_size,
      left_features,
      right_image_path,
      right_image_size,
      right_features,
      matches,
      b_vertical_display,
      b_relative_lines,
      feature_circle_radius,
      stroke_size
    );
  // Save the SVG file
  std::ofstream svgFile( svg_filename.c_str() );
  if (svgFile.is_open())
  {
    svgFile << svg_content;
    svgFile.close();
    return true;
  }
  return false;
}

bool InlierMatches2SVG
(
  const std::string & left_image_path,
  const std::pair<size_t,size_t> & left_image_size,
  const features::PointFeatures & left_features,
  const std::string & right_image_path,
  const std::pair<size_t,size_t> & right_image_size,
  const features::PointFeatures & right_features,
  const matching::IndMatches & matches,
  const std::vector<uint32_t> & inliers,
  const std::string & svg_filename,
  const bool b_vertical_display,
  const double feature_circle_radius,
  const double stroke_size
)
{
  const size_t svg_w =
    b_vertical_display ?
    left_image_size.first :
    left_image_size.first + right_image_size.first;
  const size_t svg_h =
    b_vertical_display ?
    left_image_size.second + right_image_size.second :
    left_image_size.second;
  const size_t svg_offset_x =
    b_vertical_display ?
    0 :
    left_image_size.first;
  const size_t svg_offset_y =
    b_vertical_display ?
    left_image_size.second :
    0;

  svg::svgDrawer svgStream(svg_w, svg_h);

  // Draw image side by side
  svgStream.drawImage(left_image_path, left_image_size.first, left_image_size.second);
  svgStream.drawImage(right_image_path, right_image_size.first, right_image_size.second,
    b_vertical_display ? 0 : left_image_size.first,
    b_vertical_display ? left_image_size.second : 0);

  std::vector<std::string> colors;
  colors.reserve(inliers.size());
  // Draw corresponding matches
  // Perform two loop (it helps to recognize the scene when there is many matches):
  // 1. First draw lines
  for (const auto inlier_it : inliers) {
    // Get back linked features
    const features::PointFeature & L = left_features[matches[inlier_it].i_];
    const features::PointFeature & R = right_features[matches[inlier_it].j_];
    { // Compute a flashy colour for the correspondence
      std::ostringstream osCol;
      uint8_t r, g, b;
      hslToRgb( (rand()%360) / 360., 1.0, .5, r, g, b);
      osCol << "rgb(" << (int)r <<',' << (int)g << ',' << (int)b <<")";
      colors.push_back(osCol.str());
    }
    // Draw the line between the corresponding feature positions
    svgStream.drawLine(
      L.x(), L.y(),
      R.x() + svg_offset_x, R.y() + svg_offset_y,
      svg::svgStyle().stroke(colors.back(), stroke_size));
  }

  for (size_t i = 0; i < inliers.size(); ++i) {
    // Get back linked features
    const features::PointFeature & L = left_features[matches[inliers[i]].i_];
    const features::PointFeature & R = right_features[matches[inliers[i]].j_];
    // Draw the features (circle)
    svgStream.drawCircle(
      L.x(), L.y(), feature_circle_radius,
      svg::svgStyle().stroke(colors[i], stroke_size));
    svgStream.drawCircle(
      R.x() + svg_offset_x, R.y() + svg_offset_y, feature_circle_radius,
      svg::svgStyle().stroke(colors[i], stroke_size));
  }

  // Save the SVG file
  std::ofstream svgFile( svg_filename.c_str() );
  if (svgFile.is_open())
  {
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
    return true;
  }
  return false;
}

}  // namespace matching
}  // namespace openMVG
