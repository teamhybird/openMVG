// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <string>

using namespace openMVG;
using namespace openMVG::sfm;

// Merge the rootpath into local paths so that directories can be merged
// This should be done better by making them relative
void RemoveRootPath(SfM_Data &sfm_data)
{
  for (auto &view : sfm_data.views)
  {
    view.second->s_Img_path = stlplus::create_filespec(sfm_data.s_root_path, view.second->s_Img_path);
  }
  sfm_data.s_root_path = "";
}

// Merge two SFM data files - well really just append them
// we do not remove duplicates
void Merge(SfM_Data &sfm_data, const SfM_Data &sfm_merge)
{
  // Find a suitable offset to add to each key to make it unique
  IndexT viewOffset = sfm_data.views.size();
  IndexT poseOffset = sfm_data.poses.size();
  IndexT intrinsicOffset = sfm_data.intrinsics.size();
  // Do not assume that keys are contiguous starting from zero, since they are unordered maps
  for (const auto &view : sfm_data.views)
  {
    viewOffset = std::max(viewOffset, view.first + 1);
  }
  for (const auto &pose : sfm_data.poses)
  {
    poseOffset = std::max(poseOffset, pose.first + 1);
  }
  for (const auto &intrinsic : sfm_data.intrinsics)
  {
    intrinsicOffset = std::max(intrinsicOffset, intrinsic.first + 1);
  }
  std::cout << "  Adding offset " << viewOffset << " to views, "
    << poseOffset << " to poses and " << intrinsicOffset << " to intrinsics\n";

  // Merge (append) the views
  for (const auto &view : sfm_merge.views)
  {
    auto &v = sfm_data.views[view.first + viewOffset] = view.second;
    v->id_view = view.first + viewOffset;
    v->id_intrinsic += intrinsicOffset;
    v->id_pose += viewOffset;
  }

  // Merge (append) the poses
  for (const auto &pose : sfm_merge.poses)
  {
    sfm_data.poses[pose.first + poseOffset] = pose.second;
  }

  // Merge (append) the intrinsics, which contain derived class data (transform, inverse transform, parameters)
  for (const auto &intrinsic : sfm_merge.intrinsics)
  {
    sfm_data.intrinsics[intrinsic.first + intrinsicOffset] = intrinsic.second;
  }

  // Not merging structure or control points
  // ...
}

// Convert from a SfM_Data format to another
int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string
      sSfM_Data_Filename_In,
      sSfM_Data_Filename_Merge,
      sSfM_Data_Filename_Out;

  cmd.add(make_option('i', sSfM_Data_Filename_In, "input_file"));
  cmd.add(make_option('m', sSfM_Data_Filename_Merge, "merge_file"));
  cmd.add(make_switch('V', "VIEWS"));
  cmd.add(make_switch('I', "INTRINSICS"));
  cmd.add(make_switch('E', "EXTRINSICS"));
  cmd.add(make_switch('S', "STRUCTURE"));
  cmd.add(make_switch('C', "CONTROL_POINTS"));
  cmd.add(make_option('o', sSfM_Data_Filename_Out, "output_file"));

  try
  {
    if (argc == 1)
      throw std::string("Invalid command line parameter.");
    cmd.process(argc, argv);
  }
  catch (const std::string &s)
  {
    std::cerr << "Usage: " << argv[0] << '\n'
              << "[-i|--input_file] path to the input SfM_Data scene\n"
              << "[-m|--merge_file] path to an optional input SfM_Data scene to merge\n"
              << "[-o|--output_file] path to the output SfM_Data scene\n"
              << "\t .json, .bin, .xml, .ply, .baf\n"
              << "\n[Options to export partial data (by default all data are exported)]\n"
              << "\nUsable for json/bin/xml format\n"
              << "[-V|--VIEWS] export views\n"
              << "[-I|--INTRINSICS] export intrinsics\n"
              << "[-E|--EXTRINSICS] export extrinsics (view poses)\n"
              << "[-S|--STRUCTURE] export structure\n"
              << "[-C|--CONTROL_POINTS] export control points\n"
              << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  if (sSfM_Data_Filename_In.empty() || sSfM_Data_Filename_Out.empty())
  {
    std::cerr << "Invalid input or output filename." << std::endl;
    return EXIT_FAILURE;
  }

  // OptionSwitch is cloned in cmd.add(),
  // so we must use cmd.used() instead of testing OptionSwitch.used
  int flags =
      (cmd.used('V') ? VIEWS : 0) | (cmd.used('I') ? INTRINSICS : 0) | (cmd.used('E') ? EXTRINSICS : 0) | (cmd.used('S') ? STRUCTURE : 0) | (cmd.used('C') ? CONTROL_POINTS : 0);

  flags = (flags) ? flags : ALL;

  // Load input SfM_Data scene
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename_In, ESfM_Data(ALL)))
  {
    std::cerr << std::endl
              << "The input SfM_Data file \"" << sSfM_Data_Filename_In << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  if (!sSfM_Data_Filename_Merge.empty())
  {
    SfM_Data sfm_merge;
    if (!Load(sfm_merge, sSfM_Data_Filename_Merge, ESfM_Data(ALL)))
    {
      std::cerr << std::endl
                << "The merge SfM_Data file \"" << sSfM_Data_Filename_Merge << "\" cannot be read." << std::endl;
      return EXIT_FAILURE;
    }
    else
    {
      RemoveRootPath(sfm_data);
      RemoveRootPath(sfm_merge);
      Merge(sfm_data, sfm_merge);
    }
  }

  // Export the SfM_Data scene in the expected format
  if (Save(
          sfm_data,
          sSfM_Data_Filename_Out.c_str(),
          ESfM_Data(flags)))
  {
    return EXIT_SUCCESS;
  }

  std::cerr
      << std::endl
      << "An error occured while trying to save \"" << sSfM_Data_Filename_Out << "\"." << std::endl;
  return EXIT_FAILURE;
}
