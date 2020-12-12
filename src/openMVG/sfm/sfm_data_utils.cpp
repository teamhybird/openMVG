// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "openMVG/sfm/sfm_data.hpp"

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

namespace openMVG {
namespace sfm {

// CPM: Allow feature paths to include parent paths if the feature directory has '@' at the end of it
// so we can support video_a/frame_1.desc and video_b/frame_1.desc
const std::string generate_feature_path(const std::string& feat_directory, 
  const std::string& sImageName, const std::string& extension)
{
  std::string dir = feat_directory;
  int nd = 0;
  while (!dir.empty() && dir.back() == '@')
  {
    ++nd;
    dir.pop_back();
  }
  if (nd)
  {
    std::vector<std::string> pth = stlplus::filespec_elements(sImageName);
    int ioffset = pth.size() - nd - 1;
    if (ioffset >= 0)
    {
      for (int i = 0; i < nd; ++i)
      {
        dir = stlplus::folder_append_separator(dir);
        dir += pth[ioffset + i];
      }
    }
    else
    {
      // Error
    }
  }
  const std::string basename = stlplus::basename_part(sImageName);
  const std::string featFile = stlplus::create_filespec(dir, basename, extension);
  return featFile;
}

void GroupSharedIntrinsics(SfM_Data & sfm_data)
{
  Views & views = sfm_data.views;
  Intrinsics & intrinsics = sfm_data.intrinsics;

  // Build hash & build a set of the hash in order to maintain unique Ids
  std::set<size_t> hash_index;
  std::vector<size_t> hash_value;

  for (const auto & intrinsic_it : intrinsics)
  {
    const cameras::IntrinsicBase * intrinsicData = intrinsic_it.second.get();
    const size_t hashVal = intrinsicData->hashValue();
    hash_index.insert(hashVal);
    hash_value.push_back(hashVal);
  }

  // From hash_value(s) compute the new index (old to new indexing)
  Hash_Map<IndexT, IndexT> old_new_reindex;
  size_t i = 0;
  for (const auto & intrinsic_it : intrinsics)
  {
    old_new_reindex[intrinsic_it.first] = std::distance(hash_index.cbegin(), hash_index.find(hash_value[i]));
    ++i;
  }
  //--> Save only the required Intrinsics (do not need to keep all the copy)
  Intrinsics intrinsic_updated;
  for (const auto & intrinsic_it : intrinsics)
  {
    intrinsic_updated[old_new_reindex[intrinsic_it.first]] = intrinsics[intrinsic_it.first];
  }
  // Update intrinsics (keep only the necessary ones) -> swapping
  intrinsics.swap(intrinsic_updated);

  // Update views intrinsic IDs (since some intrinsic position have changed in the map)
  for (auto & view_it: views)
  {
    View * v = view_it.second.get();
    // Update the Id only if a corresponding index exists
    if (old_new_reindex.count(v->id_intrinsic))
      v->id_intrinsic = old_new_reindex[v->id_intrinsic];
  }
}

} // namespace sfm
} // namespace openMVG
