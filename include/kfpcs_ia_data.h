#ifndef KFPCS_IA_DATA_H_
#define KFPCS_IA_DATA_H_

#include <string>
#include <pcl/common/eigen.h>

 std::string source_file = "../data/office2.ply";
 std::string target_file = "../data/office3.ply";

 float voxel_size = 0.1f; // optimally set to estimated scan resolution in overlap
 float min_contrast = 0.01f; // set to extract approximately 1000-5000 keypoints
 float approx_overlap = 0.9f; // rough estimation of scan overlap
 float abort_score = 0.0f; // zero to avoid early termination

#endif // KFPCS_IA_DATA_H_