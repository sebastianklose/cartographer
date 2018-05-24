/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/io/map_format_deserializer.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(original_pose_graph_filename, "",
              "PoseGraph proto stream file that should be migrated.");
DEFINE_string(output_filename, "",
              "Filename for the migrated PoseGraph output file.");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::SetUsageMessage(
      "\n\n"
      "This program can be used to migrate old PoseGraph file formats to the\n"
      "current file-format version.");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_original_pose_graph_filename.empty() ||
      FLAGS_output_filename.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0],
                                       "migrate_pose_graph_file_format");
    return EXIT_FAILURE;
  }
}
