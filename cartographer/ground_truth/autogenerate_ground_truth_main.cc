/*
 * Copyright 2016 The Cartographer Authors
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

#include <cmath>
#include <fstream>
#include <string>

#include "cartographer/common/port.h"
#include "cartographer/ground_truth/proto/relations.pb.h"
#include "cartographer/io/mapping_state_deserializer.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/transform/transform.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(
    pose_graph_filename, "",
    "Proto stream file containing the pose graph used to generate ground truth "
    "data.");
DEFINE_string(output_filename, "", "File to write the ground truth proto to.");
DEFINE_double(min_covered_distance, 100.,
              "Minimum covered distance in meters before a loop closure is "
              "considered a candidate for autogenerated ground truth.");
DEFINE_double(outlier_threshold_meters, 0.15,
              "Distance in meters beyond which constraints are considered "
              "outliers.");
DEFINE_double(outlier_threshold_radians, 0.02,
              "Distance in radians beyond which constraints are considered "
              "outliers.");

namespace cartographer {
namespace ground_truth {
namespace {

std::vector<double> ComputeCoveredDistance(
    const mapping::proto::Trajectory& trajectory) {
  std::vector<double> covered_distance;
  covered_distance.push_back(0.);
  CHECK_GT(trajectory.node_size(), 0)
      << "Trajectory does not contain any nodes.";
  for (int i = 1; i < trajectory.node_size(); ++i) {
    const auto last_pose = transform::ToRigid3(trajectory.node(i - 1).pose());
    const auto this_pose = transform::ToRigid3(trajectory.node(i).pose());
    covered_distance.push_back(
        covered_distance.back() +
        (last_pose.inverse() * this_pose).translation().norm());
  }
  return covered_distance;
}

// We pick the representative node in the middle of the submap.
//
// TODO(whess): Should we consider all nodes inserted into the submap and
// exclude, e.g. based on large relative linear or angular distance?
std::vector<int> ComputeSubmapRepresentativeNode(
    const mapping::proto::PoseGraph& pose_graph) {
  std::vector<int> submap_to_node_index;
  for (const auto& constraint : pose_graph.constraint()) {
    if (constraint.tag() !=
        mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP) {
      continue;
    }
    CHECK_EQ(constraint.submap_id().trajectory_id(), 0);
    CHECK_EQ(constraint.node_id().trajectory_id(), 0);

    const int next_submap_index = static_cast<int>(submap_to_node_index.size());
    const int submap_index = constraint.submap_id().submap_index();
    if (submap_index <= next_submap_index) {
      continue;
    }

    CHECK_EQ(submap_index, next_submap_index + 1);
    submap_to_node_index.push_back(constraint.node_id().node_index());
  }
  return submap_to_node_index;
}

proto::GroundTruth GenerateGroundTruth(
    const mapping::proto::PoseGraph& pose_graph,
    const double min_covered_distance, const double outlier_threshold_meters,
    const double outlier_threshold_radians) {
  const mapping::proto::Trajectory& trajectory = pose_graph.trajectory(0);
  const std::vector<double> covered_distance =
      ComputeCoveredDistance(trajectory);

  const std::vector<int> submap_to_node_index =
      ComputeSubmapRepresentativeNode(pose_graph);

  int num_outliers = 0;
  proto::GroundTruth ground_truth;
  for (const auto& constraint : pose_graph.constraint()) {
    // We're only interested in loop closure constraints.
    if (constraint.tag() ==
        mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP) {
      continue;
    }

    // For some submaps at the very end, we have not chosen a representative
    // node, but those should not be part of loop closure anyway.
    CHECK_EQ(constraint.submap_id().trajectory_id(), 0);
    CHECK_EQ(constraint.node_id().trajectory_id(), 0);
    if (constraint.submap_id().submap_index() >=
        static_cast<int>(submap_to_node_index.size())) {
      continue;
    }
    const int matched_node = constraint.node_id().node_index();
    const int representative_node =
        submap_to_node_index.at(constraint.submap_id().submap_index());

    // Covered distance between the two should not be too small.
    if (std::abs(covered_distance.at(matched_node) -
                 covered_distance.at(representative_node)) <
        min_covered_distance) {
      continue;
    }

    // Compute the transform between the nodes according to the solution and
    // the constraint.
    const transform::Rigid3d solution_pose1 =
        transform::ToRigid3(trajectory.node(representative_node).pose());
    const transform::Rigid3d solution_pose2 =
        transform::ToRigid3(trajectory.node(matched_node).pose());
    const transform::Rigid3d solution =
        solution_pose1.inverse() * solution_pose2;

    const transform::Rigid3d submap_solution = transform::ToRigid3(
        trajectory.submap(constraint.submap_id().submap_index()).pose());
    const transform::Rigid3d submap_solution_to_node_solution =
        solution_pose1.inverse() * submap_solution;
    const transform::Rigid3d node_to_submap_constraint =
        transform::ToRigid3(constraint.relative_pose());
    const transform::Rigid3d expected =
        submap_solution_to_node_solution * node_to_submap_constraint;

    const transform::Rigid3d error = solution * expected.inverse();

    if (error.translation().norm() > outlier_threshold_meters ||
        transform::GetAngle(error) > outlier_threshold_radians) {
      ++num_outliers;
      continue;
    }
    auto* const new_relation = ground_truth.add_relation();
    new_relation->set_timestamp1(
        trajectory.node(representative_node).timestamp());
    new_relation->set_timestamp2(trajectory.node(matched_node).timestamp());
    *new_relation->mutable_expected() = transform::ToProto(expected);
  }
  LOG(INFO) << "Generated " << ground_truth.relation_size()
            << " relations and ignored " << num_outliers << " outliers.";
  return ground_truth;
}

mapping::proto::PoseGraph ReadPoseGraph(
    const std::string& pose_graph_filename) {
  io::ProtoStreamReader reader(pose_graph_filename);
  io::MappingStateDeserializer map_deserializer(&reader);
  CHECK_EQ(map_deserializer.pose_graph().trajectory_size(), 1)
      << "Only pose graphs containing a single trajectory are supported.";
  return map_deserializer.pose_graph();
}

void Run(const std::string& pose_graph_filename,
         const std::string& output_filename, const double min_covered_distance,
         const double outlier_threshold_meters,
         const double outlier_threshold_radians) {
  LOG(INFO) << "Reading pose graph from '" << pose_graph_filename << "'...";
  mapping::proto::PoseGraph pose_graph = ReadPoseGraph(pose_graph_filename);

  LOG(INFO) << "Autogenerating ground truth relations...";
  const proto::GroundTruth ground_truth =
      GenerateGroundTruth(pose_graph, min_covered_distance,
                          outlier_threshold_meters, outlier_threshold_radians);
  LOG(INFO) << "Writing " << ground_truth.relation_size() << " relations to '"
            << output_filename << "'.";
  {
    std::ofstream output_stream(output_filename,
                                std::ios_base::out | std::ios_base::binary);
    CHECK(ground_truth.SerializeToOstream(&output_stream))
        << "Could not serialize ground truth data.";
    output_stream.close();
    CHECK(output_stream) << "Could not write ground truth data.";
  }
}

}  // namespace
}  // namespace ground_truth
}  // namespace cartographer

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::SetUsageMessage(
      "\n\n"
      "This program semi-automatically generates ground truth data from a\n"
      "pose graph proto.\n"
      "\n"
      "The input should contain a single trajectory and should have been\n"
      "manually assessed to be correctly loop closed. Small local distortions\n"
      "are acceptable if they are tiny compared to the errors we want to\n"
      "assess using the generated ground truth data.\n"
      "\n"
      "All loop closure constraints separated by long covered distance are\n"
      "included in the output. Outliers are removed.\n");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_pose_graph_filename.empty() || FLAGS_output_filename.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0], "autogenerate_ground_truth");
    return EXIT_FAILURE;
  }
  ::cartographer::ground_truth::Run(
      FLAGS_pose_graph_filename, FLAGS_output_filename,
      FLAGS_min_covered_distance, FLAGS_outlier_threshold_meters,
      FLAGS_outlier_threshold_radians);
}
