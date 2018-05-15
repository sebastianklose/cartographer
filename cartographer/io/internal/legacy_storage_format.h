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
#ifndef CARTOGRAPHER_IO_INTERNAL_LEGACY_FORMAT_H_
#define CARTOGRAPHER_IO_INTERNAL_PROTO_STREAM_H_

#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"

// Helper functions to serialize & deserialize the "legacy" format, which refers
// to the last file format that did not include a `SerializationHeader`.
namespace cartographer {
namespace io {

// Serialize mapping state to a pbstream file.
void ToLegacyFormat(
    const mapping::PoseGraph& pose_graph,
    const mapping::proto::AllTrajectoryBuilderOptions& builder_options,
    ProtoStreamWriterInterface* const writer);

// De-serialize a pbstream file to a valid mapping state.
void FromLegacyFormat(ProtoStreamReaderInterface* const reader,
                      bool load_frozen_state,
                      mapping::MapBuilderInterface* map_builder,
                      mapping::PoseGraph* pose_graph);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_INTERNAL_PROTO_STREAM_H_
