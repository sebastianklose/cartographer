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
#ifndef CARTOGRAPHER_IO_LEGACY_FORMAT_H_
#define CARTOGRAPHER_IO_PROTO_STREAM_H_

#include "cartographer/io/proto_stream_interface.h"

namespace cartographer {
  namespace io {

    // Helper function to serialize.
    void ToLegacyFormat(const mapping::PoseGraph& pose_graph,
        const proto::AllTrajectoryBuilderOptions& builder_options,
        ProtoStreamWriterInterface* const writer);

    // Helper function to de-serialize.
    void FromLegacyFormat(ProtoStreamReaderInterface* const reader,
        bool load_frozen_state,
        mapping::PoseGraph* pose_graph,
        proto::AllTrajectoryBuilderOptions* builder_options);

  } // namespace io
} // namespace cartographer

#endif // CARTOGRAPHER_IO_PROTO_STREAM_H_
