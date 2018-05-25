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

#include "cartographer/io/mapping_state_deserializer.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

namespace {
template <typename T>

T ReadMessageOrDie(ProtoStreamReaderInterface* const reader) {
  T msg_type;
  CHECK(reader->ReadProto(&msg_type)) << "Failed to read message from stream.";
  return msg_type;
}

mapping::proto::SerializationHeader ReadHeaderOrDie(
    ProtoStreamReaderInterface* const reader) {
  mapping::proto::SerializationHeader header;
  CHECK(reader->ReadProto(&header)) << "Failed to read SerializationHeader.";
  return header;
}

}  // namespace

MappingStateDeserializer::MappingStateDeserializer(
    ProtoStreamReaderInterface* const reader)
    : reader_(reader), header_(ReadHeaderOrDie(reader)) {
  mapping::proto::SerializedData serialized_data;
  CHECK(GetNextSerializedData(&serialized_data))
      << "Serialized stream misses PoseGraph.";

  CHECK_EQ(pose_graph_.trajectory_size(),
           all_trajectory_builder_options_.options_with_sensor_ids_size());
}

bool MappingStateDeserializer::GetNextSerializedData(
    mapping::proto::SerializedData* data) {
  return reader_->ReadProto(data);
}

}  // namespace io
}  // namespace cartographer
