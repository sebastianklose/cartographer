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

#ifndef CARTOGRAPHER_IO_MAP_FORMAT_DESERIALIZER_H_
#define CARTOGRAPHER_IO_MAP_FORMAT_DESERIALIZER_H_

#include <iterator>

#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "google/protobuf/util/message_differencer.h"

namespace cartographer {
namespace io {

// Custom iterator for `SerializedData` through a ProtoStreamReaderInterface.
class SerializedDataIterator
    : public std::iterator<std::input_iterator_tag,
                           mapping::proto::SerializedData> {
 public:
  SerializedDataIterator();
  SerializedDataIterator(const SerializedDataIterator& other);
  explicit SerializedDataIterator(ProtoStreamReaderInterface* reader);

  SerializedDataIterator& operator++() {
    ReadNext();
    return *this;
  }

  SerializedDataIterator operator++(int) {
    SerializedDataIterator tmp(*this);
    operator++();
    return tmp;
  }

  bool operator==(const SerializedDataIterator& rhs) const {
    return reader_ == rhs.reader_ &&
           google::protobuf::util::MessageDifferencer::Equals(data_, rhs.data_);
  }

  bool operator!=(const SerializedDataIterator& rhs) const {
    return reader_ != rhs.reader_ ||
           !google::protobuf::util::MessageDifferencer::Equals(data_,
                                                               rhs.data_);
  }
  const mapping::proto::SerializedData& operator*() const { return data_; }
  mapping::proto::SerializedData& operator*() { return data_; }

  const mapping::proto::SerializedData* operator->() const { return &data_; }
  mapping::proto::SerializedData* operator->() { return &data_; }

 private:
  void ReadNext();

  ProtoStreamReaderInterface* reader_;
  mapping::proto::SerializedData data_;
};

// Class for representing a range (begin, end)-pair of serialized data, to
// enable use of range based for loops.
class SerializedDataRange {
 public:
  using iterator = SerializedDataIterator;
  using const_iterator = SerializedDataIterator;
  using value_type = mapping::proto::SerializedData;

  SerializedDataRange() : begin_(), end_() {}
  explicit SerializedDataRange(iterator begin) : begin_(begin), end_() {}

  iterator begin() const { return begin_; }
  const_iterator end() const { return end_; }

 private:
  const iterator begin_;
  const iterator end_;
};

// Class to help deserializing a previously serialized mapping state from a
// stream.
class MapFormatDeserializer {
 public:
  explicit MapFormatDeserializer(ProtoStreamReaderInterface* const reader);

  MapFormatDeserializer(const MapFormatDeserializer&) = delete;
  MapFormatDeserializer& operator=(const MapFormatDeserializer&) = delete;
  MapFormatDeserializer(MapFormatDeserializer&&) = delete;

  mapping::proto::PoseGraph& pose_graph() { return pose_graph_; }
  const mapping::proto::PoseGraph& pose_graph() const { return pose_graph_; }

  const mapping::proto::AllTrajectoryBuilderOptions&
  all_trajectory_builder_options() {
    return all_trajectory_builder_options_;
  }

  // Gives an iterable range which stops when the reader hits the end of the
  // stream. After one serializion pass, a subsequent call to
  // `GetSerializedData()` will return an empty range, because the stream has
  // already been completely read.
  SerializedDataRange GetSerializedData() { return serialized_data_range_; }

 private:
  ProtoStreamReaderInterface* reader_;

  mapping::proto::PoseGraph pose_graph_;
  mapping::proto::AllTrajectoryBuilderOptions all_trajectory_builder_options_;
  const SerializedDataRange serialized_data_range_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_MAP_FORMAT_DESERIALIZER_H_
