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

#include "cartographer/io/map_format_parser.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

SerializedDataIterator::SerializedDataIterator() : reader_(nullptr) {}

SerializedDataIterator::SerializedDataIterator(
    const SerializedDataIterator& other)
    : reader_(other.reader_), data_(other.data_) {}

SerializedDataIterator::SerializedDataIterator(
    ProtoStreamReaderInterface* reader)
    : reader_(reader) {}

void SerializedDataIterator::ReadNext() {
  if (!reader_->ReadProto(&data_)) {
    *this = SerializedDataIterator();
  }
}

MapFormatParser::MapFormatParser(ProtoStreamReaderInterface* const reader)
    : reader_(reader) {
  CHECK(reader_->ReadProto(&pose_graph_));
  CHECK(reader_->ReadProto(&all_trajectory_builder_options_));
}

SerializedDataRange MapFormatParser::GetSerializedData() {
  return SerializedDataRange(SerializedDataIterator(reader_));
}

}  // namespace io
}  // namespace cartographer
