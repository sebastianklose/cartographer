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

#include <memory>
#include <sstream>

#include "cartographer/io/internal/in_memory_proto_stream.h"
#include "cartographer/io/internal/legacy_storage_format.h"
#include "cartographer/io/internal/testing/serialized_test_text_proto.h"
#include "google/protobuf/text_format.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

using common::make_unique;
using google::protobuf::Message;
using mapping::proto::AllTrajectoryBuilderOptions;
using mapping::proto::PoseGraph;
using mapping::proto::SerializedData;

constexpr char kProtoDelim = '#';

// open text file with a set of text-protos that mimic a pbstream.
std::unique_ptr<InMemoryProtoStreamReader> CreateInMemoryReaderFromTextProto(
    const std::string& file_string) {
  std::queue<std::unique_ptr<Message>> proto_queue;
  std::istringstream in_stream(file_string);

  // Parse the PoseGraph proto.
  std::string proto_string;
  proto_queue.push(make_unique<PoseGraph>());
  std::getline(in_stream, proto_string, kProtoDelim);
  CHECK(google::protobuf::TextFormat::ParseFromString(
      proto_string, proto_queue.back().get()));

  proto_queue.push(make_unique<AllTrajectoryBuilderOptions>());
  std::getline(in_stream, proto_string, kProtoDelim);
  CHECK(google::protobuf::TextFormat::ParseFromString(
      proto_string, proto_queue.back().get()));

  // Parse all the remaining SerializedData protos
  // while (std::getline(in_stream, proto_string, kProtoDelim)) {
  //  proto_queue.emplace(new Message());
  //  CHECK(google::protobuf::TextFormat::ParseFromString(
  //      proto_string, proto_queue.back().get()));
  //}

  return make_unique<InMemoryProtoStreamReader>(std::move(proto_queue));
}

// This test checks, if the serialization works.
TEST(LegacyStorageFormatTest, FromLegacyFormatWorks) {
  // Load text proto into in_memory_reader.
  auto reader = CreateInMemoryReaderFromTextProto(testing::kLegacyTextProto);
  // deserialize with io::FromLegacyFormat(...)
  // verify invariants.
}

// This test checks, if the serialization works.
TEST(LegacyStorageFormatTest, ToLegacyFormatWorks) {
  // Load text proto
  // serialize with io::ToLegacyFormat(...)
  // deserialize with io::FromLegacyFormat(...)
  // verify invariants.
}

}  // namespace
}  // namespace io
}  // namespace cartographer
