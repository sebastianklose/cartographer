#ifndef CARTOGRAPHER_IO_INTERNAL_TESTING_SERIALIZED_TEST_TEXT_PROTO_H_
#define CARTOGRAPHER_IO_INTERNAL_TESTING_SERIALIZED_TEST_TEXT_PROTO_H_

#include <string>

namespace cartographer {
namespace io {
namespace testing {

static const std::string kLegacyTextProto = R"(
  bla: "blubb"
  #
)";

}
}  // namespace io
}  // namespace cartographer

#endif  // ifndef CARTOGRAPHER_IO_INTERNAL_TESTING_SERIALIZED_TEST_TEXT_PROTO_H_
