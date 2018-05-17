#ifndef CARTOGRAPHER_IO_INTERNAL_TESTING_SERIALIZED_TEST_TEXT_PROTO_H_
#define CARTOGRAPHER_IO_INTERNAL_TESTING_SERIALIZED_TEST_TEXT_PROTO_H_

#include <string>

namespace cartographer {
namespace io {
namespace testing {

// Single proto messages are separated by `#` for simple parsing.
// The first proto is a `PoseGraph`, the second `AllTrajectoryBuilderOptions`,
// then followed by a sequence of `SerializedData`.
static const std::string kLegacyTextProto = R"(
  constraint {
    submap_id {
      trajectory_id: 0
      submap_index: 0
    }
    node_id {
      trajectory_id: 0
      node_index: 0
    }
    relative_pose {
      translation {
        x: 0.0
        y: 0.0
        z: 0.0
      }
      rotation {
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
      }
    }
    translation_weight: 1.0
    rotation_weight: 1.0
    tag: INTRA_SUBMAP
  }
  trajectory {
    trajectory_id: 0
    node {
      node_index: 0
      timestamp: 0
      pose {
        translation {
          x: 0.0
          y: 0.0
          z: 0.0
        }
        rotation {
          x: 0.0
          y: 0.0
          z: 0.0
          z: 1.0
        }
      }
    }
    submap {
      submap_index: 0
      pose {
        translation {
          x: 0.0
          y: 0.0
          z: 0.0
        }
        rotation {
          x: 0.0
          y: 0.0
          z: 0.0
          z: 1.0
        }
      }
    }
  }
  landmark_poses {
  }
#
  options_with_sensor_ids {
    sensor_id {
      type: RANGE
      id: "laser_scanner_0"
    }
    sensor_id {
      type: IMU
      id: "imu_0"
    }
    trajectory_builder_options {
      trajectory_builder_2d_options {
        min_range: 0.1
        max_range: 40.0
        min_z: 0.0
        max_z: 2.0
      }
      pure_localization: false
      initial_trajectory_pose {
        relative_pose {
          translation {
            x: 0.0
            y: 0.0
            z: 0.0
          }
          rotation {
            x: 0.0
            y: 0.0
            z: 0.0
            w: 1.0
          }
        }
        to_trajectory_id: 0
        timestamp: 0
      }
    }
  }
#
  node {
    node_id {
      trajectory_id: 0
      node_index: 0
    }
    node_data {
      timestamp: 0
      local_pose {
        translation {
          x: 0.0
          y: 0.0
          z: 0.0
        }
        rotation {
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
        }
      }
    }
  }
#
  submap {
    submap_id {
    }
    submap_2d {
    }
  }
)";

}  // namespace testing
}  // namespace io
}  // namespace cartographer

#endif  // ifndef CARTOGRAPHER_IO_INTERNAL_TESTING_SERIALIZED_TEST_TEXT_PROTO_H_
