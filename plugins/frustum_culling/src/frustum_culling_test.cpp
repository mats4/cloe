/*
 * Copyright 2024 Robert Bosch GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * \file frustum_culling_test.cpp
 * \see  frustum_culling_conf.hpp
 * \see  frustum_culling_objects.cpp
 * \see  frustum_culling_lanes.cpp
 */

#include <gtest/gtest.h>

#include <fable/utility/gtest.hpp>   // for assert_validate
#include "frustum_culling_conf.hpp"  // for NoiseConf
#include "frustum_culling_objects.cpp"

using namespace cloe;  // NOLINT(build/namespaces)
using namespace cloe::component;

TEST(frustum_culling, deserialize) {
  component::FrustumCullingConf c;

  fable::assert_validate(c, R"({
    "reference_frame": {
        "x": 2.5,
        "y": 0.0,
        "z": 0.6,
        "roll": 0.0,
        "pitch": 0.1,
        "yaw": 0.0
    },
    "frustum" : {
      "clip_near": 0.0,
      "clip_far": 100.0,
      "fov_h": 0.7854,
      "fov_v": 0.7854,
      "offset_h": 0.0,
      "offset_v": 0.0
    }
  })");
}

class MyObjectSensor : public NopObjectSensor {
 public:
  MyObjectSensor() = default;

  void add_object(const Object& object) { objects_.push_back(std::make_shared<Object>(object)); }

  void clear_objects() { objects_.clear(); }
  void set_mount(const Eigen::Isometry3d& mount) { mount_ = mount; }
};

class ObjectFrustumCullingTest : public ::testing::Test {
 public:
  void createObjectFustumCullingSensor() {
    culling_sensor_ = std::make_shared<ObjectFrustumCulling>(
        "test_controller", config_, std::make_shared<MyObjectSensor>(object_sensor_));
  }

  std::shared_ptr<ObjectFrustumCulling> culling_sensor_;
  MyObjectSensor object_sensor_{};
  FrustumCullingConf config_{};
};

class RotatePoint : public ::testing::TestWithParam<std::array<double, 5>> {};

// for the definition of the input values, see the test class definition
// inputs are: original.x, original.y, rotation of point in mathematical positive direction (counter-clockwise),
// i.e. rotation of coordinate system clockwise, point in rotated coordinate system x, point in rotated coordinate system y.
INSTANTIATE_TEST_CASE_P(
    RotationTests,
    RotatePoint,
    ::testing::Values(std::array<double, 5>{0.0, 0.0, 0.0, 0.0, 0.0},
                      std::array<double, 5>{10.0, 0.0, M_PI / 2.0, 0.0, 10.0},
                      std::array<double, 5>{10.0, 10.0, M_PI / 2.0, -10.0, 10.0},
                      std::array<double, 5>{-10.0, -10.0, 3.0 / 4.0 * M_PI, 14.14, 0.0}));

TEST_P(RotatePoint, rotation) {
  std::array<double, 5> input_vector = GetParam();

  Point result = rotate_point(Point{input_vector[0], input_vector[1]}, input_vector[2]);

  EXPECT_NEAR(result.x, input_vector[3], 0.01);
  EXPECT_NEAR(result.y, input_vector[4], 0.01);
}

cloe::Object createDefaultObject() {
  cloe::Object object{};

  // Unfortunately cloe uses this complicated classes
  Eigen::Quaterniond qt = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

  object.pose.setIdentity();
  object.pose.linear() = qt.matrix();

  object.pose.translation() = Eigen::Vector3d{0.0, 0.0, 0.0};
  // set the length, width and height of the ego
  object.dimensions = Eigen::Vector3d{4.0, 2.0, 1.5};

  return object;
}

TEST_F(ObjectFrustumCullingTest, test_rotation) {
  // configuration
  fable::Conf c;
  config_.ref_frame.yaw = M_PI / 2.0;
  config_.ref_frame.from_conf(c);
  config_.frustum.fov_h = M_PI;
  config_.frustum.clip_far = 500;

  //set input object
  auto object = createDefaultObject();
  object.pose.translation() = Eigen::Vector3d{10.0, 15.0, 0.0};
  object_sensor_.add_object(object);

  createObjectFustumCullingSensor();

  auto objects = culling_sensor_->sensed_objects();
  if (objects.size() > 0) {
    EXPECT_NEAR(objects.front()->pose.translation().x(), 15.0, 0.01);
    EXPECT_NEAR(objects.front()->pose.translation().y(), -10.0, 0.01);
    EXPECT_NEAR(objects.front()->pose.translation().z(), 0.0, 0.01);
  } else {
    FAIL() << "Fail: No object inside the fov found. Evaluation not possible.";
  }
}

TEST_F(ObjectFrustumCullingTest, test_translation) {
  // configuration
  fable::Conf c;
  config_.ref_frame.x = 30.0;
  config_.ref_frame.y = 30.0;
  config_.ref_frame.from_conf(c);

  config_.frustum.offset_h = M_PI;
  config_.frustum.fov_h = M_PI;
  config_.frustum.clip_far = 500;

  //set input object
  auto object = createDefaultObject();
  object.pose.translation() = Eigen::Vector3d{10.0, 0.0, 0.0};
  object_sensor_.add_object(object);

  createObjectFustumCullingSensor();

  auto objects = culling_sensor_->sensed_objects();

  if (objects.size() > 0) {
    // expect the difference of the "config_.ref_frame - object.pose.translation"
    EXPECT_NEAR(objects.front()->pose.translation().x(), -20.0, 0.01);
    EXPECT_NEAR(objects.front()->pose.translation().y(), -30.0, 0.01);
    EXPECT_NEAR(objects.front()->pose.translation().z(), 0.0, 0.01);
  } else {
    FAIL() << "Fail: No object inside the fov found. Evaluation not possible.";
  }
}

TEST_F(ObjectFrustumCullingTest, rotation_and_translation) {
  // configuration
  fable::Conf c;
  config_.ref_frame.x = 30.0;
  config_.ref_frame.y = 40.0;
  config_.ref_frame.yaw = M_PI / 2.0;
  config_.ref_frame.from_conf(c);

  config_.frustum.offset_h = M_PI;
  config_.frustum.fov_h = M_PI;
  config_.frustum.clip_far = 500;

  //set input object
  auto object = createDefaultObject();
  object.pose.translation() = Eigen::Vector3d{10.0, 0.0, 0.0};
  object_sensor_.add_object(object);

  createObjectFustumCullingSensor();

  auto objects = culling_sensor_->sensed_objects();

  /*
                c2   
      x <-------|       
                |
                |       + P1
                |y
               \/       ^ x
                        |
                        |
                y <-----|  c1 
  */
  if (objects.size() > 0) {
    EXPECT_NEAR(objects.front()->pose.translation().x(), -40.0, 0.01);
    EXPECT_NEAR(objects.front()->pose.translation().y(), 20.0, 0.01);
    EXPECT_NEAR(objects.front()->pose.translation().z(), 0.0, 0.01);
  } else {
    FAIL() << "Fail: No object inside the fov found. Evaluation not possible.";
  }
}

TEST_F(ObjectFrustumCullingTest, rotation_and_translation_including_obj_rotation) {
  // configuration
  fable::Conf c;
  config_.ref_frame.x = 30.0;
  config_.ref_frame.y = 40.0;
  config_.ref_frame.yaw = M_PI / 2.0;
  config_.ref_frame.from_conf(c);

  config_.frustum.offset_h = M_PI;
  config_.frustum.fov_h = M_PI;
  config_.frustum.clip_far = 500;

  //set input object
  auto object = createDefaultObject();
  // rotate object by 135 degree in mathematical negative direction
  Eigen::Quaterniond qt = Eigen::AngleAxisd(3.0 / 4.0 * M_PI, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

  object.pose.linear() = qt.toRotationMatrix();
  object.pose.translation() = Eigen::Vector3d{10.0, 0.0, 0.0};
  object_sensor_.add_object(object);

  createObjectFustumCullingSensor();

  auto objects = culling_sensor_->sensed_objects();

  /*
                c2   
      x <-------|         ___
                |        /  /
                |       /x / P1
                |      /__/
                |y      /
               \/      \/ (direction of orientation)

                        ^ x
                        |
                        |
                y <-----|  c1 
  */
  if (objects.size() > 0) {
    EXPECT_NEAR(objects.front()->pose.translation().x(), -40.0, 0.01);
    EXPECT_NEAR(objects.front()->pose.translation().y(), 20.0, 0.01);
    EXPECT_NEAR(objects.front()->pose.translation().z(), 0.0, 0.01);

    // be aware that for calculating the rotation from an object, several solutions exist
    // hence you have to take care of all values returned by the eulerAngles function
    // here, the solution was designed in a way so it works with the input values of the test case
    Eigen::Vector3d yaw_pitch_roll = objects.front()->pose.rotation().eulerAngles(2, 1, 0);

    // Expect a rotation of 45 degree in positive direction in the new coordinate system.
    EXPECT_NEAR(yaw_pitch_roll[0], M_PI / 4.0, 0.01);
  } else {
    FAIL() << "Fail: No object inside the fov found. Evaluation not possible.";
  }
}

TEST_F(ObjectFrustumCullingTest, object_in_fov) {
  // configuration
  fable::Conf c;
  config_.ref_frame.x = 10.0;
  config_.ref_frame.y = 10.0;
  config_.ref_frame.yaw = 0.0;
  config_.ref_frame.from_conf(c);

  // cover 1 quadrant with field of view
  config_.frustum.offset_h = M_PI / 4.0;
  config_.frustum.fov_h = M_PI / 2.0;
  config_.frustum.clip_far = 500;

  // set input object
  auto object = createDefaultObject();
  object.pose.translation() = Eigen::Vector3d{30.0, 20.0, 0.0};
  object_sensor_.add_object(object);

  createObjectFustumCullingSensor();

  auto objects = culling_sensor_->sensed_objects();

  if (objects.size() > 0) {
    EXPECT_NEAR(objects.front()->pose.translation().x(), 20.0, 0.01);
    EXPECT_NEAR(objects.front()->pose.translation().y(), 10.0, 0.01);
    EXPECT_NEAR(objects.front()->pose.translation().z(), 0.0, 0.01);
  } else {
    FAIL() << "Fail: No object inside the fov found. Evaluation not possible.";
  }
}

TEST_F(ObjectFrustumCullingTest, object_outside_fov) {
  // configuration
  fable::Conf c;
  config_.ref_frame.x = 10.0;
  config_.ref_frame.y = 10.0;
  config_.ref_frame.yaw = 0.0;
  config_.ref_frame.from_conf(c);

  // cover 1 quadrant with field of view
  config_.frustum.offset_h = M_PI / 4.0;
  config_.frustum.fov_h = M_PI / 2.0;
  config_.frustum.clip_far = 500;

  // set input object
  auto object = createDefaultObject();
  object.pose.translation() = Eigen::Vector3d{30.0, -20.0, 0.0};
  object_sensor_.add_object(object);

  createObjectFustumCullingSensor();

  auto objects = culling_sensor_->sensed_objects();

  if (objects.size() > 0) {
    FAIL() << "Fail: Expected no object inside the field of view in this test case, but objects "
              "size was greater 0";
  }
}

TEST_F(ObjectFrustumCullingTest, velocity) {
  // configuration
  fable::Conf c;
  config_.ref_frame.x = 10.0;
  config_.ref_frame.y = 10.0;
  config_.ref_frame.yaw = M_PI/2.0;
  config_.ref_frame.from_conf(c);

  // cover 1 quadrant with field of view
  config_.frustum.offset_h = 0.0;
  config_.frustum.fov_h = M_PI;
  config_.frustum.clip_far = 500;

  // set input object
  auto object = createDefaultObject();
  object.pose.translation() = Eigen::Vector3d{30.0, 30.0, 0.0};
  object.velocity.x() = 10.0;
  object.acceleration.y() = 5.0;
  object.angular_velocity.x() = -1.0;
  object_sensor_.add_object(object);

  createObjectFustumCullingSensor();

  auto objects = culling_sensor_->sensed_objects();

  if (objects.size() > 0) {
    EXPECT_NEAR(objects.front()->velocity.y(), -10.0, 0.01);
    EXPECT_NEAR(objects.front()->acceleration.x(), 5.0, 0.01);
    EXPECT_NEAR(objects.front()->angular_velocity.y(), 1.0, 0.01);
    EXPECT_NEAR(objects.front()->dimensions.x(), 4.0, 0.01);
    EXPECT_NEAR(objects.front()->dimensions.y(), 2.0, 0.01);
  }
  else{
     FAIL() << "Expected at least 1 object";
  }
}