/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
*/
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class fish1_animate : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

        // create the animation
        gazebo::common::PoseAnimationPtr anim(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("fish2_animate", 20.0, true));

        gazebo::common::PoseKeyFrame *key;

        // set starting location of the box
        key = anim->CreateKeyFrame(0);
        // key->Translation(ignition::math::Vector3d(26, 20, -92));
        key->Translation(ignition::math::Vector3d(27.5, 15, -93));
        key->Rotation(ignition::math::Quaterniond(0, 0, -0.2));

        // siwm directly
        key = anim->CreateKeyFrame(7.0);
        key->Translation(ignition::math::Vector3d(23.4, 8.5, -93));
        key->Rotation(ignition::math::Quaterniond(0, 0, -0.2));

        // turn around
        key = anim->CreateKeyFrame(10);
        key->Translation(ignition::math::Vector3d(24.3, 9, -93));
        key->Rotation(ignition::math::Quaterniond(0, 0, 2.1));

        // swim back
        key = anim->CreateKeyFrame(17);
        key->Translation(ignition::math::Vector3d(25, 17, -92.3));
        key->Rotation(ignition::math::Quaterniond(0, 0, 2.1));

       // set starting location of the box
        key = anim->CreateKeyFrame(20);
        key->Translation(ignition::math::Vector3d(27.5,15, -93));
        key->Rotation(ignition::math::Quaterniond(0, 0, -0.2));

        // set the animation
        _parent->SetAnimation(anim);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(fish1_animate)
}
