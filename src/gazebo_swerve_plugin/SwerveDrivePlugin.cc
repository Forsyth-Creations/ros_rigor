/*
 * Copyright (C) 2024 Forsyth Creations LLC
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

#include "SwerveDrivePlugin.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/twist.pb.h>

#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/DiffDriveOdometry.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/SpeedLimiter.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace gz::sim;
using namespace gz::sim::systems;

struct Commands
{
    double desiredPosition;

    Commands() : desiredPosition(0.0) {}
};

class SwerveDrivePlugin::SwerveDrivePluginPrivate
{
    /// \brief Callback for position subscriptions
    /// \param[in] _msg The message
    public: void OnCmdPos(const msgs::Twist &_msg);

}

//////////////////////////////////////////////////////////////////
SwerveDrivePlugin::SwerveDrivePlugin()
    : dataPtr(std::make_unique<SwerveDrivePluginPrivate>())
{
}
//////////////////////////////////////////////////////////////////


// Configure
void SwerveDrivePlugin::Configure(const Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  EntityComponentManager &_ecm,
                                  EventManager &_eventMgr)
{
    this->dataPtr->OnCmdPos(msgs::Twist());
}

void DiffDrivePrivate::OnCmdPos(const msgs::Twist &_msg)
{
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->enabled)
    {
        this->commands.desiredPosition = _msg.linear().x();
    }
}