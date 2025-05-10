/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef GZ_GAZEBO_SYSTEMS_HELICOPTERCONTROL_HH_
#define GZ_GAZEBO_SYSTEMS_HELICOPTERCONTROL_HH_

#include <memory>
#include <optional>

#include <gz/sim/System.hh>

namespace gz::sim::systems
{
    // Forward declaration of private data class
    class HelicopterControlPrivate;

    /// \brief Helicopter control system plugin for Ignition Gazebo
    ///
    /// This system controls the helicopter's altitude using a PID controller
    /// and by changing the angle of attack of the ingenuity helicopter blades.
    ///
    /// ## System Parameters
    /// `<kp_alt>`, `<ki_alt>`, `<kd_alt>` PID gains for altitude control.
    ///
    /// `<desired_altitude>` Desired altitude for the helicopter to maintain.
    ///
    /// `<link_name>` The name of the link to which the forces and torques are applied.
    ///
    class HelicopterControl : public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPreUpdate
    {
        /// \brief Constructor
    public:
        HelicopterControl();

        /// \brief Destructor
    public:
        ~HelicopterControl() override = default;

        // Documentation inherited
    public:
        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &_eventMgr) override;

        // Documentation inherited
    public:
        void PreUpdate(
            const gz::sim::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm) override;

        /// \brief Private data pointer for PIMPL pattern
    private:
        std::unique_ptr<HelicopterControlPrivate> dataPtr;
    };
} // namespace gz::sim::systems

#endif // GZ_GAZEBO_SYSTEMS_HELICOPTERCONTROL_HH_
