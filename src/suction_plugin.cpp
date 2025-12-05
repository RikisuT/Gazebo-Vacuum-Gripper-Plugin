#include "suction_plugin.hpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Collision.hh>
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

using namespace gz;
using namespace sim;
using namespace systems;

SuctionPlugin::SuctionPlugin()
{
    std::cout << "======= SuctionPlugin constructed =======" << std::endl;
}

// Configuration
bool useSuctionRadius = false;

void SuctionPlugin::Configure(const Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              EntityComponentManager &_ecm,
                              EventManager &_eventMgr)
{
    this->modelEntity = _entity;
    std::cout << "======= Configure started. Entity ID: " << _entity << " =======" << std::endl;

    // Check if entity is valid and has Name component
    auto name = _ecm.Component<components::Name>(_entity);
    if (name)
    {
        std::cout << "Model name attached to plugin: " << name->Data() << std::endl;
    }
    else
    {
        std::cerr << "ERROR: Entity does not have Name component!" << std::endl;
    }

    // Read suction radius from SDF
    if (_sdf->HasElement("suction_radius"))
    {
        this->suctionRadius = _sdf->Get<double>("suction_radius");
        std::cout << "Suction radius set to: " << this->suctionRadius << std::endl;
    }
    else
    {
        std::cout << "Using default suction radius: " << this->suctionRadius << std::endl;
    }

    // Read filter name from SDF
    if (_sdf->HasElement("filter_name"))
    {
        this->filterName = _sdf->Get<std::string>("filter_name");
        std::cout << "Filter name set to: " << this->filterName << std::endl;
    }
    else
    {
        std::cout << "Using default filter name: " << this->filterName << std::endl;
    }

    // Read parent link name from SDF
    if (_sdf->HasElement("link_name"))
    {
        this->parentLinkName = _sdf->Get<std::string>("link_name");
        std::cout << "Parent link name set to: " << this->parentLinkName << std::endl;
    }

    // Read use_suction_radius from SDF
    if (_sdf->HasElement("use_suction_radius"))
    {
        this->useSuctionRadius = _sdf->Get<bool>("use_suction_radius");
        std::cout << "Use suction radius: " << (this->useSuctionRadius ? "TRUE" : "FALSE") << std::endl;
    }
    
    if (_sdf->HasElement("suction_force"))
    {
        this->suctionForce = _sdf->Get<double>("suction_force");
        std::cout << "Suction Force: " << this->suctionForce << std::endl;
    }
    
    if (_sdf->HasElement("suction_radius"))
    {
        this->suctionRadius = _sdf->Get<double>("suction_radius");
        std::cout << "Suction Radius: " << this->suctionRadius << std::endl;
    }

    if (_sdf->HasElement("suction_topic"))
    {
        this->suctionTopic = _sdf->Get<std::string>("suction_topic");
        std::cout << "Suction Topic: " << this->suctionTopic << std::endl;
    }

    // Create and advertise suction activation topic
    this->publisher = this->node.Advertise<msgs::Boolean>(this->suctionTopic);
    if (!this->publisher)
    {
        std::cerr << "ERROR: Failed to advertise topic: " << this->suctionTopic << std::endl;
    }
    else
    {
        std::cout << "Topic advertised successfully: " << this->suctionTopic << std::endl;
    }

    // Subscribe to suction activation topic
    if (!this->node.Subscribe(this->suctionTopic,
                              &SuctionPlugin::OnSuctionActivate,
                              this))
    {
        std::cerr << "ERROR: Failed to subscribe to topic: " << this->suctionTopic << std::endl;
    }
    else
    {
        std::cout << "Successfully subscribed to topic: " << this->suctionTopic << std::endl;
    }

    std::cout << "======= Configure completed =======" << std::endl;
}

void SuctionPlugin::PreUpdate(const UpdateInfo &_info,
                              EntityComponentManager &_ecm)
{
    // 1. Logic for Suction (Sensing/Actuation)
    if (this->suctionActive && this->attachedEntity == kNullEntity)
    {
        if (this->useSuctionRadius)
        {
            this->FindTargetRadius(_ecm);
        }
        // Contact mode is handled in PostUpdate, which sets targetEntityToAttach
    }

    // 2. Handle Attachment (Joint Creation)
    if (this->targetEntityToAttach != kNullEntity)
    {
        if (this->attachedEntity == kNullEntity) // Double check
        {
             // std::cout << "PreUpdate: Attaching to target " << this->targetEntityToAttach << std::endl;
             
             // Find gripper link
             Entity gripperLink = kNullEntity;
             if (!this->parentLinkName.empty())
             {
                auto links = _ecm.ChildrenByComponents(this->modelEntity, components::Link());
                for (const auto &link : links) {
                    auto nameComp = _ecm.Component<components::Name>(link);
                    if (nameComp && nameComp->Data() == this->parentLinkName) { gripperLink = link; break; }
                }
             }
             if (gripperLink == kNullEntity) {
                 auto links = _ecm.ChildrenByComponents(this->modelEntity, components::Link());
                 if (!links.empty()) gripperLink = links[0];
             }

             // Find target link
             Entity targetLink = kNullEntity;
             auto targetLinks = _ecm.ChildrenByComponents(this->targetEntityToAttach, components::Link());
             if (!targetLinks.empty()) targetLink = targetLinks[0];

             if (gripperLink != kNullEntity && targetLink != kNullEntity)
             {
                 this->jointEntity = _ecm.CreateEntity();
                 components::DetachableJointInfo jointInfo;
                 jointInfo.parentLink = gripperLink;
                 jointInfo.childLink = targetLink;
                 jointInfo.jointType = "fixed";
                 _ecm.CreateComponent(this->jointEntity, components::DetachableJoint(jointInfo));
                 _ecm.CreateComponent(this->jointEntity, components::Name("suction_joint"));
                 
                 this->attachedEntity = this->targetEntityToAttach;
                 std::cout << "SUCCESS: Attached to " << this->attachedEntity << std::endl;
             }
             else
             {
                 std::cerr << "ERROR: Failed to find links for attachment in PreUpdate." << std::endl;
             }
        }
        this->targetEntityToAttach = kNullEntity; // Reset
    }

    // 3. Handle Detachment
    if (!this->suctionActive && this->attachedEntity != kNullEntity)
    {
        this->Detach(_ecm);
    }
}

void SuctionPlugin::PostUpdate(const UpdateInfo &_info,
                               const EntityComponentManager &_ecm)
{
    // Sensing: Only look for targets if suction is active and we are NOT attached
    // AND we are in Contact Mode (Radius mode is done in PreUpdate)
    if (this->suctionActive && this->attachedEntity == kNullEntity && !this->useSuctionRadius)
    {
        this->FindTargetContact(_ecm);
    }
}

void SuctionPlugin::FindTargetRadius(EntityComponentManager &_ecm)
{
    // Find the gripper link
    Entity gripperLink = kNullEntity;
    if (!this->parentLinkName.empty())
    {
        auto links = _ecm.ChildrenByComponents(this->modelEntity, components::Link());
        for (const auto &link : links)
        {
            auto nameComp = _ecm.Component<components::Name>(link);
            if (nameComp && nameComp->Data() == this->parentLinkName) { gripperLink = link; break; }
        }
    }
    if (gripperLink == kNullEntity)
    {
        auto links = _ecm.ChildrenByComponents(this->modelEntity, components::Link());
        if (!links.empty()) gripperLink = links[0];
    }

    if (gripperLink == kNullEntity) return;

    // Calculate Gripper Link World Pose
    auto modelPose = _ecm.Component<components::Pose>(this->modelEntity);
    if (!modelPose) return;
    
    math::Pose3d gripperWorldPose = modelPose->Data();
    auto linkPose = _ecm.Component<components::Pose>(gripperLink);
    if (linkPose) gripperWorldPose = modelPose->Data() * linkPose->Data();

    Entity nearestEntity = kNullEntity;
    double minDistance = std::numeric_limits<double>::max();

    // Iterate all models to find candidates
    _ecm.Each<components::Name, components::Pose, components::Model>(
        [&](const Entity &_entity,
            const components::Name *_name,
            const components::Pose *_pose,
            const components::Model *) -> bool
        {
            if (_entity == this->modelEntity) return true;
            if (_name->Data().find(this->filterName) == std::string::npos) return true;

            double distance = this->CalculateDistance(gripperWorldPose, _pose->Data());
            
            if (distance <= this->suctionRadius)
            {
                // --- APPLY SUCTION FORCE ---
                // Find the first link of the target model to apply force to
                auto targetLinks = _ecm.ChildrenByComponents(_entity, components::Link());
                if (!targetLinks.empty())
                {
                    Entity targetLink = targetLinks[0];
                    
                    // Calculate vector from target to gripper
                    math::Vector3d direction = gripperWorldPose.Pos() - _pose->Data().Pos();
                    direction.Normalize();
                    
                    math::Vector3d force = direction * this->suctionForce;
                    
                    // Add External World Wrench
                    auto wrenchComp = _ecm.Component<components::ExternalWorldWrenchCmd>(targetLink);
                    if (!wrenchComp)
                    {
                        components::ExternalWorldWrenchCmd wrenchCmd;
                        msgs::Wrench wrenchMsg;
                        msgs::Set(wrenchMsg.mutable_force(), force);
                        wrenchCmd.Data() = wrenchMsg;
                        _ecm.CreateComponent(targetLink, wrenchCmd);
                    }
                    else
                    {
                        // Add to existing force (simplified, just overwriting for now as we are the suction)
                        msgs::Set(wrenchComp->Data().mutable_force(), force);
                    }
                    // std::cout << "Applying suction force " << this->suctionForce << " to " << _name->Data() << std::endl;
                }

                // Check for attachment
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestEntity = _entity;
                }
            }
            return true;
        });

    // If close enough (e.g. < 0.02m), schedule attachment
    if (nearestEntity != kNullEntity && minDistance < 0.02) 
    {
        this->targetEntityToAttach = nearestEntity;
        // std::cout << "FindTargetRadius: Target " << nearestEntity << " in range (" << minDistance << "). Scheduling attachment." << std::endl;
    }
}

void SuctionPlugin::FindTargetContact(const EntityComponentManager &_ecm)
{
    // Find the gripper link
    Entity gripperLink = kNullEntity;
    if (!this->parentLinkName.empty())
    {
        auto links = _ecm.ChildrenByComponents(this->modelEntity, components::Link());
        for (const auto &link : links)
        {
            auto nameComp = _ecm.Component<components::Name>(link);
            if (nameComp && nameComp->Data() == this->parentLinkName) { gripperLink = link; break; }
        }
    }
    if (gripperLink == kNullEntity)
    {
        auto links = _ecm.ChildrenByComponents(this->modelEntity, components::Link());
        if (!links.empty()) gripperLink = links[0];
    }

    if (gripperLink == kNullEntity) return;

    auto sensors = _ecm.ChildrenByComponents(gripperLink, components::Sensor());
    for (const auto &sensor : sensors)
    {
        auto contactData = _ecm.Component<components::ContactSensorData>(sensor);
        if (contactData)
        {
            for (const auto &contact : contactData->Data().contact())
            {
                Entity col1 = contact.collision1().id();
                Entity col2 = contact.collision2().id();
                
                auto GetModelFromCollision = [&](Entity _colEntity) -> Entity {
                    auto linkEntity = _ecm.ParentEntity(_colEntity);
                    if (linkEntity == kNullEntity) return kNullEntity;
                    return _ecm.ParentEntity(linkEntity);
                };

                Entity model1 = GetModelFromCollision(col1);
                Entity model2 = GetModelFromCollision(col2);
                
                Entity potentialTarget = kNullEntity;
                if (model1 == this->modelEntity && model2 != kNullEntity) potentialTarget = model2;
                else if (model2 == this->modelEntity && model1 != kNullEntity) potentialTarget = model1;
                
                if (potentialTarget != kNullEntity)
                {
                    auto nameComp = _ecm.Component<components::Name>(potentialTarget);
                    if (nameComp && nameComp->Data().find(this->filterName) != std::string::npos)
                    {
                        this->targetEntityToAttach = potentialTarget;
                        // std::cout << "FindTargetContact: Match found " << potentialTarget << ". Scheduling attachment." << std::endl;
                        return; // Found one, that's enough
                    }
                }
            }
        }
    }
}

void SuctionPlugin::Detach(EntityComponentManager &_ecm)
{
    if (this->jointEntity != kNullEntity)
    {
        std::cout << "Detaching entity: " << this->attachedEntity << std::endl;
        _ecm.RequestRemoveEntity(this->jointEntity);
        this->jointEntity = kNullEntity;
        this->attachedEntity = kNullEntity;
    }
}

void SuctionPlugin::OnSuctionActivate(const msgs::Boolean &_msg)
{
    bool newState = _msg.data();
    std::cout << "======= ALERT: Received suction command: " << (newState ? "ACTIVATE" : "DEACTIVATE") << " =======" << std::endl;
    this->suctionActive = newState;
}

double SuctionPlugin::CalculateDistance(const math::Pose3d &_pose1,
                                        const math::Pose3d &_pose2)
{
    return _pose1.Pos().Distance(_pose2.Pos());
}

GZ_ADD_PLUGIN(SuctionPlugin,
              gz::sim::System,
              SuctionPlugin::ISystemConfigure,
              SuctionPlugin::ISystemPreUpdate,
              SuctionPlugin::ISystemPostUpdate)