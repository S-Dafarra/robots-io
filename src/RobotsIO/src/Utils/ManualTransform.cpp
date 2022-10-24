/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/ManualTransform.h>

RobotsIO::Utils::ManualTransform::ManualTransform(const TransformWithVelocityStorage &transformWithVelocity)
    : data_(transformWithVelocity)
{

}

RobotsIO::Utils::ManualTransform::ManualTransform()
    :ManualTransform({Eigen::Transform<double, 3, Eigen::Affine>::Identity(),
                     Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()})
{

}

RobotsIO::Utils::ManualTransform::~ManualTransform()
{

}

Eigen::Transform<double, 3, Eigen::Affine> RobotsIO::Utils::ManualTransform::transform()
{
    return data_.transform;
}

Eigen::Vector3d RobotsIO::Utils::ManualTransform::linear_velocity()
{
    return data_.linear_velocity;
}

Eigen::Vector3d RobotsIO::Utils::ManualTransform::angular_velocity()
{
    return data_.angular_velocity;
}

bool RobotsIO::Utils::ManualTransform::freeze(const bool blocking)
{
    // Not needed
    return true;
}

void RobotsIO::Utils::ManualTransform::set_transform(const Eigen::Transform<double, 3, Eigen::Affine> &transform)
{
    data_.transform = transform;
}

void RobotsIO::Utils::ManualTransform::set_linear_velocity(const Eigen::Vector3d &linear_velocity)
{
    data_.linear_velocity = linear_velocity;
}

void RobotsIO::Utils::ManualTransform::set_angular_velocity(const Eigen::Vector3d &angular_velocity)
{
    data_.angular_velocity = angular_velocity;
}
