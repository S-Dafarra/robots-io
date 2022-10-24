/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_MANUALTRANSFORM_H
#define ROBOTSIO_MANUALTRANSFORM_H

#include <RobotsIO/Utils/TransformWithVelocity.h>

namespace RobotsIO {
    namespace Utils {
        class ManualTransform;
    }
}

class RobotsIO::Utils::ManualTransform : public RobotsIO::Utils::TransformWithVelocity
{

    RobotsIO::Utils::TransformWithVelocityStorage data_;

public:

    ManualTransform(const RobotsIO::Utils::TransformWithVelocityStorage& transformWithVelocity);

    ManualTransform();

    virtual ~ManualTransform() override;

    virtual Eigen::Transform<double, 3, Eigen::Affine> transform() override;

    virtual Eigen::Vector3d linear_velocity() override;

    virtual Eigen::Vector3d angular_velocity() override;

    virtual bool freeze(const bool blocking = false) override;

    void set_transform(const Eigen::Transform<double, 3, Eigen::Affine>& transform);

    void set_linear_velocity(const Eigen::Vector3d& linear_velocity);

    void set_angular_velocity(const Eigen::Vector3d& angular_velocity);

};

#endif // ROBOTSIO_MANUALTRANSFORM_H
