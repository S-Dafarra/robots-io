/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_ICUBHAND_H
#define ROBOTSIO_ICUBHAND_H

#include <Eigen/Dense>

#include <iCub/iKin/iKinFwd.h>

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <string>
#include <unordered_map>

namespace RobotsIO {
    namespace Hand {
        class iCubHand;
    }
}


class RobotsIO::Hand::iCubHand
{
public:
    iCubHand(const std::string& robot_name, const std::string& laterality, const std::string& port_prefix, const std::string& context, const bool& use_analogs, const std::string& thumb_version = "", const bool& use_abduction = true);

    iCubHand(const std::string& robot_name, const std::string& laterality, const std::string& port_prefix, const bool& use_analogs, const bool& use_analogs_bounds, const yarp::sig::Matrix& analog_bounds, const std::string& thumb_version = "", const bool& use_abduction = true);

    virtual ~iCubHand();

    std::pair<bool, std::unordered_map<std::string, Eigen::VectorXd>> encoders(const bool& blocking);

protected:
    std::pair<bool, yarp::sig::Vector> load_vector_double(const yarp::os::ResourceFinder& rf, const std::string key, const std::size_t size);

    yarp::os::Network yarp_;

    bool use_analogs_ = false;

    /**
     * Indicates whether the PolyDriver interface is available.
     */

    bool use_interface_analogs_ = false;

    bool use_interface_arm_ = false;

    /**
     * To be used if the interface is available.
     */

    yarp::dev::PolyDriver drv_analog_;

    yarp::dev::IAnalogSensor *ianalog_{nullptr};

    yarp::dev::IControlLimits *ilimits_{nullptr};

    yarp::dev::PolyDriver drv_arm_;

    yarp::dev::IEncoders *iarm_{nullptr};

    /**
     * To be used if the interface is not available.
     */

    yarp::os::BufferedPort<yarp::os::Bottle> port_analogs_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_arm_;


    /**
     * Utility struct to combine data relative to a single finger
     */
    struct Finger
    {
        iCub::iKin::iCubFinger iCubFinger;
        yarp::sig::Vector chain_joints;
        Eigen::VectorXd chain_joints_eigen;
    };

    /**
     * Instances of iCub::iKin::iCubFinger required to combine arm and analog encoders.
     */
    std::unordered_map<std::string, Finger> fingers_;

    /**
     * Optional analog bounds.
     */

    yarp::sig::Matrix analog_bounds_;

    bool use_bounds_ = false;

    /**
     * Option to avoid using the abduction, since it is often out of bounds.
     */
    bool use_abduction_ = true;

    /**
     * Log name to be used in messages printed by the class.
     */

    const std::string log_name_ = "iCubHand";

    /**
     * Internal buffers
     */
    yarp::sig::Vector analogs_;

    yarp::sig::Vector finger_encoders_;

    yarp::sig::Vector arm_encoders_;

    std::unordered_map<std::string, Eigen::VectorXd> output_encoders_;

    void setup(const std::string& robot_name, const std::string& laterality, const std::string& port_prefix, const bool& use_analogs, const bool& use_analogs_bounds = false, const yarp::sig::Matrix& analog_bounds = yarp::sig::Matrix(), const std::string& thumb_version = "", const bool& use_abduction = true);

};

#endif /* ROBOTSIO_ICUBHAND_H */
