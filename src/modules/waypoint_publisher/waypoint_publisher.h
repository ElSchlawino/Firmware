/**
 * iFR GNC - simulink - PX4 interface
 * Institut fuer Flugmechanik und Flugregelung
 * Copyright 2018 Universitaet Stuttgart.
 *
 * @author Lorenz Schmitt <lorenz.schmitt@ifr.uni-stuttgart.de>  */

#ifndef WAYPOINT_PUBLISHER_H
#define WAYPOINT_PUBLISHER_H
#pragma once

#include <utility>
#include <vector>
#include <px4_module.h>
#include <px4_posix.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/waypoint_list.h>
#include <uORB/topics/mission.h>



struct WaypointPublisher: ModuleBase<WaypointPublisher> {
    WaypointPublisher();
    ~WaypointPublisher() override;
    /** @see ModuleBase::run() */
    void run() override;
    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    /** @see ModuleBase */
    static WaypointPublisher *instantiate(int argc, char *argv[]);
    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);
    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

private:
    double getHiResTime();
    double mLaunchTime;

    void connectParameters();
    void copyTopics();
    void publishTopics();

    int mActuatorArmed;
    int mSensorCombined;
    int mSensorBaro;
    int mVehicleMagnetometer;
    int mVehicleGpsPosition;
    int mOpticalFlow;
	int mMission;
    int mRcChannels;
    int mAirspeed;
    int mNoseboomAnglesIfr;
    int mInputRc;
    orb_advert_t mControllerStatusIfr;
	orb_advert_t mWaypointList;
	waypoint_list_s mWaypointListData;
    bool mParametersAreInitialized;
    int mParameterUpdate;
    std::vector<std::pair<param_t, float*>> mParameters;
    std::vector<px4_pollfd_struct_t> mPollingList;
    
    static int mShowRuntime;
};

#endif
