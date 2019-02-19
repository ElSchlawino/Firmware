/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file load_mon.cpp
 *
 * @author Jonathan Challinger <jonathan@3drobotics.com>
 * @author Julian Oes <julian@oes.ch
 * @author Andreas Antener <andreas@uaventure.com>
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <systemlib/cpuload.h>
#include <perf/perf_counter.h>

#include <px4_config.h>
#include <px4_module.h>
#include <px4_workqueue.h>
#include <px4_defines.h>

#include <drivers/drv_hrt.h>

#include "../mavlink/mavlink_mission.h" //
#include "../mavlink/mavlink_main.h" //
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/waypoint_list.h>



#include <uORB/uORB.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/task_stack_info.h>

#include "waypoint_publisher.h"



#define STACK_LOW_WARNING_THRESHOLD 300 ///< if free stack space falls below this, print a warning
#define FDS_LOW_WARNING_THRESHOLD 3 ///< if free file descriptors fall below this, print a warning


int WaypointPublisher::mShowRuntime = 0;

WaypointPublisher::WaypointPublisher():
        mLaunchTime(getHiResTime()),
		mMission(orb_subscribe(ORB_ID(mission))),
		mWaypointList(orb_advertise(ORB_ID(waypoint_list), &mWaypointListData)),
		mPollingList(1){
		mPollingList[0].fd=mMission;
		for (auto & entry: mPollingList) entry.events=POLLIN;
		}


WaypointPublisher::~WaypointPublisher() {
    orb_unsubscribe(mMission);
    // terminate topic members
}

void WaypointPublisher::run() {
    double stepSize(/*step size*/);
    int divider = 1;
	double tdiff_mean = 0;
	double t_init(getHiResTime());
	

    while (!should_exit()) 
    {
		int pollingResult(px4_poll(mPollingList.data(), mPollingList.size(), 100));

		int dm_lock_ret = dm_lock(DM_KEY_WAYPOINTS_OFFBOARD_0);
		
		if (mPollingList[0].revents & POLLIN) {
		    		/* obtained data for the first file descriptor */
		    		struct mission_s raw;
		    		/* copy sensors raw data into local buffer */
		    		orb_copy(ORB_ID(mission), mMission, &raw);
		
		
			for(int i=0; i<100; ++i) {
				mission_item_s mission_item;
				dm_read(DM_KEY_WAYPOINTS_OFFBOARD_0, i, &mission_item, sizeof(mission_item_s));
				mWaypointListData.lat[i] = (float)mission_item.lat;
				mWaypointListData.lon[i] = (float)mission_item.lon;
				mWaypointListData.alt[i] = (float)mission_item.altitude;
			}
			mWaypointListData.count = raw.count;
			mWaypointListData.timestamp =  hrt_absolute_time();
			orb_publish(ORB_ID(waypoint_list), mWaypointList, &mWaypointListData);
		}
		//mission_save_point_s mission_start;
		//mission_s
		//mission_stats_entry_s mission_stats;
		//dm_read(DM_KEY_SAFE_POINTS, 0, &mission_start, sizeof(mission_save_point_s));
		
		/* unlock MISSION_STATE item */
		if (dm_lock_ret == 0) {
			dm_unlock(DM_KEY_WAYPOINTS_OFFBOARD_0);
		}
    }
}

int WaypointPublisher::task_spawn(int argc, char *argv[]) {
    _task_id=px4_task_spawn_cmd("simulink_interface_elflean", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 8000, (px4_main_t)&run_trampoline, (char *const *)argv);
    
    if (_task_id<0) {
        _task_id=-1;
        return -errno;
    }
    else return 0;
}

WaypointPublisher * WaypointPublisher::instantiate(int argc, char *argv[]) {
    return new WaypointPublisher();
}

int WaypointPublisher::print_usage(const char *reason) {
    if (reason) PX4_WARN("%s\n", reason);
    PRINT_MODULE_DESCRIPTION("simulink_interface_elflean runs simulink generated GNC code.");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "start module thread");
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "stop module thread");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "return module thread state");
    PRINT_MODULE_USAGE_COMMAND_DESCR("runtime", "display time needed to execute model");
    PRINT_MODULE_USAGE_NAME("simulink_interface_elflean", "system");
    return 0;
}

int WaypointPublisher::custom_command(int argc, char *argv[]) {
	for(int i=0; i<argc; ++i) {
		if (strcmp(argv[i], "runtime") == 0) {
			mShowRuntime ^= 1;
			printf("display of runtime is %s\n", mShowRuntime ? "on" : "off");
		} else {
			return print_usage("unknown command");
		}
	}
	return 0;
}

double WaypointPublisher::getHiResTime() {
    hrt_abstime hrt;
    hrt = hrt_absolute_time();
    return double(hrt * 1e-6);
}
