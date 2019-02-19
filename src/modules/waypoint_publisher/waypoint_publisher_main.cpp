/**
 * iFR GNC - simulink - PX4 interface
 * Institut fuer Flugmechanik und Flugregelung
 * Copyright 2018 Universitaet Stuttgart.
 *
 * @author Lorenz Schmitt <lorenz.schmitt@ifr.uni-stuttgart.de>  */

#include "waypoint_publisher.h"

extern "C" __EXPORT int waypoint_publisher_main(int argc, char *argv[]);

int waypoint_publisher_main(int argc, char *argv[]) {
    return WaypointPublisher::main(argc, argv);
}