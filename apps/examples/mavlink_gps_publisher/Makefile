############################################################################
# apps/examples/mavlink_gps_publisher/Makefile
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(APPDIR)/Make.defs

# MAVLink GPS/IMU publisher built-in application info

PROGNAME  = $(CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_PROGNAME)
PRIORITY  = $(CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_STACKSIZE)
MODULE    = $(CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER)

CFLAGS += ${INCDIR_PREFIX}$(APPDIR)/examples/mavlink_gps_publisher/mavlink
CFLAGS += ${INCDIR_PREFIX}$(APPDIR)/examples/mavlink_gps_publisher/mavlink/common
CFLAGS += ${INCDIR_PREFIX}$(APPDIR)/examples/mavlink_gps_publisher/dronecan/generated
CFLAGS += ${INCDIR_PREFIX}$(APPDIR)/canutils/libdronecan/libcanard
CFLAGS += ${INCDIR_PREFIX}$(APPDIR)/canutils/libdronecan/libcanard/drivers/nuttx

CSRCS  = nav_state.c imu_state.c ahrs_state.c ahrs_filter.c gps_nmea.c mavlink_link.c mpu9250.c
CSRCS += dronecan_gnss.c
CSRCS += dronecan/generated/uavcan.equipment.gnss.Fix2.c
CSRCS += dronecan/generated/uavcan.equipment.ahrs.MagneticFieldStrength2.c
CSRCS += dronecan/generated/uavcan.equipment.ahrs.RawIMU.c
CSRCS += dronecan/generated/uavcan.Timestamp.c
CSRCS += dronecan/generated/uavcan.equipment.gnss.ECEFPositionVelocity.c
MAINSRC = mavlink_gps_publisher_main.c

include $(APPDIR)/Application.mk
