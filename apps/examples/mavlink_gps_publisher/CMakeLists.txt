# ##############################################################################
# apps/examples/mavlink_gps_publisher/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

if(CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER)
  set(INCDIR
      ${CMAKE_CURRENT_LIST_DIR}/mavlink
      ${CMAKE_CURRENT_LIST_DIR}/mavlink/common
      ${CMAKE_CURRENT_LIST_DIR}/dronecan/generated
      ${NUTTX_APPS_DIR}/canutils/libdronecan/libcanard
      ${NUTTX_APPS_DIR}/canutils/libdronecan/libcanard/drivers/nuttx)

  nuttx_add_application(
    NAME
    ${CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_PROGNAME}
    PRIORITY
    ${CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_PRIORITY}
    STACKSIZE
    ${CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_STACKSIZE}
    MODULE
    ${CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER}
    SRCS
    mavlink_gps_publisher_main.c
    nav_state.c
    imu_state.c
    ahrs_state.c
    ahrs_filter.c
    baro_state.c
    mavlink_link.c
    dronecan_gnss.c
    dronecan/generated/uavcan.equipment.gnss.Fix2.c
    dronecan/generated/uavcan.equipment.gnss.Auxiliary.c
    dronecan/generated/uavcan.equipment.ahrs.MagneticFieldStrength.c
    dronecan/generated/uavcan.equipment.ahrs.MagneticFieldStrength2.c
    dronecan/generated/dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes.c
    dronecan/generated/uavcan.equipment.ahrs.RawIMU.c
    dronecan/generated/uavcan.equipment.air_data.StaticPressure.c
    dronecan/generated/uavcan.equipment.air_data.StaticTemperature.c
    dronecan/generated/uavcan.protocol.NodeStatus.c
    dronecan/generated/uavcan.protocol.debug.LogMessage.c
    dronecan/generated/uavcan.protocol.debug.LogLevel.c
    dronecan/generated/uavcan.protocol.dynamic_node_id.Allocation.c
    dronecan/generated/uavcan.Timestamp.c
    dronecan/generated/uavcan.equipment.gnss.ECEFPositionVelocity.c
    INCLUDE_DIRECTORIES
    ${INCDIR})
endif()
