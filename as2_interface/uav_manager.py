# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__authors__ = 'Rafael Pérez Seguí'

"""
uav_interface.py
"""

import threading
import json
from math import tan ,radians
from time import sleep
from typing import Callable
from rclpy.qos import qos_profile_system_default, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from as2_msgs.msg import MissionUpdate
from as2_python_api.mission_interpreter.mission import MissionItem, Mission, InterpreterStatus
from as2_python_api.drone_interface_gps import DroneInterfaceBase
from as2_python_api.behavior_manager.behavior_manager import DroneBehaviorManager
from as2_python_api.modules.land_module import LandModule
from as2_python_api.modules.takeoff_module import TakeoffModule
from as2_python_api.modules.go_to_module import GoToModule
from as2_python_api.modules.follow_path_module import FollowPathModule
from as2_python_api.modules.gps_module import GpsModule
from as2_python_api.modules.go_to_gps_module import GoToGpsModule
from as2_python_api.modules.follow_path_gps_module import FollowPathGpsModule
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from as2_msgs.msg import YawMode


VIRTUAL_MODE = False 
GPS_COORDINATES = [40.4405, -3.68982, 0.0]
YAW_ANGLE = radians(0.0) # 135.0º
GIMBAL_ANGLE = 0.0


class UavInterface(DroneInterfaceBase):
    """ UAV Interface """
    info_lock = threading.Lock()

    def __init__(self, drone_id: str, verbose: bool = False, sim_mode: bool = False,
                 use_sim_time: bool = False, use_cartesian_coordinates: bool = False):

        self.verbose = verbose
        self.use_cartesian_coordinates = use_cartesian_coordinates

        if not VIRTUAL_MODE:
            DroneInterfaceBase.__init__(self,
                                        drone_id=drone_id,
                                        verbose=verbose,
                                        use_sim_time=use_sim_time)
            if not self.use_cartesian_coordinates:
                self.gps = GpsModule(drone=self)

        else:
            self.drone_id_aux = drone_id

        self._sim_mode = sim_mode
        self._yaw_mode = YawMode()
        self._yaw_mode.mode = YawMode.KEEP_YAW
        self._yaw_mode.angle = YAW_ANGLE

        # ROS 2 Mission interpreter
        if not VIRTUAL_MODE:
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )

            self.mission_update_pub = self.create_publisher(
                MissionUpdate, 'mission_update', qos_profile_system_default)

            self.mission_status_sub = self.create_subscription(
                String, 'mission_status', self.mission_status_callback,
                qos_profile)

        self.missions = {}
        self.mission_status = "IDLE"

    def info_lock_decor(func: Callable) -> Callable:
        """ Decorator for info lock """

        def wrapper(self, *args, **kwargs):
            with self.info_lock:
                return func(self, *args, **kwargs)
        return wrapper

    @info_lock_decor
    def get_info(self):
        """ Get info """
        info = {
            'id': self.drone_id,
            'state': self.info,
        }
        if self.use_cartesian_coordinates:
            info['pose'] = [
                self.position[0],
                self.position[1]
            ]
        else:
            info['pose'] = [
                self.gps.pose[0],
                self.gps.pose[1]
            ]
        info['pose'].append(self.position[2])
        info['pose'].append(self.orientation[2])
        return info

    def start_mission(self, mission_id: int) -> None:
        """ Start UAV mission """
        mission_update = MissionUpdate()
        mission_update.drone_id = self.namespace
        mission_update.mission_id = mission_id
        mission_update.action = MissionUpdate.START

        # Publish the mission
        self.mission_update_pub.publish(mission_update)

    def __virtual_mission_status_change(self):
        """ Virtual mission status change """
        result = {}
        for behavior in self.modules:
            if isinstance(self.modules[behavior], BehaviorHandler):
                result[str(behavior)] = True
        return result

    def pause_mission(self) -> None:
        """ Pause UAV mission """
        if VIRTUAL_MODE:
            return self.__virtual_mission_status_change()
        mission_update = MissionUpdate()
        mission_update.drone_id = self.namespace
        mission_update.action = MissionUpdate.PAUSE

        # Publish the mission
        self.mission_update_pub.publish(mission_update)

    def resume_mission(self) -> None:
        """ Resume UAV mission """
        if VIRTUAL_MODE:
            return self.__virtual_mission_status_change()
        mission_update = MissionUpdate()
        mission_update.drone_id = self.namespace
        mission_update.action = MissionUpdate.RESUME

        # Publish the mission
        self.mission_update_pub.publish(mission_update)

    def stop_mission(self) -> None:
        """ Stop UAV mission """
        if VIRTUAL_MODE:
            return self.__virtual_mission_status_change()

        mission = Mission(target=self.namespace, verbose=True)
        mission.plan.append(MissionItem(behavior='rtl', args={
            'height': 15.0,
            'speed': 3.0,
            'land_speed': 1.0,
            'wait': True
        }))

        mission_update = MissionUpdate()
        mission_update.drone_id = self.namespace
        mission_update.action = MissionUpdate.LOAD
        mission_update.mission_id = 10
        mission_update.mission = mission.json()

        # Publish the mission
        self.mission_update_pub.publish(mission_update)

    def load_mission(self, mission_id: int, mission_list: list) -> None:
        """ Load mission """

        # Mission
        mission = Mission(target=self.namespace, verbose=True)

        for element in mission_list:
            speed = float(element['speed'])

            if element['name'] == 'TakeOffPoint':
                mission.plan.append(MissionItem(behavior='takeoff', args={
                    'height': element['values'][0][2], 'speed': speed, 'wait': True
                }))

                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                mission.plan.append(MissionItem(behavior='go_to_gps', args={
                    'lat': waypoint[0], 'lon': waypoint[1], 'alt': waypoint[2],
                    'speed': speed, 'yaw_mode': self._yaw_mode.mode,
                    'yaw_angle': self._yaw_mode.angle, 'wait': True
                }))
                # Enable Gimbal reset
                # if GIMBAL_ANGLE != 0.0:
                #     x = 1.0
                #     z = x * tan(radians(GIMBAL_ANGLE))
                #     mission.plan.append(MissionItem(
                #         behavior='point_gimbal',
                #         args={
                #             '_x': x, '_y': 0.0, '_z': z, 'frame_id': f"{self.namespace}/base_link",
                #             'wait': True
                #     }))

            elif element['name'] == 'LandPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                mission.plan.append(MissionItem(behavior='go_to_gps', args={
                    'lat': waypoint[0], 'lon': waypoint[1], 'alt': waypoint[2],
                    'speed': speed, 'yaw_mode': self._yaw_mode.mode,
                    'yaw_angle': self._yaw_mode.angle, 'wait': True
                }))
                # Enable Gimbal reset
                # if GIMBAL_ANGLE != 0.0:
                #     mission.plan.append(MissionItem(
                #         behavior='point_gimbal',
                #         args={
                #             '_x': 1.0, '_y': 0.0, '_z': 0.0,
                #             'frame_id': f"{self.namespace}/base_link",
                #             'wait': True
                #     }))
                mission.plan.append(MissionItem(
                    behavior='land', args={'speed': speed}))

            elif element['name'] == 'Path':
                waypoints = element['values']
                for waypoint in waypoints:
                    mission.plan.append(MissionItem(behavior='go_to_gps', args={
                        'lat': waypoint[0], 'lon': waypoint[1], 'alt': waypoint[2],
                        'speed': speed, 'yaw_mode': self._yaw_mode.mode,
                        'yaw_angle': self._yaw_mode.angle, 'wait': True
                    }))

            elif element['name'] == 'WayPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                mission.plan.append(MissionItem(behavior='go_to_gps', args={
                    'lat': waypoint[0], 'lon': waypoint[1], 'alt': waypoint[2],
                    'speed': speed, 'yaw_mode': self._yaw_mode.mode,
                    'yaw_angle': self._yaw_mode.angle, 'wait': True
                }))

            elif element['name'] == 'Area':
                waypoints = element['values']
                mission.plan.append(MissionItem(behavior='follow_path_gps', args={
                    'geopath': waypoints, 'speed': speed, 'yaw_mode': self._yaw_mode.mode,
                    'yaw_angle': self._yaw_mode.angle, 'wait': True
                }))
            else:
                raise Exception(
                    "Unknown mission element name: ", element['name'])

        mission_update = MissionUpdate()
        mission_update.drone_id = self.namespace
        mission_update.mission_id = mission_id
        mission_update.mission = mission.json()
        mission_update.action = MissionUpdate.LOAD

        # Publish the mission
        self.mission_update_pub.publish(mission_update)
        self.mission_status = "LOAD"

    def mission_status_callback(self, msg: String) -> None:
        """ Mission status callback """
        if self.verbose:
            print(f"[UavInterface] Mission status: {msg.data}")
        dict_data = json.loads(msg.data)
        # Check if dict has the key 'id' and 'status'
        if 'id' in dict_data and 'status' in dict_data:
            self.missions[dict_data['id']] = dict_data['status']
