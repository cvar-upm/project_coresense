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
mission_manager.py
"""

import copy
import math
import numpy as np
from swarm_pylib.swarm_pylib import Swarm


def _densify_path(waypoints: list, max_dist: float) -> list:
    """Insert intermediate points so no consecutive pair is farther than max_dist apart."""
    result = []
    for i, p in enumerate(waypoints):
        result.append(p)
        if i < len(waypoints) - 1:
            q = waypoints[i + 1]
            dist = math.sqrt(sum((b - a) ** 2 for a, b in zip(p, q)))
            n_segments = math.ceil(dist / max_dist)
            for j in range(1, n_segments):
                t = j / n_segments
                result.append([a + t * (b - a) for a, b in zip(p, q)])
    return result


class MissionInterpreter():
    """ Mission Interpreter """
    use_cartesian_coordinates = False

    @staticmethod
    def interpreter(msg: dict) -> tuple:
        """ Mission interpreter """
        if msg['payload']['status'] != 'request':
            message_info = (f"Invalid mission status: {msg['payload']['status']},"
                            " only 'request' is allowed for now")
            print(f"[MissionInterpreter] {message_info}")

        if msg['payload']['id'] != 'New Mission':
            message_info = (f"Invalid mission id: {msg['payload']['id']},"
                            " only 'New Mission' is allowed for now")
            print(f"[MissionInterpreter] {message_info}")
            return False, message_info

        if len(msg['payload']['uavList']) == 0:
            message_info = "No UAVs selected in mission"
            print(f"[MissionInterpreter] {message_info}")
            return False, message_info

        if len(msg['payload']['layers']) == 0:
            message_info = "No layers selected in mission"
            print(f"[MissionInterpreter] {message_info}")
            return False, message_info

        return True, str("")

    @staticmethod
    def planner(mission_id: str, mission_info: dict, use_cartesian_coordinates: bool = False,
                initial_positions: dict = None, max_wp_distance: float = None,
                filter_collinear: bool = True) -> dict:
        """Convert Aerostack UI mission to ROS mission

        Args:
            mission_id (number): Mission id
            uavList (list): List of UAVs
            layers (list): List of layers
            initial_positions (dict): Current position [x, y, z] keyed by drone name;
                used as the starting point for coverage planning when TakeOffPoint
                layers are absent from the mission spec.
            max_wp_distance (float): If set, intermediate waypoints are inserted along
                each Area path segment so that no two consecutive points are farther
                apart than this distance (metres). None disables densification.
            filter_collinear (bool): If True, collinear waypoints along each street pass
                are removed (default). Set False (via mission field filter_collinear: false)
                to keep all wpSpace-generated intermediate points.
        """

        send_mission = {
            'id': mission_id,
            'uavList': mission_info['uavList'],
            'layers': []
        }
        first_uav = mission_info['uavList'][0]

        mission = {}
        last_position = {}
        for uav in mission_info['uavList']:
            mission[str(uav)] = []
            if initial_positions and str(uav) in initial_positions:
                last_position[str(uav)] = list(initial_positions[str(uav)])
            else:
                last_position[str(uav)] = [None, None, None]
        for layer in mission_info['layers']:
            send_layer, mission, new_last_position = MissionInterpreter.layer_interpreter(
                layer, mission, last_position, first_uav, mission_info, use_cartesian_coordinates,
                max_wp_distance, filter_collinear)

            for uav in new_last_position:
                last_position[uav] = new_last_position[uav]

            send_mission['layers'].append(send_layer)

        return send_mission, mission

    @staticmethod
    def layer_interpreter(layer: dict, mission: dict, last_position: dict,
                          first_uav: str, mission_info: dict, use_cartesian_coordinates: bool = False,
                          max_wp_distance: float = None, filter_collinear: bool = True) -> tuple:
        """ Layer interpreter """

        name = layer['name']
        uav_list = layer['uavList']
        height = float(layer['height'])
        values = layer['values']
        speed = float(layer['speed'])
        send_layer = {
            'name': name,
            'uavList': uav_list,
            'height': height,
            'speed': speed,
            'values': values
        }

        new_last_position = {}

        if name in ('TakeOffPoint', 'LandPoint', 'WayPoint', 'Path'):

            uav = uav_list[0]
            if uav == 'auto':
                uav = first_uav

            waypoints = []
            if name == 'Path':
                for point in layer['values']:
                    waypoints.append(
                        [point[0], point[1], height])
                new_last_position[uav] = waypoints[len(waypoints)-1]
            else:
                waypoints = [[values[0], values[1], height]]
                new_last_position[uav] = waypoints[0]

            mission[uav].append({
                'name': name,
                'speed': speed,
                'values': waypoints
            })
            send_layer['uavList'] = [uav]

        elif name == 'Area':

            if uav_list[0] == 'auto':
                uav_list_aux = mission_info['uavList']
            else:
                uav_list_aux = uav_list

            index = mission_info['layers'].index(layer)
            range_to_end = range(index+1, len(mission_info['layers']))
            sublist = [mission_info['layers'][i] for i in range_to_end]

            algorithm = layer['algorithm']
            street_spacing = layer['streetSpacing']
            wp_space = layer['wpSpace']
            theta = float(layer['orientation'])
            filter_collinear = bool(layer.get('filterCollinear', layer.get('filter_collinear', filter_collinear)))

            if theta < 0 or theta > 360:
                theta = None

            next_position = MissionInterpreter.get_next_position(
                sublist, mission, last_position, first_uav, mission_info, use_cartesian_coordinates,
                max_wp_distance, filter_collinear)

            # area_path = MissionInterpreter.swarm_planning(
            #     uav_list_aux, last_position, next_position, height,
            #     values, str(algorithm), float(
            #         street_spacing), float(wp_space),
            #     theta)

            uavs_state = {}
            for uav in uav_list_aux:
                uavs_state[uav] = {
                    'initial_position': last_position[uav],
                    'last_position': next_position[uav]}

            area = []
            for point in values:
                area.append([point[0], point[1], height])

            if use_cartesian_coordinates:
                uavs_path = Swarm.swarm_planning(
                    uavs_state,
                    area,
                    str(algorithm),
                    'binpat',
                    float(street_spacing),
                    float(wp_space),
                    theta,
                    filter_path_waypoints=filter_collinear)
            else:
                uavs_path = Swarm.swarm_planning_gps(
                    uavs_state,
                    area,
                    str(algorithm),
                    'binpat',
                    float(street_spacing),
                    float(wp_space),
                    theta,
                    filter_path_waypoints=filter_collinear)

            area_path = {}
            for uav_path, uav in zip(uavs_path, uav_list_aux):
                path = uav_path
                if max_wp_distance is not None:
                    path = _densify_path(path, max_wp_distance)
                area_path[uav] = path

            send_layer['uavPath'] = {}
            for uav in uav_list_aux:
                new_last_position[uav] = next_position[uav]
                mission[uav].append({
                    'name': name,
                    'speed': speed,
                    'values': area_path[uav]
                })
            send_layer['uavPath'] = area_path
            send_layer['uavList'] = uav_list_aux

        else:
            raise Exception("Unknown layer name")

        return send_layer, mission, new_last_position

    @staticmethod
    def get_next_position(sublist: list, mission: list, last_position: list,
                          first_uav: str, mission_info: dict, use_cartesian_coordinates: bool = True,
                          max_wp_distance: float = None, filter_collinear: bool = True) -> dict:
        """ Get next position """
        mission_aux = copy.deepcopy(mission)
        last_position_aux = copy.deepcopy(last_position)

        last_position_list = {}
        last_position_flag = {}
        for uav in mission_info['uavList']:
            last_position_list[str(uav)] = [None, None, None]
            last_position_flag[str(uav)] = False

        for layer in sublist:

            send_layer, mission, last_position = MissionInterpreter.layer_interpreter(
                layer, mission_aux, last_position_aux, first_uav, mission_info,
                use_cartesian_coordinates,
                max_wp_distance=max_wp_distance, filter_collinear=filter_collinear)

            for uav in last_position:
                if last_position_flag[uav] == False:
                    last_position_list[uav] = last_position[uav]
                    last_position_flag[uav] = True

        for uav in last_position_flag:
            if not last_position_flag[uav]:
                raise Exception("Next position for UAV not found")

        return last_position_list
