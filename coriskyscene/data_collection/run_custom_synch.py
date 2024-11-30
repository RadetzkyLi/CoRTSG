#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Modified by: Rongsong Li <rongsong.li@qq.com>
"""
Script to integrate CARLA and SUMO simulations
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import time

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import os
import sys

# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci 

# ==================================================================================================
# -- sumo integration imports ----------------------------------------------------------------------
# ==================================================================================================

from coriskyscene.data_collection.sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from coriskyscene.data_collection.sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from coriskyscene.data_collection.sumo_integration.constants\
    import INVALID_ACTOR_ID,CAV_TYPE_ID_LIST,CustomActorType  # pylint: disable=wrong-import-position
from coriskyscene.data_collection.sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position

# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================

class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of sumo and carla
    simulations.
    """
    def __init__(self,
                 sumo_simulation,
                 carla_simulation,
                 tls_manager='none',
                 sync_vehicle_color=False,
                 sync_vehicle_lights=False,
                 gather_data=False):

        self.sumo = sumo_simulation
        self.carla = carla_simulation

        self.tls_manager = tls_manager
        self.sync_vehicle_color = sync_vehicle_color
        self.sync_vehicle_lights = sync_vehicle_lights

        self.gather_data = gather_data
        self.steps = 0  # elapsed steps 
        self.start_sync_steps = 0 
        self.num_of_cav = 0  # spawned cav
        self.max_num_of_cav = 1000

        if tls_manager == 'carla':
            self.sumo.switch_off_traffic_lights()
        elif tls_manager == 'sumo':
            self.carla.switch_off_traffic_lights()

        # Mapped actor ids.
        self.sumo2carla_ids = {}  # Contains only actors controlled by sumo.
        self.carla2sumo_ids = {}  # Contains only actors controlled by carla.

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        BridgeHelper.offset = self.sumo.get_net_offset()

    def tick(self):
        """
        Tick to simulation synchronization
        """
        # -----------------
        # sumo-->carla sync
        # -----------------
        self.sumo.tick()

        # Spawning new sumo actors in carla (i.e, not controlled by carla).
        if self.steps < self.start_sync_steps:
            return
        elif self.steps == self.start_sync_steps:
            sumo_spawned_actors = set(traci.vehicle.getIDList()) | set(traci.person.getIDList())
            logging.debug("[run_custom_synch] at steps {0}, spawned actors: {1}"
                          .format(self.steps, sumo_spawned_actors))
        else:
            if self.steps%10 == 0:
                logging.info("[run_custom_synch] at steps {0}, carla overview: {1}"
                             .format(self.steps, self.carla.get_current_overview()))
                logging.debug('[run_custom_synch] carla2sumo: {0}'.format(self.carla2sumo_ids))
            sumo_spawned_actors = self.sumo.spawned_actors - set(self.carla2sumo_ids.values())
        
        for sumo_actor_id in sumo_spawned_actors:
            self.sumo.subscribe(sumo_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)
            carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, self.sync_vehicle_color)
            if carla_blueprint is not None:
                carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                                   sumo_actor.extent)

                carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)
                if carla_actor_id != INVALID_ACTOR_ID:
                    self.sumo2carla_ids[sumo_actor_id] = carla_actor_id
                    # spawn sensors for cav
                    if self.gather_data and sumo_actor.type_id in CAV_TYPE_ID_LIST and \
                        self.num_of_cav < self.max_num_of_cav:
                        self.num_of_cav += 1
                        self.carla.spawn_sensors_for_agent(carla_actor_id, 'cav')
            else:
                self.sumo.unsubscribe(sumo_actor_id)

        # spawn sensors for rsu
        if self.gather_data:
            self.carla.spawn_sensors_for_agent(None, 'rsu')

        # Destroying sumo arrived actors in carla.
        for sumo_actor_id in self.sumo.destroyed_actors:
            if sumo_actor_id in self.sumo2carla_ids:
                self.carla.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))

        # Updating sumo actors in carla.
        carla_actor_speed = {}
        for sumo_actor_id in self.sumo2carla_ids:
            carla_actor_id = self.sumo2carla_ids[sumo_actor_id]

            sumo_actor = self.sumo.get_actor(sumo_actor_id)
            carla_actor = self.carla.get_actor(carla_actor_id)

            carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                               sumo_actor.extent)
            if self.sync_vehicle_lights and BridgeHelper.is_vehicle(carla_actor):
                carla_lights = BridgeHelper.get_carla_lights_state(carla_actor.get_light_state(),
                                                                   sumo_actor.signals)
            else:
                carla_lights = None

            carla_actor_speed[carla_actor_id] = sumo_actor.speed

            self.carla.synchronize_actor(carla_actor_id, carla_transform, carla_lights)
            
        self.carla.synchronize_velocity(carla_actor_speed)

        # Updates traffic lights in carla based on sumo information.
        if self.tls_manager == 'sumo':
            common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
            for landmark_id in common_landmarks:
                sumo_tl_state = self.sumo.get_traffic_light_state(landmark_id)
                carla_tl_state = BridgeHelper.get_carla_traffic_light_state(sumo_tl_state)

                self.carla.synchronize_traffic_light(landmark_id, carla_tl_state)

        # -----------------
        # carla-->sumo sync
        # -----------------
        self.carla.tick()

        # Spawning new carla actors (not controlled by sumo)
        carla_spawned_actors = self.carla.spawned_actors - set(self.sumo2carla_ids.values())
        for carla_actor_id in carla_spawned_actors:
            carla_actor = self.carla.get_actor(carla_actor_id)

            type_id = BridgeHelper.get_sumo_vtype(carla_actor)
            color = carla_actor.attributes.get('color', None) if self.sync_vehicle_color else None
            if type_id is not None:
                sumo_actor_id = self.sumo.spawn_actor(type_id, color)
                if sumo_actor_id != INVALID_ACTOR_ID:
                    self.carla2sumo_ids[carla_actor_id] = sumo_actor_id
                    self.sumo.subscribe(sumo_actor_id)

        # Destroying required carla actors in sumo.
        for carla_actor_id in self.carla.destroyed_actors:
            if carla_actor_id in self.carla2sumo_ids:
                carla_actor = self.carla.get_actor(carla_actor_id)
                self.sumo.destroy_actor(self.carla2sumo_ids.pop(carla_actor_id))

        # Updating carla actors in sumo.
        for carla_actor_id in self.carla2sumo_ids:
            sumo_actor_id = self.carla2sumo_ids[carla_actor_id]

            carla_actor = self.carla.get_actor(carla_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            sumo_transform = BridgeHelper.get_sumo_transform(carla_actor.get_transform(),
                                                             carla_actor.bounding_box.extent)
            if self.sync_vehicle_lights:
                carla_lights = self.carla.get_actor_light_state(carla_actor_id)
                if carla_lights is not None:
                    sumo_lights = BridgeHelper.get_sumo_lights_state(sumo_actor.signals,
                                                                     carla_lights)
                else:
                    sumo_lights = None
            else:
                sumo_lights = None

            sumo_speed = BridgeHelper.get_sumo_velocity(carla_actor.get_velocity())

            self.sumo.synchronize_actor(sumo_actor_id, sumo_transform, sumo_lights)

        # Updates traffic lights in sumo based on carla information.
        if self.tls_manager == 'carla':
            common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
            for landmark_id in common_landmarks:
                carla_tl_state = self.carla.get_traffic_light_state(landmark_id)
                sumo_tl_state = BridgeHelper.get_sumo_traffic_light_state(carla_tl_state)

                # Updates all the sumo links related to this landmark.
                self.sumo.synchronize_traffic_light(landmark_id, sumo_tl_state)

    def close(self):
        """
        Cleans synchronization.
        """
        # Configuring carla simulation in async mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)

        # Destroying synchronized actors.
        for carla_actor_id in self.sumo2carla_ids.values():
            self.carla.destroy_actor(carla_actor_id)

        for sumo_actor_id in self.carla2sumo_ids.values():
            self.sumo.destroy_actor(sumo_actor_id)

        # Closing sumo and carla client.
        self.carla.close()
        self.sumo.close()


def synchronization_loop(args):
    """
    Entry point for sumo-carla co-simulation.
    """
    extra_cfg = {
        "start_frame": args.start_frame,
        "end_frame": args.end_frame,
        "save_every": args.save_every
    }
    sumo_simulation = SumoSimulation(args.sumo_cfg_file, args.step_length, args.sumo_host,
                                     args.sumo_port, args.sumo_gui, args.client_order)
    carla_simulation = CarlaSimulation(args.carla_cfg_file, args.carla_host, args.carla_port, 
                                       args.step_length, extra_cfg)

    synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, args.tls_manager,
                                                args.sync_vehicle_color, args.sync_vehicle_lights,
                                                args.gather_data)
    synchronization.max_num_of_cav = args.max_cavs
    synchronization.start_sync_steps = args.start_sync_steps
    MAX_STEPS = args.max_steps

    try:
        while True:
            start = time.time()

            synchronization.tick()

            end = time.time()
            elapsed = end - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)

            synchronization.steps += 1
            if synchronization.steps > MAX_STEPS:
                break

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    except Exception as e:
        logging.error(e)

    finally:
        logging.info("Simulation stopped after {0} steps".format(synchronization.steps))
        logging.info('Cleaning synchronization')

        synchronization.close()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('sumo_cfg_file', type=str, help='sumo configuration file')
    argparser.add_argument('carla_cfg_file', type=str, help='carla configuration file')
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--sumo-host',
                           metavar='H',
                           default=None,
                           help='IP of the sumo host server (default: 127.0.0.1)')
    argparser.add_argument('--sumo-port',
                           metavar='P',
                           default=None,
                           type=int,
                           help='TCP port to listen to (default: 8813)')
    argparser.add_argument('--sumo-gui', action='store_true', help='run the gui version of sumo')
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--client-order',
                           metavar='TRACI_CLIENT_ORDER',
                           default=1,
                           type=int,
                           help='client order number for the co-simulation TraCI connection (default: 1)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'sumo', 'carla'],
                           help="select traffic light manager (default: sumo)",
                           default='sumo')
    argparser.add_argument('--start-sync-steps',
                           type=int,
                           help="To avoid useless and costly rendering, we can start the sync after the "
                           "traffic flows in SUMO have reached the state what we want. (default: 1200)",
                           default=120)
    argparser.add_argument('--max-steps',
                           type=int,
                           help="The co-simulation would be closed after maximum steps. (default: 3600)",
                           default=3600)
    # dumping config
    argparser.add_argument('--gather-data',
                           action='store_true',
                           help='If set all cars will be equipped with sensors and collecting data.'\
                            'Otherwise just run simulation and no data dumping. (default: False)')
    argparser.add_argument('--start-frame',
                           type=int,
                           help="the frame from which the sensing data will be dumped (default: 0)",
                           default=0)
    argparser.add_argument('--end-frame',
                           type=int,
                           help="the frame after which the sensing data won't be dumped (default: 600)",
                           default=600)
    argparser.add_argument('--save-every',
                           type=int,
                           help="the data will be dumped every ``save_every`` frames (default: 2)",
                           default=2)
    argparser.add_argument('--max-cavs',
                           type=int,
                           help="Too many CAVs collecting data bring huge burden for CARLA. One can "
                           "alliviate this by restricting the number of CAVs. (default: 100)",
                           default=100)
    # sync config
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.sync_vehicle_all is True:
        arguments.sync_vehicle_lights = True
        arguments.sync_vehicle_color = True

    # check arguments
    assert arguments.max_steps > 0 , "Expect: ``max_steps``>0"
    assert arguments.start_sync_steps > 0 , "Expect: ``start_sync_step``>0"
    assert arguments.start_frame >=0 , "``start_frame`` should be greater than 0!"
    assert arguments.end_frame > arguments.start_frame , "Expect: ``end_frame`` > ``start_frame``"
    assert arguments.save_every > 0 , "Expect: ``save_every``>0"
    assert arguments.max_cavs > 0 , "Expect: ``max_cavs``>0"


    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    synchronization_loop(arguments)
