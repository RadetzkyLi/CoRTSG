# Introduction
This README provides details about collecting multi-modal sensing data in [OPV2V](https://mobility-lab.seas.ucla.edu/opv2v/) format
 for cooperative perception using SUMO-CARLA co-simulation,
in which [SUMO](https://sumo.dlr.de/docs/index.html0) is in charge of traffic controlling and 
[CARLA](https://carla.org/) in charge of sensor simulation. 
The basic idea is that at each timestep: 
1. **SUMO updation**: SUMO updates states of traffic participants (vehicles, walkers, etc.) and traffic signals by its built-in powerful models.
2. **State synchronization**: transform states in SUMO simulator into CARLA simulator. States includes vehicle/walker's location, speed, vehicle light etc., 
traffic signal's state and so on.
3. **CARLA updataion**: call ``tick()`` to update physical world of CARLA simulator and then dump sensors' data.


# Detailed Steps
To create a customized dataset, follow these steps:

## Route planning
To plan route for vehicles and walkers, you can leverage Python tools provided by SUMO:
```
cd ${SUMO_HOME}
python tools/randomTrips.py -n ${net_file_path} -o ${output_trip_path} --end ${end_time}\
--period ${period} --min_distance ${min_distance}
```
in which:
- ``net_file_path``: the map in SUMO's format on which you want to generate vehicles.
Usually ends with ``.net.xml``. All maps in CARLA had been converted, modified and saved in ``coriskyscene/data_collection/data/net/*.net.xml``. You may need to add crossings manually using ``netedit`` in SUMO. 
- ``output_trip_path``: the path the results will be saved, ending with ``.rou.xml``.
- ``end_time``: vehicle's departure end time in seconds, default to ``3600``.
- ``period``:  the vehicle will depart every ``period`` seconds.
- ``min_distance``: the minimum distance in meters a vehicle will dirve.

For more usages, refer to ``${SUMO_HOME}/tools/randomTrips.py``.

## Config
You need to prepare the following two configuration files:
- **SUMO config**: write SUMO's simulation config file (ends with ``.sumocfg``) as that in ``coriskyscene/data/data_collection/data/*.sumocfg`` or official's [tutorial](https://sumo.dlr.de/docs/Tutorials/quick_start.html). This config file controls network, route, simulation time and so on for SUMO. 
- **CARLA config**: write CARLA's simulation config file (``.yaml``) as ``coriskyscene/data/data_collection/data_protocal.yaml``. This config file controls sensor's configuration, data dumping path and simulation settings for CARLA.


## Running

To collect data at a map, e.g., Town01, you need to:

#### launch CARLA
Open a terminal and launch CARLA in server mode:
```
export CUDA_VISIBLE_DEVICES=0   # specify the GPU you want to use, keep the followng three number same
export SDL_HINT_CUDA_DEVICE=0
${CARLA_HOME}/CarlaUE4.sh -carla-server -RenderOffScreen -graphicsadapter=0
```
in which ``${CARLA_HOME}`` is your CARLA's installation directory.

#### start co-simulation
Using exsiting net and route file, you can:
```
cd CoRiskyScene/coriskyscene/data_collection
python run_custom_synch.py ./data/Town01.sumocfg data_protocal.yaml --tls-manager sumo --sync-vehicle-all --sumo-gui --debug --gather-data
```
See more usages in ``run_custom_synch.py``.

By default, a CAV is equipped with 1 RGB cameras, 1 GNSS, 1 LiDAR and 1 semantic LiDAR. Depending on your setting, there may be tens even hundreds of CAVs in a map. Don't be worry if the co-simulation was stuck. Just wait because at each timestep there may be hundreds of sensors needing rendering.

# Requirements
The following softwares are needed:
```
sumo>=1.18.0
CARLA==0.9.12
python==3.7
```
It's recommended to create a virtual environment for running CARLA, 
e.g., ``conda create -n python37 python=3.7; conda activate python37``.