# Introduction
``coriskyscene.opendriveparser`` is modified from this GitHub [repo](https://github.com/liuyf5231/opendriveparser).
It aims to parse a [OpenDRIVE](https://www.asam.net/index.php?eID=dumpFile&t=f&f=3768&token=66f6524fbfcdb16cfb89aae7b6ad6c82cfc2c7f2) 
map into a Python class. 

For risky traffic scene generation, the core lies in the calculation of **waypoint**.
A waypoint is a 3D-directed point correpsonding to an OpenDRIVE lane.
Each waypoint contains attrubites which states its location on the map
and the orientation of the lane containing it. The variables ``road_id``,
``section_id``, ``lane_id`` and ``s`` correspond to the OpenDRIVE road.
For more details, refer to docs of CARLA in [here](https://carla.readthedocs.io/en/latest/core_map/#waypoints).