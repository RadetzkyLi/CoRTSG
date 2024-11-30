# -*- coding: utf-8 -*-
"""
Sensor's mounting position.
"""
# Author: Rongsong Li <rongsong.li@qq.com>
# License: TDG-Attribution-NonCommercial-NoDistrib

_relative_x_2lane = -(3.5 + 2.0 + 1.13)     # used for traffic light lying on road with 2 lanes
_relative_x_3lane = -(3.5*1.5 + 2.0 + 1.13) # used for traffic light lying on road with 3 lanes
_relative_x_4lane = -(3.5*2.0 + 2.0 + 1.13)
_relative_x_5lane = -(3.5*2.5 + 2.0 + 3.13)
_relative_z = 4.2672    # meters
_pitch = -15            # degrees 
_yaw_forward = +90      # degrees
_yaw_backward = -90     # degrees


# vertical traffic light standing by road side
_default_transform_pole = {
    "x": 0.0,
    "y": 0.0,
    "z": _relative_z,
    "forward": {"yaw": -90.0, "pitch": _pitch},
    "backward": {"yaw": +120.0, "pitch": _pitch}
}
_default_transform_pole2 = {
    "x": 0.0,
    "y": 0.0,
    "z": _relative_z,
    "forward": {"yaw": -120.0, "pitch": _pitch},
    "backward": {"yaw": +120.0, "pitch": _pitch}
}

# hanging traffic light crossing the road
_default_transform_cross = {
    "x": _relative_x_4lane,
    "y": 0,
    "z": _relative_z,
    "forward": {"yaw": _yaw_forward, "pitch": _pitch},
    "backward": {"yaw": _yaw_backward, "pitch": _pitch}
}


# All traffic lights in Town01, Town03, Town05, Town06, Town07 and Town10HD
# can be regarded as Road Side Unit and the sensor's mounting positions
# have been hard coded in the following.
# All sensor's positions have been checked manually by visualizaiton.
_TLS_Town01 = {
        # T Junction "110"
        "369": {
            **_default_transform_pole2
        },
        "370": {
            **_default_transform_pole2
        },
        "371": {
            **_default_transform_pole
        },
        # T Junction "171"
        "375": {
            **_default_transform_pole
        },
        "376": {
            **_default_transform_pole2
        },
        "377": {
            **_default_transform_pole2
        },
        # T Junction "306"
        "390": {
            **_default_transform_pole
        },
        "391": {
            **_default_transform_pole2
        },
        "392": {
            **_default_transform_pole2
        },
        # T Junction "255"
        "384": {  # occluded by street light pole
            **_default_transform_pole2
        },
        "385": {
            **_default_transform_pole2
        },
        "386": {  # occluded by street light pole
            **_default_transform_pole2
        },
        # T Junction "87"
        "366": {
            **_default_transform_pole2
        },
        "367": {
            **_default_transform_pole
        },
        "368": {
            **_default_transform_pole2
        },
        # T Junction "54"
        "363": {
            **_default_transform_pole2
        },
        "364": {
            **_default_transform_pole
        },
        "365": {
            **_default_transform_pole2
        },
        # T Junction "332" 
        "393": {
            **_default_transform_pole2
        },
        "394": {
            **_default_transform_pole2
        },
        "395": {
            **_default_transform_pole,
        },
        # T Junction "26"
        "360": {
            **_default_transform_pole
        },
        "361": {
            **_default_transform_pole2
        },
        "362": {
            **_default_transform_pole2
        },
        # T Junction "222"
        "381": {
            **_default_transform_pole2
        },
        "382": {
            **_default_transform_pole2
        },
        "383": {
            **_default_transform_pole
        },
        # T Junction "278"
        "387": {
            **_default_transform_pole2
        },
        "388": {
            **_default_transform_pole2
        },
        "389": {
            **_default_transform_pole
        },
        # T Junction "194"
        "378": {
            **_default_transform_pole,
            "forward": {"yaw": -135, "pitch": _pitch}
        },
        "379": {
            **_default_transform_pole
        },
        "380": {
            **_default_transform_pole2
        },
        # T Junction "143"
        "372": {
            **_default_transform_pole2
        },
        "373": {
            **_default_transform_pole2
        },
        "374": {
            **_default_transform_pole
        }
}

_TLS_Town03 = {
        # X Junction "1820"
        "2029": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "2030": {
            **_default_transform_cross
        },
        "2031": {
            **_default_transform_cross
        },
        # 5-lag Junction "861"
        "2015": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2016": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "2017": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2018": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        # X Junction "238"
        "1998": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "1999": {
            **_default_transform_cross
        },
        "2000": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2001": {
            **_default_transform_cross
        },
        # T Junction "655"
        "2012": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2013": {
            **_default_transform_cross,
            "x": _relative_x_5lane,
            "backward": {"yaw": 25.0, "pitch": _pitch}
        },
        "2014": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        # X Junction "1221"
        "2019": {
            **_default_transform_cross
        },
        "2020": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2021": {
            **_default_transform_cross
        },
        "2022": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        # X Junction "103"
        "1994": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "1995": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "1996": {
            **_default_transform_cross
        },
        "1997": {
            **_default_transform_cross
        },
        # X Junction "576"
        "2009": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2010": {
            **_default_transform_cross
        },
        "2011": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        # X Junction "356"
        "2002": {
            **_default_transform_cross
        },
        "2003": {
            **_default_transform_cross
        },
        "2004": {
            **_default_transform_cross
        },
        "2005": {
            **_default_transform_cross
        },
        # X Junction "498"
        "2006": {
            **_default_transform_cross
        },
        "2007": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2008": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        # T Junction "1696"
        "2026": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "2027": {
            **_default_transform_cross
        },
        "2028": {
            **_default_transform_cross
        },
        # T Junction "1352"
        "2023": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2024": {
            **_default_transform_cross,
            "backward": {"yaw": 25.0, "pitch": _pitch}
        },
        "2025": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        }
}

_TLS_Town05 = {
        # X Junction "838"
        "2385": {
            **_default_transform_cross
        },
        "2386": {
            **_default_transform_cross
        },
        "2387": {
            **_default_transform_cross
        },
        "2388": {
            **_default_transform_cross
        },
        # T Junction "1882"
        "2413": {
            **_default_transform_cross
        },
        "2414": {
            **_default_transform_cross,
            "backward": {"yaw": 25.0, "pitch": _pitch}
        },
        "2415": {
            **_default_transform_cross
        },
        # X Junction "53"
        "2376": {
            **_default_transform_cross
        },
        "2377": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2378": {
            **_default_transform_cross
        },
        "2379": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        # X Junction "979"
        "2389": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2390": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2391": {
            **_default_transform_cross
        },
        "2392": {
            **_default_transform_cross
        },
        # X Junction "1722"
        "2409": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2410": {
            **_default_transform_cross
        },
        "2411": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2412": {
            **_default_transform_cross
        },
        # T Junction "2240"
        "2427": {
            **_default_transform_cross
        },
        "2428": {
            **_default_transform_cross,
            "x": _relative_x_5lane,
            "backward": {"yaw": 25.0, "pitch": _pitch}
        },
        "2429": {
            **_default_transform_cross
        },
        # X Junction "1126"
        "2393": {
            **_default_transform_cross,
            "backward": {"yaw": -110.0, "pitch": _pitch}
        },
        "2394": {
            **_default_transform_cross
        },
        "2395": {
            **_default_transform_cross
        },
        "2396": {
            **_default_transform_cross
        },
        # X Junction "1427"
        "2401": {
            **_default_transform_cross
        },
        "2402": {
            **_default_transform_cross
        },
        "2403": {
            **_default_transform_cross
        },
        "2404": {
            **_default_transform_cross
        },
        # X Junction "207"
        "2380": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2381": {
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        "2382": {
            **_default_transform_cross
        },
        "2383": {
            **_default_transform_cross
        },
        # T Junction "2014"
        "2419": {
            **_default_transform_cross
        },
        "2420": {
            **_default_transform_cross
        },
        "2421": {  # in junction center
            **_default_transform_cross,
            "backward": {"yaw": 0.0, "pitch": _pitch}
        },
        # T Junction "2296"
        "2430": {
            **_default_transform_cross,
            "backward": {"yaw": 25.0, "pitch": _pitch}
        },
        "2431": {
            **_default_transform_cross
        },
        # X Junction "1292"
        "2397": {
            **_default_transform_cross
        },
        "2398": {
            **_default_transform_cross
        },
        "2399": {
            **_default_transform_cross
        },
        "2400": {
            **_default_transform_cross
        },
        # X Junction "1574"
        "2405": {
            **_default_transform_cross
        },
        "2406": {
            **_default_transform_cross
        },
        "2407": {
            **_default_transform_cross
        },
        "2408": {
            **_default_transform_cross
        },
        # X Junction "2086"
        "2423": {
            **_default_transform_cross
        },
        "2424": {
            **_default_transform_cross
        },
        "2425": {
            **_default_transform_cross
        },
        "2426": {
            **_default_transform_cross
        },
        # T Junction "2328"
        "2432": {
            **_default_transform_cross,
            "x": _relative_x_4lane
        },
        "2433": {
            **_default_transform_cross,
            "backward": {"yaw": 0.0, "pitch": _pitch}
        },
        "2434": {
            **_default_transform_cross
        }
    }

_TLS_Town06 = {
        # T Junction "763"
        "1233": {
            **_default_transform_cross
        },
        "1234": {
            **_default_transform_pole,
            "forward": {"yaw": +90.0, "pitch": _pitch},
            "backward": {"yaw": +160.0, "pitch": _pitch}
        },
        # T Junction "1162"
        "1238": {
            **_default_transform_cross
        },
        "1239": {
            **_default_transform_pole,
            "forward": {"yaw": +60.0, "pitch": _pitch},
            "backward": {"yaw": +160.0, "pitch": _pitch}
        },
        # X Junction "582"
        "1230": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "1231": { # in junction center
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "1232": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        # X Junction "452"
        "1227": {
            **_default_transform_cross
        },
        "1228": {
            **_default_transform_cross,
            "x": _relative_x_3lane
        },
        "1229": {  # in junction center
            **_default_transform_cross,
            "x": _relative_x_3lane
        },
        # T Junction "72"
        "1221": {
            **_default_transform_cross,
            "x": _relative_x_3lane
        },
        "1222": {
            **_default_transform_cross,
            "x": _relative_x_3lane
        },
        # X Junction "838"
        "1235": {
            **_default_transform_cross,
            "x": _relative_x_3lane
        },
        "1236": {
            **_default_transform_cross,
            "x": _relative_x_3lane
        },
        "1237": {  # in junction center
            **_default_transform_cross,
            "x": _relative_x_5lane
        },
        # T Junction "268"
        "1223": {
            **_default_transform_cross,
            "backward": {"yaw": 25.0, "pitch": _pitch}
        },
        "1224": {
            **_default_transform_cross,
            "backward": {"yaw": -120.0, "pitch": _pitch}
        },
        # T Junction "332"
        "1225": {
            **_default_transform_cross,
            "backward": {"yaw": 25.0, "pitch": _pitch}
        },
        "1226": {
            **_default_transform_cross
        }
    }

_TLS_Town07 = {
        # T Junction "918"
        "1032": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "1033": { # near junction center
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "1034": {
            **_default_transform_cross,
            "x": _relative_x_2lane,
            "backward": {"yaw": +30, "pitch": _pitch}
        },
        # T Junction "466"
        "1007": {
            **_default_transform_cross,
            "x": _relative_x_2lane,
            "backward": {"yaw": -130, "pitch": _pitch}
        },
        "1008": {
            **_default_transform_cross,
            "x": _relative_x_2lane,
            "backward": {"yaw": 15.0, "pitch": _pitch}
        },
        "1009": {  # in junction center
            **_default_transform_cross,
            "x": _relative_x_2lane,
            "backward": {"yaw": -60.0, "pitch": _pitch}
        },
        # T Junction "355"
        "998": {  # near junction center
            **_default_transform_cross,
            "x": _relative_x_2lane,
            "backward": {"yaw": -45.0, "pitch": _pitch}
        },
        "999": {  
            **_default_transform_cross,
            "x": _relative_x_2lane,
            "backward": {"yaw": 25.0, "pitch": _pitch}
        },
        "1000": {
            **_default_transform_cross,
            "x": _relative_x_2lane,
            "backward": {"yaw": -120.0, "pitch": _pitch},
            "forward": {"yaw": 75.0, "pitch": _pitch}
        },
        # X Junction "725"
        "1022": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "1023": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "1024": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "1025": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        # X Junction "68"
        "979": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "980": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "981": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "982": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        }
    }

_TLS_Town10HD = {
        # T Junction "719"
        "960": {
            **_default_transform_cross,
            "x": _relative_x_2lane,
            "backward": {"yaw": 25.0, "pitch": _pitch}
        },
        "961": { # in junction center
            **_default_transform_cross
        },
        "962": { # in junction center
            **_default_transform_cross
        },
        # T Junction "23"
        "943": { # in junction center
            **_default_transform_cross
        },
        "944": {
            **_default_transform_cross
        },
        "945": {
            **_default_transform_cross
        },
        # X Junction "189"
        "949": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        "950": { # in junction center
            **_default_transform_cross
        },
        "951": { # in junction center
            **_default_transform_cross
        },
        "952": {
            **_default_transform_cross,
            "x": _relative_x_2lane
        },
        # T Junction "468"
        "953": {
            **_default_transform_cross,
            "backward": {"yaw": 25.0, "pitch": _pitch}
        },
        "954": {
            **_default_transform_cross
        },
        # X Junction "532"
        "957": { # in junction center
            **_default_transform_cross
        },
        "958": { # in junction center
            **_default_transform_cross
        },
        "959": {
            **_default_transform_cross,
            "x": _relative_x_2lane,
            "backward": {"yaw": 45.0, "pitch": _pitch}
        }
}

RSU_RELATIVE_TRANSFORM = {
    "Town01": _TLS_Town01,
    "Town01_Opt": _TLS_Town01,
    "Town03": _TLS_Town03,
    "Tonw03_Opt": _TLS_Town03,
    "Town05": _TLS_Town05,
    "Town05_Opt": _TLS_Town05,
    "Town06": _TLS_Town06,
    "Town06_Opt": _TLS_Town06,
    "Town07": _TLS_Town07,
    "Town07_Opt": _TLS_Town07,
    "Town10HD": _TLS_Town10HD,
    "Town10HD_Opt": _TLS_Town10HD
}


def _unit_test():
    # check info integrity
    set_1 = set(["x", "y", "z", "forward", "backward"])
    set_2 = set(["yaw", "pitch"])
    print("Start checking...")
    for town_name, info in RSU_RELATIVE_TRANSFORM.items():
        print("{0} has {1} traffic lights in total".format(town_name, len(info)))
        for landmark_id,v in info.items():
            keys = set(list(v.keys()))
            if keys == set_1 and set(list(v["forward"].keys())) == set_2 \
                and set((list(v["backward"].keys()))) == set_2:
                continue
            print("In {0}, traffic light {1} miss params!".format(town_name, landmark_id))

if __name__ == '__main__':
    _unit_test()