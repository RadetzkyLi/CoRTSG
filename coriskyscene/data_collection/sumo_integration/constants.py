#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
""" This module defines constants used for the sumo-carla co-simulation. """

# ==================================================================================================
# -- constants -------------------------------------------------------------------------------------
# ==================================================================================================
import enum

INVALID_ACTOR_ID = -1
SPAWN_OFFSET_Z = 25.0  # meters

# only passenger car and evehicle are regarded as CAV
CAV_TYPE_ID_LIST = [
   "vehicle.audi.a2",
   "vehicle.mercedes.sprinter",
   "vehicle.chevrolet.impala",
   "vehicle.citroen.c3",
   "vehicle.tesla.model3",
   "vehicle.micro.microlino",
   "vehicle.mercedes.coupe_2020",
   "vehicle.dodge.charger_2020",
   "vehicle.lincoln.mkz_2020",
   "vehicle.mini.cooper_s_2021",
   "vehicle.toyota.prius",
   "vehicle.ford.mustang",
   "vehicle.nissan.patrol_2021",
   "vehicle.lincoln.mkz_2017",
   "vehicle.audi.etron",
   "vehicle.seat.leon",
   "vehicle.bmw.grandtourer",
   "vehicle.audi.tt",
   "vehicle.jeep.wrangler_rubicon",
   "vehicle.nissan.patrol",
   "vehicle.nissan.micra",
   "vehicle.mini.cooper_s",
   "vehicle.mercedes.coupe"
]

class ObjectCategory(enum.Enum):
    CAR = "car"
    VAN = "van"
    TRUCK = "truck"
    PED = "pedestrian"
    CYCLE = "cycle"
    MOTORCYCLE = "motorcycle"
    OTHER = "other"


TYPE_ID_TO_CATEGORY = {
    "vehicle.audi.a2": ObjectCategory.CAR,
    "vehicle.mercedes.sprinter": ObjectCategory.CAR,
    "vehicle.chevrolet.impala": ObjectCategory.CAR,
    "vehicle.citroen.c3": ObjectCategory.CAR,
    "vehicle.tesla.model3": ObjectCategory.CAR,
    "vehicle.dodge.charger_police_2020": ObjectCategory.CAR,
    "vehicle.micro.microlino": ObjectCategory.CAR,
    "vehicle.dodge.charger_police": ObjectCategory.CAR,
    "vehicle.mercedes.coupe_2020": ObjectCategory.CAR,
    "vehicle.harley-davidson.low_rider": ObjectCategory.MOTORCYCLE,
    "vehicle.dodge.charger_2020": ObjectCategory.CAR,
    "vehicle.ford.ambulance": ObjectCategory.VAN,
    "vehicle.lincoln.mkz_2020": ObjectCategory.CAR,
    "vehicle.mini.cooper_s_2021": ObjectCategory.CAR,
    "vehicle.toyota.prius": ObjectCategory.CAR,
    "vehicle.ford.mustang": ObjectCategory.CAR,
    "vehicle.volkswagen.t2": ObjectCategory.VAN,
    "vehicle.carlamotors.firetruck": ObjectCategory.TRUCK,
    "vehicle.carlamotors.carlacola": ObjectCategory.TRUCK,
    "vehicle.vespa.zx125": ObjectCategory.MOTORCYCLE,
    "vehicle.nissan.patrol_2021": ObjectCategory.CAR,
    "vehicle.lincoln.mkz_2017": ObjectCategory.CAR,
    "vehicle.tesla.cybertruck": ObjectCategory.CAR,
    "vehicle.audi.etron": ObjectCategory.CAR,
    "vehicle.seat.leon": ObjectCategory.CAR,
    "vehicle.diamondback.century": ObjectCategory.CYCLE,
    "vehicle.gazelle.omafiets": ObjectCategory.CYCLE,
    "vehicle.bmw.grandtourer": ObjectCategory.CAR,
    "vehicle.bh.crossbike": ObjectCategory.CYCLE,
    "vehicle.kawasaki.ninja": ObjectCategory.MOTORCYCLE,
    "vehicle.yamaha.yzf": ObjectCategory.MOTORCYCLE,
    "vehicle.audi.tt": ObjectCategory.CAR,
    "vehicle.jeep.wrangler_rubicon": ObjectCategory.CAR,
    "vehicle.nissan.patrol": ObjectCategory.CAR,
    "vehicle.nissan.micra": ObjectCategory.CAR,
    "vehicle.mini.cooper_s": ObjectCategory.CAR,
    "vehicle.mercedes.coupe": ObjectCategory.CAR,
    "walker.pedestrian": ObjectCategory.PED,
    "other": ObjectCategory.OTHER
}
TYPE_ID_TO_EXTENT = {
    "vehicle.audi.a2": [1.852684736251831, 0.8943392634391785, 0.7745251059532166],
    "vehicle.audi.etron": [2.427854299545288, 1.0163782835006714, 0.8246796727180481],
    "vehicle.audi.tt": [2.0906050205230713, 0.9970585703849792, 0.6926480531692505],
    "vehicle.bh.crossbike": [0.7436444163322449, 0.42962872982025146, 0.5397894978523254],
    "vehicle.bmw.grandtourer": [2.3055028915405273, 1.1208566427230835, 0.8336379528045654],
    "vehicle.carlamotors.carlacola": [2.601919174194336, 1.3134948015213013, 1.2337223291397095],
    "vehicle.carlamotors.firetruck": [4.234020709991455, 1.4455441236495972, 1.9137061834335327],
    "vehicle.chevrolet.impala": [2.6787397861480713, 1.0166014432907104, 0.7053293585777283],
    "vehicle.citroen.c3": [1.9938424825668335, 0.9254241585731506, 0.8085547685623169],
    "vehicle.diamondback.century": [0.8214218020439148, 0.18625812232494354, 0.5979812741279602],
    "vehicle.dodge.charger_2020": [2.5030298233032227, 1.0485419034957886, 0.7673624753952026],
    "vehicle.dodge.charger_police_2020": [2.6187572479248047, 1.0485419034957886, 0.819191575050354],
    "vehicle.dodge.charger_police": [2.487122058868408, 1.0192005634307861, 0.7710590958595276],
    "vehicle.ford.ambulance": [3.18282151222229, 1.1755871772766113, 1.215687870979309],
    "vehicle.ford.mustang": [2.358762502670288, 0.947413444519043, 0.650469958782196],
    "vehicle.gazelle.omafiets": [0.9177202582359314, 0.16446444392204285, 0.5856872797012329],
    "vehicle.harley-davidson.low_rider": [1.1778701543807983, 0.38183942437171936, 0.6382853388786316],
    "vehicle.jeep.wrangler_rubicon": [1.9331103563308716, 0.9525982737541199, 0.9389679431915283],
    "vehicle.kawasaki.ninja": [1.0166761875152588, 0.4012899398803711, 0.5727267861366272],
    "vehicle.lincoln.mkz_2020": [2.44619083404541, 1.115301489830017, 0.7400735020637512],
    "vehicle.lincoln.mkz_2017": [2.4508416652679443, 1.0641621351242065, 0.7553732395172119],
    "vehicle.mercedes.sprinter": [2.957595109939575, 0.9942164421081543, 1.2803276777267456],
    "vehicle.mercedes.coupe": [2.5133883953094482, 1.0757731199264526, 0.8253258466720581],
    "vehicle.mercedes.coupe_2020": [2.3368194103240967, 1.0011461973190308, 0.7209736704826355],
    "vehicle.micro.microlino": [1.1036475896835327, 0.7404598593711853, 0.6880123615264893],
    "vehicle.mini.cooper_s": [1.9029000997543335, 0.985137939453125, 0.7375151515007019],
    "vehicle.mini.cooper_s_2021": [2.2763495445251465, 1.0485360622406006, 0.8835831880569458],
    "vehicle.nissan.micra": [1.8166879415512085, 0.9225568771362305, 0.7506412863731384],
    "vehicle.nissan.patrol": [2.3022549152374268, 0.9657964706420898, 0.9274230599403381],
    "vehicle.nissan.patrol_2021": [2.782914400100708, 1.0749834775924683, 1.0225735902786255],
    "vehicle.seat.leon": [2.0964150428771973, 0.9080929160118103, 0.7369155883789062],
    "vehicle.tesla.model3": [2.3958897590637207, 1.081725001335144, 0.744159996509552],
    "vehicle.tesla.cybertruck": [3.1367764472961426, 1.1947870254516602, 1.049095630645752],
    "vehicle.toyota.prius": [2.256761312484741, 1.0034072399139404, 0.7624167203903198],
    "vehicle.vespa.zx125": [0.9023334980010986, 0.42784383893013, 0.6178141832351685],
    "vehicle.volkswagen.t2": [2.2402184009552, 1.034657597541809, 1.0188959836959839],
    "vehicle.yamaha.yzf": [1.1047229766845703, 0.43351709842681885, 0.6255727410316467], 
    "walker.pedestrian.0001": [0.18767888844013214, 0.18767888844013214, 0.9300000071525574],
    "walker.pedestrian.0002": [0.18767888844013214, 0.18767888844013214, 0.9300000071525574],
    "walker.pedestrian.0007": [0.18767888844013214, 0.18767888844013214, 0.9300000071525574],
    "walker.pedestrian.0010": [0.25, 0.25, 0.550000011920929],
    "walker.pedestrian.0015": [0.18767888844013214, 0.18767888844013214, 0.9300000071525574],
    "walker.pedestrian.0021": [0.18767888844013214, 0.18767888844013214, 0.9300000071525574],
    "walker.pedestrian.0029": [0.18767888844013214, 0.18767888844013214, 0.9300000071525574],
    "walker.pedestrian.0030": [0.18767888844013214, 0.18767888844013214, 0.9300000071525574],
    "walker.pedestrian.0036": [0.18767888844013214, 0.18767888844013214, 0.9300000071525574],
    "walker.pedestrian.0040": [0.18767888844013214, 0.18767888844013214, 0.9300000071525574]
}


class CustomActorType(enum.Enum):
    # used to differentiate the actor is vehicle or person
    # given its id.
    VEHICLE = 'vehicle'
    WALKER = 'walker'
    UNKNOWN = 'unknown'



# prefefined RSU (only traffic lights are RSU)
# similar landmark.id means the traffic lights lie in the same junction
# format: {"{town_name}": [landmark.id, ...]}
_tls_town01 = ["360", "383", "388", "379", "372", "365", "394", "369", "375", "392", "385", "368"]
_tls_town03 = ["2030", "2017", "2000", "2014", "2022", "1994", "2009", "2005", "2008", "2028", "2025"]
_tls_town05 = ["2385", "2413", "2379", "2389", "2409", "2429", "2395", "2404", "2383", "2419", "2430", "2400", "2407", "2434", "2433"]
_tls_town06 = ["1233", "1239", "1231", "1229", "1221", "1237", "1223", "1226"]
_tls_town07 = ["1033", "1009", "998", "1024", "981"]
_tls_town10 = ["961", "943", "951", "954", "957"]
RSU_LANDMARK_IDS = {
    "Town01": _tls_town01,
    "Carla/Maps/Town01": _tls_town01,
    "Town03": _tls_town03,
    "Carla/Maps/Town03": _tls_town03,
    "Town05": _tls_town05,
    "Carla/Maps/Town05": _tls_town05,
    "Town06": _tls_town06,
    "Carla/Maps/Town06": _tls_town06,
    "Town07": _tls_town07,
    "Carla/Maps/Town07": _tls_town07,
    "Town10HD": _tls_town10,
    "Carla/Maps/Town10HD": _tls_town10
}