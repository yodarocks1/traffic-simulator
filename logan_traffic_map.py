import typing
import numpy as np
import math
from traffic_map import TrafficMap, Direction, TrafficGenerator, ControlledIntersection, LightIntersection, SignIntersection, RoundaboutIntersection, ACTION_PHASE_SETS

######################
#      |  SB  |      #
# _____|      |_____ #
#                    #
# EB              WB #
# _____        _____ #
#      |      |      #
#      |  NB  |      #
######################

__all__ = ["LoganTrafficMap"]

# SOURCE: https://stackoverflow.com/a/56769419/8210988
def haversine(lat1, lon1, lat2, lon2):
    """
    This uses the ‘haversine’ formula to calculate the great-circle distance between two points – that is, 
    the shortest distance over the earth’s surface – giving an ‘as-the-crow-flies’ distance between the points 
    (ignoring any hills they fly over, of course!).
    Haversine
    formula:    a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
    c = 2 ⋅ atan2( √a, √(1−a) )
    d = R ⋅ c
    where   φ is latitude, λ is longitude, R is earth’s radius (mean radius = 3,958.8mi);
    note that angles need to be in radians to pass to trig functions!

    Returns distance in miles
    """
    R = 3958.8
    lat1,lon1,lat2,lon2 = map(np.radians, [lat1,lon1,lat2,lon2])

    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2) **2
    c = 2 * np.arctan2(a**0.5, (1-a)**0.5)
    d = R * c
    return round(d,4)

def coordinate_str_to_float(s: str) -> float:
    degrees, s = s.split("°", 1)
    minutes, s = s.split("'", 1)
    seconds, s = s.split("\"", 1)
    direction = s
    degrees = float(degrees) + float(minutes)/60 + float(seconds)/3600
    if direction in ["S", "W"]:
        degrees = -degrees
    return degrees

def nodes_from_dict(node_data: dict[str, dict[str, dict[str, typing.Union[type, tuple, list, str]]]], speeds: dict[str, float], default_speed = 25):
    nodes: dict[str, dict[str, ControlledIntersection]] = {}
    coordinates: dict[str, dict[str, tuple[float, float]]] = {}
    for road_a in node_data:
        nodes[road_a] = {}
        coordinates[road_a] = {}

    for road_a in node_data:
        for road_b in node_data[road_a]:
            if type(node_data[road_a][road_b]["type"]) is not type or not issubclass(node_data[road_a][road_b]["type"], ControlledIntersection):
                raise ValueError(f"Intersection data 'type' key must be a subclass of ControlledIntersection (got {node_data[road_a][road_b]['type']})")
            elif type(node_data[road_a][road_b]["args"]) not in [tuple, list]:
                raise ValueError(f"Intersection data 'type' key must be a tuple or a list (got {type(node_data[road_a][road_b]['args'])})")
            nodes[road_a][road_b] = node_data[road_a][road_b]["type"](road_a, road_b, *node_data[road_a][road_b]["args"])
            coordinates[road_a][road_b] = tuple(map(coordinate_str_to_float, (node_data[road_a][road_b]["lat"], node_data[road_a][road_b]["lon"])))

    for road_a in node_data:
        for road_b in node_data[road_a]:
            node = nodes[road_a][road_b]
            data = node_data[road_a][road_b]
            coords_this = coordinates[road_a][road_b]
            for k, direction in [("NB", Direction.NB), ("EB", Direction.EB), ("SB", Direction.SB), ("WB", Direction.WB)]:
                if data[k] is None:
                    pass
                elif isinstance(data[k], TrafficGenerator):
                    node.add_generator(direction, data[k])
                else:
                    other_node = nodes[data[k][0]][data[k][1]]
                    coords_other = coordinates[data[k][0]][data[k][1]]
                    dist = haversine(*coords_this, *coords_other) # yields in miles
                    if data[k][0] == road_a and road_a in speeds:
                        speed = speeds[road_a]
                    elif data[k][1] == road_b and road_b in speeds:
                        speed = speeds[road_b]
                    else:
                        speed = default_speed
                    node.add_edge(other_node, dist / (speed / 60), direction=direction)

    return nodes, coordinates


class GeneratorMajor(TrafficGenerator):
    def __init__(self, rate=1):
        super().__init__(rate*10)
class GeneratorMinor(TrafficGenerator):
    def __init__(self, rate=1):
        super().__init__(rate*0.5)
class LoganTrafficMap(TrafficMap):
    INTERSECTION_DATA = {
        "Main St": {
            "300 N": {"type": SignIntersection, "args": (500, "NS"),
                "NB": ("Main St", "400 N"),
                "EB": ("100 E", "300 N"),
                "SB": GeneratorMajor(6),
                "WB": GeneratorMajor(),
                "lat": "41°44'13.7\"N", "lon": "111°50'05.4\"W",
            },
            "400 N": {"type": LightIntersection, "args": (10, ACTION_PHASE_SETS["offset-lefts-a"]),
                "NB": ("Main St", "500 N"),
                "EB": ("100 E", "400 N"),
                "SB": ("Main St", "300 N"),
                "WB": GeneratorMajor(),
                "lat": "41°44'20.6\"N", "lon": "111°50'05.1\"W",
            },
            "500 N": {"type": LightIntersection, "args": (2, ACTION_PHASE_SETS["straights-first"]),
                "NB": ("Main St", "600 N (A)"),
                "EB": ("100 E", "500 N"),
                "SB": ("Main St", "400 N"),
                "WB": GeneratorMajor(),
                "lat": "41°44'27.4\"N", "lon": "111°50'05.0\"W",
            },
            "600 N (A)": {"type": SignIntersection, "args": (2, "NS"),
                "NB": ("Main St", "600 N"),
                "EB": None,
                "SB": ("Main St", "500 N"),
                "WB": GeneratorMajor(),
                "lat": "41°44'33.4\"N", "lon": "111°50'04.7\"W",
            },
            "600 N": {"type": SignIntersection, "args": (2, "NS"),
                "NB": ("Main St", "700 N"),
                "EB": ("100 E", "600 N"),
                "SB": ("Main St", "600 N (A)"),
                "WB": None,
                "lat": "41°44'34.5\"N", "lon": "111°50'04.8\"W",
            },
            "700 N": {"type": LightIntersection, "args": (2, ACTION_PHASE_SETS["straights-first"]),
                "NB": ("Main St", "800 N"),
                "EB": ("100 E", "700 N"),
                "SB": ("Main St", "600 N"),
                "WB": GeneratorMajor(),
                "lat": "41°44'41.1\"N", "lon": "111°50'04.6\"W",
            },
            "800 N": {"type": SignIntersection, "args": (2, "NS"),
                "NB": ("Main St", "900 N"),
                "EB": ("100 E", "800 N"),
                "SB": ("Main St", "700 N"),
                "WB": GeneratorMajor(),
                "lat": "41°44'47.0\"N", "lon": "111°50'04.5\"W",
            },
            "900 N": {"type": SignIntersection, "args": (2, "NS"),
                "NB": ("Main St", "1000 N"),
                "EB": GeneratorMinor(),
                "SB": ("Main St", "800 N"),
                "WB": GeneratorMajor(),
                "lat": "41°44'53.7\"N", "lon": "111°50'04.3\"W",
            },
            "1000 N": {"type": LightIntersection, "args": (500, ACTION_PHASE_SETS["straights-first"]),
                "NB": GeneratorMajor(5),
                "EB": ("50 E", "1000 N"),
                "SB": ("Main St", "900 N"),
                "WB": GeneratorMajor(),
                "lat": "41°45'00.5\"N", "lon": "111°50'04.2\"W",
            },
        },
        "50 E": {
            "1000 N": {"type": SignIntersection, "args": (2, "EW"),
                "NB": GeneratorMinor(),
                "EB": ("200 E", "1000 N"),
                "SB": None,
                "WB": ("Main St", "1000 N"),
                "lat": "41°45'00.5\"N", "lon": "111°49'58.5\"W",
            },
        },
        "100 E": {
            "300 N": {"type": SignIntersection, "args": (1, "NS"),
                "NB": ("100 E", "400 N"),
                "EB": ("200 E", "300 N"),
                "SB": GeneratorMajor(),
                "WB": ("Main St", "300 N"),
                "lat": "41°44'13.5\"N", "lon": "111°49'55.9\"W",
            },
            "400 N": {"type": LightIntersection, "args": (1, ACTION_PHASE_SETS["straights-first"]),
                "NB": ("100 E", "500 N"),
                "EB": ("200 E", "400 N"),
                "SB": ("100 E", "300 N"),
                "WB": ("Main St", "400 N"),
                "lat": "41°44'20.5\"N", "lon": "111°49'55.8\"W",
            },
            "500 N": {"type": SignIntersection, "args": (1, False),
                "NB": ("100 E", "600 N"),
                "EB": ("200 E", "500 N"),
                "SB": ("100 E", "400 N"),
                "WB": ("Main St", "500 N"),
                "lat": "41°44'27.2\"N", "lon": "111°49'55.4\"W",
            },
            "600 N": {"type": SignIntersection, "args": (1, "NS"),
                "NB": ("100 E", "700 N"),
                "EB": ("200 E", "600 N"),
                "SB": ("100 E", "500 N"),
                "WB": ("Main St", "600 N"),
                "lat": "41°44'34.3\"N", "lon": "111°49'55.1\"W",
            },
            "700 N": {"type": SignIntersection, "args": (1, "EW"),
                "NB": ("100 E", "800 N"),
                "EB": ("200 E", "700 N"),
                "SB": ("100 E", "600 N"),
                "WB": ("Main St", "700 N"),
                "lat": "41°44'40.9\"N", "lon": "111°49'55.0\"W",
            },
            "800 N": {"type": SignIntersection, "args": (1, "EW"),
                "NB": None,
                "EB": ("200 E", "800 N (A)"),
                "SB": ("100 E", "700 N"),
                "WB": ("Main St", "800 N"),
                "lat": "41°44'46.7\"N", "lon": "111°49'54.8\"W",
            },
        },
        "200 E": {
            "300 N": {"type": SignIntersection, "args": (12, "NS"),
                "NB": ("200 E", "400 N"),
                "EB": ("300 E", "300 N"),
                "SB": GeneratorMajor(),
                "WB": ("100 E", "300 N"),
                "lat": "41°44'13.4\"N", "lon": "111°49'46.6\"W",
            },
            "400 N": {"type": LightIntersection, "args": (1, ACTION_PHASE_SETS["straights-first"]),
                "NB": ("200 E", "500 N"),
                "EB": ("300 E", "400 N"),
                "SB": ("200 E", "300 N"),
                "WB": ("100 E", "400 N"),
                "lat": "41°44'20.3\"N", "lon": "111°49'46.3\"W",
            },
            "500 N": {"type": RoundaboutIntersection, "args": (1,),
                "NB": ("200 E", "600 N"),
                "EB": ("Brookside Pl", "500 N"),
                "SB": ("200 E", "400 N"),
                "WB": ("100 E", "500 N"),
                "lat": "41°44'27.2\"N", "lon": "111°49'46.3\"W",
            },
            "600 N": {"type": SignIntersection, "args": (1, "NS"),
                "NB": ("200 E", "Chestnut Ln"),
                "EB": ("300 E", "600 N"),
                "SB": ("200 E", "500 N"),
                "WB": ("100 E", "600 N"),
                "lat": "41°44'34.2\"N ", "lon": "111°49'46.1\"W",
            },
            "Chestnut Ln": {"type": SignIntersection, "args": (1, "NS"),
                "NB": ("200 E", "700 N"),
                "EB": GeneratorMinor(),
                "SB": ("200 E", "600 N"),
                "WB": None,
                "lat": "41°44'37.6\"N", "lon": "111°49'46.1\"W",
            },
            "700 N": {"type": SignIntersection, "args": (1, "NS"),
                "NB": ("200 E", "800 N (A)"),
                "EB": ("250 E", "700 N"),
                "SB": ("200 E", "Chestnut Ln"),
                "WB": ("100 E", "700 N"),
                "lat": "41°44'40.9\"N", "lon": "111°49'46.1\"W",
            },
            "800 N (A)": {"type": SignIntersection, "args": (0, "NS"),
                "NB": ("200 E", "800 N (B)"),
                "EB": None,
                "SB": ("200 E", "700 N"),
                "WB": ("100 E", "800 N"),
                "lat": "41°44'46.7\"N", "lon": "111°49'45.9\"W",
            },
            "800 N (B)": {"type": SignIntersection, "args": (0, "NS"),
                "NB": ("200 E", "870 N"),
                "EB": ("300 E", "800 N"),
                "SB": ("200 E", "800 N (A)"),
                "WB": None,
                "lat": "41°44'48.0\"N", "lon": "111°49'45.9\"W",
            },
            "870 N": {"type": SignIntersection, "args": (3, "NS"),
                "NB": ("200 E", "900 N"),
                "EB": GeneratorMinor(),
                "SB": ("200 E", "800 N (B)"),
                "WB": GeneratorMinor(),
                "lat": "41°44'51.4\"N", "lon": "111°49'45.8\"W",
            },
            "900 N": {"type": SignIntersection, "args": (1, "NS"),
                "NB": ("200 E", "970 N"),
                "EB": ("300 E", "900 N"),
                "SB": ("200 E", "870 N"),
                "WB": None,
                "lat": "41°44'54.6\"N", "lon": "111°49'45.7\"W",
            },
            "970 N": {"type": SignIntersection, "args": (3, "NS"),
                "NB": ("200 E", "1000 N"),
                "EB": GeneratorMinor(),
                "SB": ("200 E", "900 N"),
                "WB": None,
                "lat": "41°44'58.2\"N", "lon": "111°49'45.6\"W",
            },
            "1000 N": {"type": LightIntersection, "args": (4, ACTION_PHASE_SETS["offset-lefts-a"]),
                "NB": GeneratorMajor(),
                "EB": ("300 E", "1000 N"),
                "SB": ("200 E", "970 N"),
                "WB": ("50 E", "1000 N"),
                "lat": "41°45'00.4\"N", "lon": "111°49'45.6\"W",
            },
        },
        "250 E": {
            "700 N": {"type": SignIntersection, "args": (3, "EW"),
                "NB": GeneratorMinor(),
                "EB": ("N Heritage Cv", "700 N"),
                "SB": None,
                "WB": ("200 E", "700 N"),
                "lat": "41°44'40.8\"N", "lon": "111°49'43.1\"W",
            },
        },
        "N Heritage Cv": {
            "700 N": {"type": SignIntersection, "args": (3, "EW"),
                "NB": GeneratorMinor(),
                "EB": ("300 E", "700 N"),
                "SB": None,
                "WB": ("250 E", "700 N"),
                "lat": "41°44'40.8\"N", "lon": "111°49'41.7\"W",
            },
        },
        "Brookside Pl": {
            "500 N": {"type": SignIntersection, "args": (3, "EW"),
                "NB": GeneratorMinor(),
                "EB": ("300 E", "500 N"),
                "SB": None,
                "WB": ("200 E", "500 N"),
                "lat": "41°44'27.1\"N", "lon": "111°49'40.7\"W",
            },
        },
        "300 E": {
            "300 N": {"type": SignIntersection, "args": (2, "NS"),
                "NB": ("300 E", "330 N"),
                "EB": GeneratorMajor(),
                "SB": GeneratorMajor(),
                "WB": ("200 E", "300 N"),
                "lat": "41°44'13.2\"N", "lon": "111°49'37.4\"W",
            },
            "330 N": {"type": SignIntersection, "args": (2, "NS"),
                "NB": ("300 E", "400 N"),
                "EB": GeneratorMinor(),
                "SB": ("300 E", "300 N"),
                "WB": None,
                "lat": "41°44'15.8\"N", "lon": "111°49'37.2\"W",
            },
            "400 N": {"type": SignIntersection, "args": (200, "EW"),
                "NB": ("300 E", "500 N"),
                "EB": GeneratorMajor(3),
                "SB": ("300 E", "330 N"),
                "WB": ("200 E", "400 N"),
                "lat": "41°44'20.1\"N", "lon": "111°49'37.1\"W",
            },
            "500 N": {"type": SignIntersection, "args": (2, "EW"),
                "NB": ("300 E", "600 N"),
                "EB": GeneratorMajor(),
                "SB": ("300 E", "400 N"),
                "WB": ("Brookside Pl", "500 N"),
                "lat": "41°44'27.0\"N", "lon": "111°49'36.9\"W",
            },
            "600 N": {"type": SignIntersection, "args": (2, "EW"),
                "NB": ("300 E", "700 N"),
                "EB": GeneratorMajor(),
                "SB": ("300 E", "500 N"),
                "WB": ("200 E", "600 N"),
                "lat": "41°44'33.9\"N", "lon": "111°49'36.6\"W",
            },
            "700 N": {"type": SignIntersection, "args": (2, "EW"),
                "NB": ("300 E", "770 N"),
                "EB": GeneratorMajor(),
                "SB": ("300 E", "600 N"),
                "WB": ("N Heritage Cv", "700 N"),
                "lat": "41°44'40.8\"N", "lon": "111°49'36.4\"W",
            },
            "770 N": {"type": SignIntersection, "args": (2, "NS"),
                "NB": ("300 E", "800 N"),
                "EB": GeneratorMinor(),
                "SB": ("300 E", "700 N"),
                "WB": None,
                "lat": "41°44'45.1\"N", "lon": "111°49'36.1\"W",
            },
            "800 N": {"type": SignIntersection, "args": (2, "NS"),
                "NB": ("300 E", "900 N"),
                "EB": GeneratorMajor(),
                "SB": ("300 E", "770 N"),
                "WB": ("200 E", "800 N (B)"),
                "lat": "41°44'47.7\"N", "lon": "111°49'35.9\"W",
            },
            "900 N": {"type": SignIntersection, "args": (2, "NS"),
                "NB": ("300 E", "1000 N"),
                "EB": GeneratorMajor(),
                "SB": ("300 E", "800 N"),
                "WB": ("200 E", "900 N"),
                "lat": "41°44'54.5\"N", "lon": "111°49'35.7\"W",
            },
            "1000 N": {"type": SignIntersection, "args": (50, "EW"),
                "NB": None,
                "EB": GeneratorMajor(),
                "SB": ("300 E", "900 N"),
                "WB": ("200 E", "1000 N"),
                "lat": "41°45'00.3\"N", "lon": "111°49'35.5\"W",
            },
        },
    }
    SPEED_DATA = {
        "Main St": 35,
        "400 N": 40,
        "1000 N": 30,
        "_": 25,
    }
    def __init__(self):
        node_dict, coord_dict = nodes_from_dict(self.INTERSECTION_DATA, self.SPEED_DATA, default_speed=self.SPEED_DATA["_"])
        node_list = []
        for road_a in node_dict:
            for road_b in node_dict[road_a]:
                node_list.append(node_dict[road_a][road_b])

        edge_list = []
        generator_list = []
        for node in node_list:
            edge_list.extend(node.get_edges())
            generator_list.extend(node.get_generators())
        
        super().__init__(node_list, edge_list, generator_list, 50) # units = distance in minutes
        self.coord_dict = coord_dict

    def __repr__(self):
        return f"<LoganTrafficMap nodes=<{len(self.node_list)} nodes> edges=<{len(self.edge_list)} edges> generators=<{len(self.generator_list)} generators>>"

    def __str__(self):
        return f"Logan, UT"

    def get_data(self):
        data = self.coord_dict
        roads_list: list[tuple[str, str]] = []
        for road_a in data:
            for road_b in data[road_a]:
                roads_list.append((road_a, road_b))
        latitudes: list[float] = []
        longitudes: list[float] = []
        for roads in roads_list:
            lat, lon = data[roads[0]][roads[1]]
            latitudes.append(lat)
            longitudes.append(lon)
        text = list(map(lambda r: f"{r[0]}, {r[1]}", roads_list))
        self.get_data = lambda: (text, latitudes, longitudes)
        return text, latitudes, longitudes

    EDGE_OFFSET = 0.00001
    def get_edge_data(self):
        data = self.coord_dict
        edge_list: list[tuple[tuple[str, str], tuple[str, str], float, Direction]] = map(lambda edge: (edge[0].roads, edge[1].roads, edge[2]['weight'], edge[2]['direction']), self.edge_list)
        texts: list[str] = []
        latitudes: list[tuple[float, float]] = []
        longitudes: list[tuple[float, float]] = []
        intersections: list[tuple[str, str, str]] = []
        for edge in edge_list:
            lat0, lon0 = data[edge[0][0]][edge[0][1]]
            lat1, lon1 = data[edge[1][0]][edge[1][1]]
            if edge[3] is Direction.NB:
                lon0 += 1.2*self.EDGE_OFFSET
                lon1 += 1.2*self.EDGE_OFFSET
            elif edge[3] is Direction.SB:
                lon0 -= 1.2*self.EDGE_OFFSET
                lon1 -= 1.2*self.EDGE_OFFSET
            elif edge[3] is Direction.EB:
                lat0 -= self.EDGE_OFFSET
                lat1 -= self.EDGE_OFFSET
            elif edge[3] is Direction.WB:
                lat0 += self.EDGE_OFFSET
                lat1 += self.EDGE_OFFSET
            texts.append(edge[2])
            latitudes.append((lat0, lat1))
            longitudes.append((lon0, lon1))
            if edge[0][0] == edge[1][0]:
                intersections.append((edge[0][0], edge[0][1], edge[1][1]))
            elif edge[0][1] == edge[1][1]:
                intersections.append((edge[0][1], edge[0][0], edge[1][0]))
            elif edge[1][0].startswith(edge[0][0]):
                intersections.append((edge[0][0], edge[0][1], edge[1][1]))
            elif edge[0][0].startswith(edge[1][0]):
                intersections.append((edge[1][0], edge[0][1], edge[1][1]))
            elif edge[1][1].startswith(edge[0][1]):
                intersections.append((edge[0][1], edge[0][0], edge[1][0]))
            elif edge[0][1].startswith(edge[1][1]):
                intersections.append((edge[1][1], edge[0][0], edge[1][0]))
            else:
                raise ValueError(f"Invalid edge: ({edge[0][0]}, {edge[0][1]}), ({edge[1][0]}, {edge[1][1]})")
        self.get_edge_data = lambda: (texts, latitudes, longitudes, intersections)
        return texts, latitudes, longitudes, intersections

    GENERATOR_TAB_LENGTH = 0.0002
    def get_generator_data(self):
        data = self.coord_dict
        generator_list: list[tuple[tuple[str, str], float, Direction]] = map(lambda gen: (gen[0].roads, gen[1].rate, gen[2]), self.generator_list)
        texts: list[str] = []
        latitudes: list[tuple[float, float]] = []
        longitudes: list[tuple[float, float]] = []
        for generator in generator_list:
            lat0, lon0 = data[generator[0][0]][generator[0][1]]
            length = self.GENERATOR_TAB_LENGTH * math.log2(generator[1] + 1)
            if generator[2] is Direction.NB:
                lat1, lon1 = lat0 + length, lon0
            elif generator[2] is Direction.SB:
                lat1, lon1 = lat0 - length, lon0
            elif generator[2] is Direction.EB:
                lat1, lon1 = lat0, lon0 + 1.2*length
            elif generator[2] is Direction.WB:
                lat1, lon1 = lat0, lon0 - 1.2*length
            texts.append(generator[1])
            latitudes.append((lat0, lat1))
            longitudes.append((lon0, lon1))
        self.get_generator_data = lambda: (texts, latitudes, longitudes)
        return texts, latitudes, longitudes
    
    def texts(self):
        return self.get_data()[0]
    def latitudes(self):
        return self.get_data()[1]
    def longitudes(self):
        return self.get_data()[2]

    def edge_texts(self):
        return self.get_edge_data()[0]
    def edge_latitudes(self):
        return self.get_edge_data()[1]
    def edge_longitudes(self):
        return self.get_edge_data()[2]
    def edge_intersections(self):
        return self.get_edge_data()[3]

    def generator_texts(self):
        return self.get_generator_data()[0]
    def generator_latitudes(self):
        return self.get_generator_data()[1]
    def generator_longitudes(self):
        return self.get_generator_data()[2]


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    #import networkx as nx
    map_ = LoganTrafficMap()
    pos = {
        node: tuple(map(lambda x: 2*x, reversed(map_.coord_dict[node.roads[0]][node.roads[1]])))
        for node in map_
    }
    map_.draw(pos, with_labels=True)
    plt.show()