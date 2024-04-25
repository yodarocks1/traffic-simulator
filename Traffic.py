from traffic_map import Action, ControlledIntersection, Direction, TrafficMap
import typing
import random
import plotly.graph_objects as go
import numpy as np

# TRAFFIC GENERATORS
class TrafficGenerator:
    def __init__(self, map_):
        self.map = map_
        self.dests, self.weights = map_.get_destinations_and_weights()
    def pick_node(self, *exclude, nodes=None, weights=None):
        if nodes is None:
            nodes = self.dests
        if weights is None:
            weights = self.weights
        if len(nodes) != len(weights):
            raise ValueError(f"Length of the nodes array ({len(nodes)}) did not match the length of the weights array ({len(weights)})")
        choice = random.choices(range(len(nodes)), weights=weights, k=1)[0]
        if self.dests[choice] in exclude:
            w = list(weights)
            w[choice] = 0
            return self.pick_destination(*exclude, weights=w)
        return self.dests[choice]
    def generate(self, start_node: ControlledIntersection, direction: typing.Optional[Direction]) -> tuple[Direction, ControlledIntersection, ControlledIntersection]:
        if direction is None:
            direction = self.generate_direction()
        end_node = self.pick_node(start_node)
        return direction, start_node, end_node
    def generate_any(self) -> tuple[Direction, ControlledIntersection, ControlledIntersection]:
        raise NotImplementedError()
    def generate_direction(self) -> Direction:
        raise NotImplementedError()
class NaiveTrafficGenerator(TrafficGenerator):
    def generate_any(self) -> tuple[Direction, ControlledIntersection, ControlledIntersection]:
        start_node = self.pick_node()
        return self.generate(start_node)
    def generate_direction(self) -> Direction:
        return random.choice(Direction)
class DirectionalTrafficGenerator(TrafficGenerator):
    def __init__(self, map_, direction_in):
        super().__init__(map_)
        self.direction_in = direction_in
    def generate_direction(self) -> Direction:
        return self.direction_in

# TRAFFIC HANDLERS
class TrafficHandler:
    def __init__(self, map_):
        self.map_ = map_
    def get_next_node(self, node: ControlledIntersection, direction: Direction):
        return node.get_edge(direction)[0]

MAPBOX_ACCESS_TOKEN = open(".mapbox_token").read()

class TrafficMapper:
    def __init__(self, map_: TrafficMap):
        self.map = map_
        self.vehicles = []
        self.fig = go.Figure()
        self.fig.update_layout(
            title_text = str(map_),
            title_font = dict(size=32, family="Times New Roman"),
            title_x = 0.5,
            title_xanchor = "center",
            showlegend = False,
            autosize = True,
            mapbox = dict(
                accesstoken = MAPBOX_ACCESS_TOKEN,
                style = "streets",
                center = dict(
                    lat=41.7435,
                    lon=-111.831
                ),
                pitch = 0,
                zoom = 14.5,
                bearing = 0,
            )
        )
        self.history = np.array([], dtype=np.int32)
    def save(self, v):
        lat_midpoints = []
        lon_midpoints = []
        texts = []
        intersections = []
        for i in range(len(self.map.edge_texts())):
            self.fig.add_trace(
                go.Scattermapbox(
                    mode = 'lines',
                    lon = self.map.edge_longitudes()[i],
                    lat = self.map.edge_latitudes()[i],
                    hoverinfo = 'skip',
                    line = dict(
                        width = 3,
                        color = 'rgb(255, 0, 0)'
                    )
                ))
            lat_midpoints.append(sum(self.map.edge_latitudes()[i])/2)
            lon_midpoints.append(sum(self.map.edge_longitudes()[i])/2)
            texts.append(self.map.edge_texts()[i])
            intersections.append(self.map.edge_intersections()[i])
        self.fig.add_trace(
            go.Scattermapbox(
                mode = 'markers',
                lon = lon_midpoints,
                lat = lat_midpoints,
                text = texts,
                customdata = intersections,
                hovertemplate =
                '<b>%{customdata[2]} to %{customdata[1]}</b><br>' +
                'Weight: %{text:0.4f}' + 
                '<extra><span style="color: red">Roads</span><br><em>%{customdata[0]}</em></extra>',
                marker_color = 'rgba(0,0,0,0)',
                hoverlabel = dict(
                    bgcolor = 'rgb(255, 0, 0)'
                ),
                name = "Roads"
            ))
        for i in range(len(self.map.generator_texts())):
            self.fig.add_trace(
                go.Scattermapbox(
                    mode = 'lines',
                    lon = self.map.generator_longitudes()[i],
                    lat = self.map.generator_latitudes()[i],
                    text = (self.map.generator_texts()[i], self.map.generator_texts()[i]),
                    hovertemplate = '<b>Generator (%{text:0.1f} / min)</b>',
                    line = dict(
                        width = 3,
                        color = 'rgb(0, 0, 255)'
                    ),
                    name = "Generators"
                ))
        self.fig.add_trace(
            go.Scattermapbox(
                mode = 'markers',
                lon = self.map.longitudes(),
                lat = self.map.latitudes(),
                text = self.map.texts(),
                hovertemplate = 
                '<b>%{text}</b><br>' +
                'Latitude: %{lat}<br>Longitude: %{lon}',
                marker = dict(
                    size = 4,
                    color = 'rgb(0, 0, 0)'
                ),
                name = "Intersections"
            ))
    def step(self):
        pass

# TRAVERSERS
class Traverser:
    def __init__(self, map_: TrafficMap, direction_in: Direction, start_node: ControlledIntersection, end_node: ControlledIntersection, length_multiplier: float = 1):
        self.map_ = map_
        self.start_node = start_node
        self.end_node = end_node
        self.unit_length = length_multiplier / map_.cars_per_unit

        self.current_node = self.start_node
        self.direction = direction_in

    def move_to(self, new_node, new_direction):
        self.current_node = new_node
        self.direction = new_direction

    def pick_action(self, next_node: ControlledIntersection, direction_in: Direction) -> Action:
        raise NotImplementedError()


if __name__ == '__main__':
    from logan_traffic_map import LoganTrafficMap
    map_ = LoganTrafficMap()
    tm = TrafficMapper(map_)
    tm.save(None)
    tm.fig.show()
