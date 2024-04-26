from dash import Dash, html, dcc, Input, Output, Patch, State, callback, clientside_callback
import plotly.graph_objects as go
import numpy as np
import networkx as nx
from Traffic import NaivePhaseController, TrafficState, FullTrafficState
from traffic_map import TrafficMap

MAPBOX_ACCESS_TOKEN = open(".mapbox_token").read()

class TrafficMapper:
    def __init__(self, map_: TrafficMap, traffic_state: TrafficState):
        self.map = map_
        self.traffic_state = traffic_state
        self.fig = go.Figure(
            data=self.get_base(),
            layout=dict(
                title_text = str(map_),
                title_font = dict(size=32, family="Times New Roman"),
                title_x = 0.5,
                title_xanchor = "center",
                height = 1000,
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
                ),
            ),
        )
        self.app = Dash(str(map_))
        self.app.layout = html.Div(
            [

                dcc.Graph(figure=self.fig, id='my-fig', config={"displaylogo": False}),
                #dcc.Slider(0, len(history), 1, value=0, marks=None, id='time-slider'),
                dcc.Interval(id='timer', interval=1000),
            ]
        )
        self.history: list[TrafficState] = []
        self.create_callbacks()
    def get_base(self):
        traces = []

        lat_midpoints = []
        lon_midpoints = []
        texts = []
        intersections = []
        for i in range(len(self.map.edge_texts())):
            lon = self.map.edge_longitudes()[i]
            lat = self.map.edge_latitudes()[i]
            text = (None, self.map.edge_texts()[i], None)
            road = (None, self.map.edge_intersections()[i], None)
            #effective_weight = (None, self.map.edge_texts()[i], None)
            cars = [None, 0, None]
            lon = (lon[0], sum(lon)/2, lon[1])
            lat = (lat[0], sum(lat)/2, lat[1])
            traces.append(
                go.Scattermapbox(
                    mode = 'markers+lines',
                    lon = lon,
                    lat = lat,
                    text = text,
                    customdata = list(zip(road, cars)),# effective_weight)),
                    hovertemplate = [None,
                        '<b>%{customdata[0][1]} to %{customdata[0][2]}</b><br>' +
                        'Weight: %{text:0.4f}<br>' + 
                        'Vehicles: %{customdata[1]}' +#<br>' +
                        #'Effective Weight: %{customdata[2]:0.4f}' +
                        '<extra><span style="color: red">Roads</span><br><em>%{customdata[0][0]}</em></extra>',
                    None],
                    marker_color = 'rgba(0, 0, 0, 0)',
                    line = dict(
                        width = 3,
                        color = 'rgb(0, 0, 0)'
                    ),
                    name = f"Road Segment {i:05d}",
                ))
        for i in range(len(self.map.generator_texts())):
            traces.append(
                go.Scattermapbox(
                    mode = 'lines',
                    lon = self.map.generator_longitudes()[i],
                    lat = self.map.generator_latitudes()[i],
                    text = (self.map.generator_texts()[i], self.map.generator_texts()[i]),
                    hovertemplate =
                    '<b>Generator (%{text:0.1f} / min)</b>',
                    line = dict(
                        width = 3,
                        color = 'rgb(0, 0, 255)'
                    ),
                    name = "Generators"
                ))
        traces.append(
            go.Scattermapbox(
                mode = 'markers',
                lon = self.map.longitudes(),
                lat = self.map.latitudes(),
                text = self.map.texts(),
                customdata = list(zip(self.map.extras(), self.traffic_state.get_queueing_ordered())),
                hovertemplate = 
                '<b>%{text}</b><br>' +
                'Latitude: %{lat}<br>Longitude: %{lon}<br>' +
                'Queueing: %{customdata[1]}' +
                '<extra>Intersections<br>%{customdata[0]}</extra>',
                marker = dict(
                    size = 4,
                    color = 'rgb(0, 0, 0)'
                ),
                name = "Intersections"
            ))
        self.get_base = lambda: traces[:]

        return traces

    def create_callbacks(self):
        #@callback(Output('my-fig', "figure"), Input('time-slider', "value"), State('my-fig', "figure"))
        #def update_colors(value, fig):
        #    patched_figure = Patch()
        #    for i in range(len(fig["data"])):
        #        if "name" in fig["data"][i]:
        #            if fig["data"][i]["name"] == "Road Segment":
        #                patched_figure["data"][i]["line"]["color"] = f'rgb({value}, 0, 0)'
        #    return patched_figure
        pass
        
    def run(self, debug=False):
        self.app.run(debug=debug)

    def step(self):
        pass

@callback(Output('my-fig', "figure"), Input('timer', "n_intervals"), State('my-fig', "figure"))
def step_traffic(n_intervals, fig):
    traffic.step()
    tm.history.append(traffic.save())
    patched_figure = Patch()
    name_map = {}
    for i in range(len(fig["data"])):
        if "name" in fig["data"][i]:
            name_map[fig["data"][i]["name"]] = i
    red = (255, 0, 0)
    yellow = (200, 200, 0)
    green = (0, 128, 0)
    threshold = traffic.max_cars() + 10
    def colorize(v):
        if v >= threshold:
            return red
        elif v >= threshold / 2:
            part_red = (v - (threshold / 2)) / (threshold / 2)
            part_yellow = 1 - part_red
            return (
                str(red[0] * part_red + yellow[0] * part_yellow),
                str(red[1] * part_red + yellow[1] * part_yellow),
                str(red[2] * part_red + yellow[2] * part_yellow),
            )
        else:
            part_yellow = v / (threshold / 2)
            part_green = 1 - part_yellow
            return (
                str(green[0] * part_green + yellow[0] * part_yellow),
                str(green[1] * part_green + yellow[1] * part_yellow),
                str(green[2] * part_green + yellow[2] * part_yellow),
            )
    for edge in tm.map.edge_list:
        i = tm.map.get_edge_index(edge[0], edge[1])
        node = name_map[f"Road Segment {i:05d}"]
        v = traffic.get_cars(edge[0], edge[1])
        patched_figure.data[node].line.color = 'rgb(' + ','.join(colorize(v)) + ')'
        patched_figure.data[node].customdata[1][1] = v
        #patched_figure.data[node].customdata[2][1] = str(traffic.effective_weight(edge[0], edge[1], edge[2]))
    for node in tm.map.nodes:
        node = name_map["Intersections"]
        patched_figure.data[node].customdata[1] = traffic.get_queueing_ordered()
    return patched_figure

if __name__ == '__main__':
    from logan_traffic_map import LoganTrafficMap
    map_ = LoganTrafficMap()
    traffic = FullTrafficState(map_, phase_controller=NaivePhaseController)
    tm = TrafficMapper(map_, traffic)
    tm.run(debug=True)
