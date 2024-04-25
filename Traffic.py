import typing
from collections import deque
import numpy as np
import networkx as nx
from traffic_map import Action, ControlledIntersection, Direction, Phase, RoundaboutIntersection, SignIntersection, TrafficMap, LightIntersection

class Vehicle:
    def __init__(self, handler: 'FullTrafficState', starting_node: ControlledIntersection, direction: Direction, destination_node: ControlledIntersection):
        self.handler = handler
        self.plan = handler.plan(starting_node, destination_node)
        self.next = starting_node
        self.next_direction = self.next.get_edge_data(self.plan[0])
        if self.next_direction is not None:
            self.next_direction = self.next_direction["direction"].oncoming()
        self.dest = destination_node
        self.direction = direction
        self.edge = None
        self.progress = 1.0

    def go(self):
        if self.edge is not None:
            self.handler.modify_cars(self.edge[0], self.edge[1], -1)

        node = self.next
        if self.next == self.dest or len(self.plan) == 0:
            try:
                self.handler.vehicles.remove(self)
            except ValueError:
                pass
            return
        self.next = self.plan.popleft()

        next_direction_in = node.get_edge_data(self.next)["direction"]
        next_direction_out = next_direction_in.oncoming()
        
        self.edge = (node, *node.get_edge(next_direction_in))
        self.direction = next_direction_out
        self.progress = 0.0
        if len(self.plan) == 0:
            self.next_direction = None
        else:
            self.next_direction = self.next.get_edge_data(self.plan[0])
            if self.next_direction is not None:
                self.next_direction = self.next_direction["direction"].oncoming()

        if self.edge is not None:
            self.handler.modify_cars(self.edge[0], self.edge[1], 1)

    def get_next_action(self):
        desired_direction = self.next_direction
        if desired_direction is self.direction:
            return Action.STRAIGHTS
        elif desired_direction is self.direction.right():
            return Action.RIGHT_TURNS
        else:
            return Action.LEFT_TURNS
    def get_next_phase_part(self):
        return self.direction * self.get_next_action()


class TrafficState:
    def __init__(self, map_: TrafficMap, data: np.ndarray = None, throughput_data: np.ndarray = None):
        self.map = map_
        if data is None:
            self.data = np.zeros(len(self.map.edge_list) + len(self.map.nodes), dtype=np.int32)
        else:
            self.data = np.array(data, dtype=np.int32)

        self.node_indexer = {}
        for i, node in enumerate(self.map.nodes):
            self.node_indexer[node] = i

        self.edge_indexer = {}
        for i, edge in enumerate(self.map.edge_list, start=len(self.map.nodes)):
            if edge[0] not in self.edge_indexer:
                self.edge_indexer[edge[0]] = {}
            self.edge_indexer[edge[0]][edge[1]] = i

        if throughput_data is None:
            self.throughput = np.zeros(len(self.map.nodes), dtype=np.int32)
        else:
            self.throughput = np.array(throughput_data, dtype=np.int32)

    def __getitem__(self, *args, **kwargs):
        return self.data.__getitem__(*args, **kwargs)
    def __setitem__(self, *args, **kwargs):
        return self.data.__setitem__(*args, **kwargs)
    def __iter__(self, *args, **kwargs):
        return self.data.__iter__(*args, **kwargs)

    def get_edge_value(self, node_from: ControlledIntersection, node_to: ControlledIntersection) -> int:
        if node_from in self.edge_indexer:
            if node_to in self.edge_indexer[node_from]:
                return self[self.edge_indexer[node_from][node_to]]
        raise ValueError("Cannot get value of unknown edge")
    def set_edge_value(self, node_from: ControlledIntersection, node_to: ControlledIntersection, value: int):
        if node_from not in self.edge_indexer or node_to not in self.edge_indexer[node_from]:
            raise ValueError("Cannot set value to unknown edge")
        self[self.edge_indexer[node_from][node_to]] = value
    def get_cars(self, node_from: ControlledIntersection, node_to: ControlledIntersection) -> int:
        return self.get_edge_value(node_from, node_to)
    def set_cars(self, node_from: ControlledIntersection, node_to: ControlledIntersection, value: int):
        self.set_edge_value(node_from, node_to, value)
    def modify_cars(self, node_from: ControlledIntersection, node_to: ControlledIntersection, change: int) -> int:
        v = self.get_cars(node_from, node_to)
        self.set_cars(node_from, node_to, v + change)
        return v + change

    def get_node_value(self, node: ControlledIntersection) -> int:
        return self[self.node_indexer[node]]
    def set_node_value(self, node: ControlledIntersection, value: int):
        self[self.node_indexer[node]] = value
    def get_current_phase(self, node: LightIntersection) -> Phase:
        return node.phase_set[self.get_node_value(node)]
    def set_current_phase(self, node: LightIntersection, idx: int) -> Phase:
        self.set_node_value(node, idx)
    def next_phase(self, node: LightIntersection):
        new_value = (self.get_node_value(node) + 1) % len(node.phase_set)
        self.set_node_value(new_value)

    def save(self):
        return NaiveTrafficState(self.map, self.copy(), self.throughput.copy())

    def step(self):
        raise NotImplementedError()

class NaiveTrafficState(TrafficState):
    pass


class FullTrafficState(TrafficState):
    def __init__(self, map_: TrafficMap, time_step: float = 2/60):
        super().__init__(map_)

        self.vehicles: list[Vehicle] = []
        self.time_step = time_step

        all_dests, weights = self.map.get_destinations_and_weights()
        weights = np.array(weights)/sum(weights)
        self.get_dests = lambda size: np.random.choice(all_dests, size, p=weights)

        self.schedule: dict[ControlledIntersection, dict[typing.Union[Phase, Direction], list[Vehicle]]] = {}

    def add_schedule(self, vehicle: Vehicle, intersection: ControlledIntersection):
        if intersection not in self.schedule:
            self.schedule[intersection] = {}
        
        if type(intersection) is SignIntersection and intersection.through is False:
            key = vehicle.direction
        else:
            key = vehicle.get_next_phase_part()
        
        if key not in self.schedule[intersection]:
            self.schedule[intersection][key] = deque()

        self.schedule[intersection][key].append(vehicle)

    def send_through_all(self, node: ControlledIntersection, active_phase: Phase, key: typing.Union[Phase, Direction], validate=False) -> tuple[int, Phase]:
        if key not in self.schedule[node]:
            return 0, active_phase

        phase_part = key if type(key) is Phase else (key * self.schedule[node][key].get_next_phase_part())
        if validate and not (active_phase | phase_part).is_valid():
            return 0, active_phase
        count = len(self.schedule[node][key])
        for vehicle in self.schedule[node][key]:
            vehicle.go()
        del self.schedule[node][key]
        return count, active_phase | phase_part
    def send_through_n(self, node: ControlledIntersection, active_phase: Phase, key: typing.Union[Phase, Direction], validate=False, n=1) -> tuple[int, Phase]:
        if key not in self.schedule[node]:
            return 0, active_phase

        phase_part = key if type(key) is Phase else (key * self.schedule[node][key].get_next_phase_part())
        if validate and not (active_phase | phase_part).is_valid():
            return 0, active_phase
        count = min(n, len(self.schedule[node][key]))
        for _ in range(count):
            self.schedule[node][key].popleft().go()
        if len(self.schedule[node][key]) == 0:
            del self.schedule[node][key]
        return count, active_phase | phase_part

    def run_schedule(self):
        for intersection in self.schedule:
            idx = self.node_indexer[intersection]
            if type(intersection) is LightIntersection:
                phase = self.get_current_phase(intersection)
                for phase_part in self.schedule[intersection]:
                    if phase_part in phase:
                        self.throughput[idx] += self.send_through_n(intersection, n=3)[0]

            elif type(intersection) is SignIntersection and intersection.through is False:
                # Round-Robin FIFO
                # Assumes one lane per side
                v = self.get_node_value(intersection)
                directions = deque(Direction)
                directions.rotate(v)
                while len(directions) > 0 and directions[0] not in self.schedule[intersection]:
                    v = (v + 1) % 4
                    directions.popleft()

                active_phase: Phase = Phase.NONE
                for direction in directions:
                    count, active_phase = self.send_through_n(intersection, active_phase, direction, validate=True)
                    self.throughput[idx] += count

                self.set_node_value(v)

            elif type(intersection) is SignIntersection:
                if intersection.through == "NS":
                    through_directions = [Direction.NB, Direction.SB]
                    stop_directions = [Direction.EB, Direction.WB]
                elif intersection.through == "EW":
                    through_directions = [Direction.EB, Direction.WB]
                    stop_directions = [Direction.NB, Direction.SB]
                    
                active_phase: Phase = Phase.NONE

                for through_direction in through_directions:
                    count1, active_phase = self.send_through_all(intersection, active_phase, Action.STRAIGHTS * through_direction)
                    count2, active_phase = self.send_through_all(intersection, active_phase, Action.RIGHT_TURNS * through_direction)
                    self.throughput[idx] += count1 + count2
                if len(self.schedule[intersection]) == 0:
                    continue
                for through_direction in through_directions:
                    if active_phase // through_direction.oncoming() == Action.NONE:
                        count3, active_phase = self.send_through_all(intersection, active_phase, Action.LEFT_TURNS * through_direction)
                        self.throughput[idx] += count3
                if len(self.schedule[intersection]) == 0:
                    continue
                        
                # Check rights and straights first
                sent = [False] * len(stop_directions)
                for i in range(len(stop_directions)):
                    if (v := self.send_through_n(intersection, active_phase, Action.RIGHT_TURNS * stop_directions[i], validate=True))[0] == 1:
                        self.throughput[idx] += 1
                        active_phase = v[1]
                    elif (v := self.send_through_n(intersection, active_phase, Action.STRAIGHTS * stop_directions[i], validate=True))[0] == 1:
                        self.throughput[idx] += 1
                        active_phase = v[1]
                if len(self.schedule[intersection]) == 0:
                    continue
                for i in range(len(stop_directions)):
                    if not sent[i]:
                        count, active_phase = self.send_through_n(intersection, active_phase, Action.STRAIGHTS * stop_directions[i], validate=True)
                        self.throughput[idx] += count
                
            elif type(intersection) is RoundaboutIntersection:
                # FIFO
                # Assumes one lane per side
                for direction in directions:
                    count, active_phase = self.send_through_n(intersection, active_phase, direction, validate=False)
                    self.throughput[idx] += count

    def do_generate(self):
        norms = np.random.normal(1, 0.25, len(self.map.generator_list))
        i = 0
        for node, generator, direction in self.map.generator_list:
            n = max(int(generator.rate * norms[i]), 0)
            dests = self.get_dests(n)
            for j in range(n):
                self.vehicles.append(Vehicle(self, node, direction.oncoming(), dests[j]))
            i += 1

    def effective_weight(self, node_from, node_to, attrs):
        weight = attrs["weight"]
        cars = self[self.edge_indexer[node_from][node_to]]
        v = weight * (cars + 20) / 26.25
        if v <= 0:
            print(v)
            print(node_from)
            print(node_to)
            print(attrs)
            print(cars)
        return v

    def plan(self, node_from, node_to) -> list[ControlledIntersection]:
        return deque(nx.shortest_path(self.map, source=node_from, target=node_to, weight=self.effective_weight))

    def step(self):
        self.do_generate()
        self.throughput.fill(0)
        for vehicle in self.vehicles:
            if vehicle.progress < 1:
                vehicle.progress += self.time_step
            else:
                self.add_schedule(vehicle, vehicle.next)
        self.run_schedule()
