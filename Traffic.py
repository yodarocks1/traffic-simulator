import typing
from collections import deque
import numpy as np
import networkx as nx
from traffic_map import Action, ControlledIntersection, Direction, Phase, RoundaboutIntersection, SignIntersection, TrafficGenerator, TrafficMap, LightIntersection

class Vehicle:
    NEXT_ID = 0
    def __init__(self, handler: 'FullTrafficState', starting_node: ControlledIntersection, direction: Direction, destination_node: ControlledIntersection):
        self.id = Vehicle.NEXT_ID
        Vehicle.NEXT_ID += 1
        self.handler = handler
        self.approaching = starting_node
        self.dest = destination_node
        self.plan = handler.plan(starting_node, destination_node)
        self.next_node = self.pop_next_node()
        self.direction = direction
        self.next_direction = self.get_next_direction()
        self.edge = None
        self.progress = 1.0
        self.scheduled = False

    def go(self):
        if self.edge is not None:
            self.handler.modify_cars(self.edge[0], self.edge[1], -1)
            
        if self.is_approaching_destination():
            try:
                self.handler.vehicles.remove(self)
            except ValueError:
                pass
            return
        self.approaching = self.next_node
        self.direction = self.next_direction
        self.next_node = self.pop_next_node()
        self.next_direction = self.get_next_direction()
        self.edge = self.get_next_edge()
        self.progress = 0.0
        self.scheduled = False

        if self.edge is not None:
            self.handler.modify_cars(self.edge[0], self.edge[1], 1)

    def get_approaching_action(self):
        desired_direction = self.get_next_direction()
        if desired_direction is None:
            return None
        elif desired_direction is self.direction:
            return Action.STRAIGHTS
        elif desired_direction is self.direction.right():
            return Action.RIGHT_TURNS
        else:
            return Action.LEFT_TURNS
    def get_approaching_phase_part(self):
        action = self.get_approaching_action()
        if action is None:
            return None
        return self.direction * action

    def pop_next_node(self):
        if self.approaching == self.dest or len(self.plan) == 0:
            return None
        nxt = self.plan.popleft()
        if nxt == self.approaching:
            return self.pop_next_node()
        return nxt
    def get_next_direction(self):
        if self.next_node is None:
            return None
        if self.next_node not in self.approaching.edges:
            self.handler.vehicles.remove(self)
            return None
        return self.approaching.get_edge_data(self.next_node)["direction"]
    def get_next_edge(self):
        next_direction = self.get_next_direction()
        if next_direction is None:
            return None
        return (self.approaching, *self.approaching.get_edge(next_direction))
    def get_next_node(self):
        return self.next_node
    def is_approaching_destination(self):
        return self.get_next_node() is None


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

    def get_queueing(self):
        return 0
    def get_moving(self):
        return sum(self.data[len(self.map.nodes):]) - self.get_queueing()

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
    def max_cars(self):
        return max(self.data[len(self.map.nodes):])

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
        self.set_node_value(node, new_value)

    def save(self):
        return NaiveTrafficState(self.map, self.data.copy(), self.throughput.copy())

    def step(self):
        raise NotImplementedError()

class NaiveTrafficState(TrafficState):
    pass


class FullTrafficState(TrafficState):
    def __init__(self, map_: TrafficMap, phase_controller: type['NullPhaseController'] = None, time_step: float = 2/60):
        super().__init__(map_)

        self.vehicles: list[Vehicle] = []
        self.time_step = time_step

        all_dests, weights = self.map.get_destinations_and_weights()
        weights = np.array(weights)/sum(weights)
        self.get_dests = lambda size: np.random.choice(all_dests, size, p=weights)

        self.schedule: dict[ControlledIntersection, dict[typing.Union[Phase, Direction], list[Vehicle]]] = {}

        self.phase_controller: 'NullPhaseController' = phase_controller
        if phase_controller is None:
            self.phase_controller = NullPhaseController
        light_intersections = list(filter(lambda x: isinstance(x, LightIntersection), self.map.nodes))
        self.phase_controller = self.phase_controller(light_intersections, self.get_traffic_callback, self.phase_action_callback)

    def get_queueing(self):
        return sum(map(lambda v: sum(map(len, v.values())), self.schedule.values()))
    def get_queueing_ordered(self):
        v = []
        for intersection in self.map.nodes:
            if intersection not in self.schedule:
                v.append(0)
            else:
                schedule = self.schedule[intersection]
                v.append(sum(map(len, schedule.values())))
        return v

    def add_schedule(self, vehicle: Vehicle, intersection: ControlledIntersection):
        if intersection not in self.schedule:
            self.schedule[intersection] = {}
        
        if type(intersection) is SignIntersection and intersection.through is False:
            key = vehicle.direction
        else:
            key = vehicle.get_approaching_phase_part()

        if key is None:
            vehicle.go()
            return
        
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

        count = min(n, len(self.schedule[node][key]))
        if type(key) is Phase:
            phase_part = key
        else:
            phase_parts = (key * self.schedule[node][key][i].get_approaching_phase_part() for i in range(count))
            phase_part = Phase.NONE
            for part in phase_parts:
                if type(part) is Phase:
                    phase_part |= part
        if validate and not (active_phase | phase_part).is_valid():
            return 0, active_phase

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
                phase_parts = list(self.schedule[intersection].keys())
                active_phase = Phase.NONE
                for phase_part in phase_parts:
                    if phase_part in phase:
                        count, active_phase = self.send_through_n(intersection, active_phase, phase_part, validate=False, n=25)
                        self.throughput[idx] += count
                count, active_phase = self.send_through_n(intersection, active_phase, Action.RIGHT_TURNS, validate=True, n=25)
                if count > 0:
                    self.throughput[idx] += count
                yield_directions = phase // Action.LEFT_YIELDS
                if yield_directions is not Direction.NONE:
                    for direction in yield_directions:
                        count, active_phase = self.send_through_n(intersection, active_phase, direction * Action.LEFT_YIELDS, validate=True, n=10)
                        if count > 0:
                            self.throughput[idx] += count

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

                self.set_node_value(intersection, v)

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
                for direction in Direction:
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
        cars = self.get_cars(node_from, node_to)
        return weight * (cars + 20) / 26.25

    def get_traffic_callback(self, intersection: ControlledIntersection) -> tuple[int, int, int, int, int]:
        """
        Returns:
            0 <int>: Throughput
            1 <int>: Waiting NB
            2 <int>: Waiting SB
            3 <int>: Waiting EB
            4 <int>: Waiting WB
        """
        throughput = self.throughput[self.node_indexer(intersection)]
        if intersection not in self.edge_indexer:
            return (throughput, 0, 0, 0, 0)
        
        indexer = self.edge_indexer[intersection]
        out = []
        for direction in (Direction.NB, Direction.SB, Direction.EB, Direction.WB):
            edge = intersection.get_edge(direction)
            if edge is None or edge[0] not in indexer:
                out.append(0)
            else:
                idx = indexer[edge[0]]
                out.append(self[idx])
        return (throughput, *out)

    def phase_action_callback(self, intersection: ControlledIntersection, action: int):
        if action == 0:
            return
        elif action == 1:
            self.next_phase(intersection)
        else:
            self.set_current_phase(intersection, action - 2)

    def plan(self, node_from, node_to) -> list[ControlledIntersection]:
        return deque(nx.shortest_path(self.map, source=node_from, target=node_to, weight=self.effective_weight))

    def step(self):
        self.do_generate()
        self.throughput.fill(0)
        self.phase_controller.step(time_step = self.time_step)
        for vehicle in self.vehicles:
            if vehicle.progress < 1:
                vehicle.progress += self.time_step
            elif not vehicle.scheduled:
                self.add_schedule(vehicle, vehicle.approaching)
                vehicle.scheduled = True
        self.run_schedule()


class NullPhaseController:
    def __init__(self, lights: list[LightIntersection],
                 get_traffic: typing.Callable[[ControlledIntersection], tuple[int, int, int, int, int]],
                 take_phase_action: typing.Callable[[ControlledIntersection, int], None]):
        self.lights = lights
        self.last_changed = np.zeros(len(self.lights))
        self.get_traffic = get_traffic
        self._phase_action_no_delay = take_phase_action
        self.delay_time = 15/60

    def take_phase_action(self, intersection, int):
        idx = self.lights.index(intersection)
        if self.last_changed[idx] > self.delay_time:
            self._phase_action_no_delay(intersection, int)
            self.last_changed[idx] = 0

    def step(self, time_step=2/60):
        self.last_changed += time_step

class NaivePhaseController(NullPhaseController):
    def step(self, time_step=2/60):
        super().step(time_step=time_step)
        for light in self.lights:
            self.take_phase_action(light, 1)
