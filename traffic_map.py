from collections import OrderedDict
import enum
import typing
import networkx as nx

EdgeWeight = typing.NewType("EdgeWeight", float)
Edge = tuple['ControlledIntersection', 'ControlledIntersection', dict[str, typing.Union[EdgeWeight, 'Direction']]]
Generator = tuple['ControlledIntersection', 'TrafficGenerator', 'Direction']

# TRAFFIC MAP
class TrafficMap(nx.DiGraph):
    def __init__(self, node_list: list['ControlledIntersection'], edge_list: list[Edge], generator_list: list[Generator], cars_per_unit: float):
        super().__init__(edge_list)
        self.update(nodes=node_list)
        self.edge_list = edge_list
        self.generator_list = generator_list
        self.cars_per_unit = cars_per_unit
        self.edge_indexer = {}
        for i, node in enumerate(node_list):
            self.edge_indexer[node] = {None: i}
        for i, edge in enumerate(edge_list):
            self.edge_indexer[edge[0]][edge[1]] = i
    def get_observation_size(self) -> int:
        observation_size = 0
        for node in self.nodes:
            observation_size += node.get_observation_size()
        return observation_size
    def get_action_size(self) -> int:
        action_size = 0
        for node in self.nodes:
            action_size += node.get_action_size()
        return action_size
    def get_destinations_and_weights(self) -> tuple[list['ControlledIntersection'], list[EdgeWeight]]:
        destinations = {}
        for node in self.nodes:
            if node.destination_weight > 0:
                destinations[node] = node.destination_weight
        dests, weights = zip(*destinations.items())
        return dests, weights
    def get_edge_index(self, node_from: 'ControlledIntersection', node_to: 'ControlledIntersection') -> int:
        if node_from not in self.edge_indexer:
            return None
        return self.edge_indexer[node_from].get(node_to)
    def get_node_index(self, node: 'ControlledIntersection') -> int:
        if node not in self.edge_indexer:
            return None
        return self.edge_indexer[node].get(None)

    def texts(self):
        raise NotImplementedError()
    def latitudes(self):
        raise NotImplementedError()
    def longitudes(self):
        raise NotImplementedError()
    def extras(self):
        raise NotImplementedError()
    def edge_texts(self):
        raise NotImplementedError()
    def edge_latitudes(self):
        raise NotImplementedError()
    def edge_longitudes(self):
        raise NotImplementedError()
    def edge_intersections(self):
        raise NotImplementedError()
    def generator_texts(self):
        raise NotImplementedError()
    def generator_latitudes(self):
        raise NotImplementedError()
    def generator_longitudes(self):
        raise NotImplementedError()
    def draw(self, *args, **kwargs):
        nx.draw(self, *args, **kwargs)

class TrafficGenerator:
    def __init__(self, rate):
        self.rate = rate

# NODES
class ControlledIntersection:
    def __init__(self, road_a: str, road_b: str, destination_weight: int):
        self.roads = (road_a, road_b)
        self.id_ = " ".join(self.roads)
        self.destination_weight = destination_weight
        self.edges = {}
        self.generators = []
        self.nb = None
        self.sb = None
        self.eb = None
        self.wb = None

    def add_edge(self, other_node: 'ControlledIntersection', weight: EdgeWeight, direction: typing.Optional['Direction'] = None):
        self.edges[other_node] = {"weight": weight, "direction": direction}
        if direction == Direction.NB:
            self.nb = (other_node, weight)
        elif direction == Direction.SB:
            self.sb = (other_node, weight)
        elif direction == Direction.EB:
            self.eb = (other_node, weight)
        elif direction == Direction.WB:
            self.wb = (other_node, weight)

    def add_generator(self, direction: 'Direction', generator: TrafficGenerator):
        if direction == Direction.NB:
            self.nb = generator
        elif direction == Direction.SB:
            self.sb = generator
        elif direction == Direction.EB:
            self.eb = generator
        elif direction == Direction.WB:
            self.wb = generator
        self.generators.append((generator, direction))

    def get_edges(self) -> list[Edge]:
        edge_list = []
        for node in self.edges:
            edge_list.append((self, node, self.edges[node]))
        return edge_list
    def get_generators(self) -> list[Generator]:
        gen_list = []
        for generator in self.generators:
            gen_list.append((self, *generator))
        return gen_list
    def get_edge(self, direction: 'Direction') -> tuple['ControlledIntersection', EdgeWeight]:
        if direction == Direction.NB:
            return self.nb
        elif direction == Direction.EB:
            return self.eb
        elif direction == Direction.SB:
            return self.sb
        elif direction == Direction.WB:
            return self.wb
    def get_edge_data(self, other: 'ControlledIntersection') -> dict[str, typing.Union[EdgeWeight, 'Direction']]:
        return self.edges.get(other)

    @classmethod
    def get_icon(cls) -> str:
        raise NotImplementedError()

    def __hash__(self):
        return hash(self.id_)
    def __str__(self):
        return f"{self.roads[0]}, {self.roads[1]}"
    def __repr__(self):
        return f"<{self.get_icon()} {str(self)}>"

    def get_observation_size(self) -> int:
        raise NotImplementedError()
    def get_action_size(self) -> int:
        raise NotImplementedError()
class LightIntersection(ControlledIntersection):
    def __init__(self, road_a : str, road_b : str, destination_weight: int, phase_set : 'PhaseSet'):
        super().__init__(road_a, road_b, destination_weight)
        self.phase_set = phase_set
    def get_observation_size(self) -> int:
        # 0: Throughput
        # 1: NB waiting
        # 2: SB waiting
        # 3: EB waiting
        # 4: WB waiting
        return 5
    def get_action_size(self) -> int:
        # 0: No phase change
        # 1: Next phase
        # 2-(N+1): Jump to phase
        return len(self.phases_set) + 2
    @classmethod
    def get_icon(cls) -> str:
        return "🚦"
class SignIntersection(ControlledIntersection):
    def __init__(self, road_a : str, road_b : str, destination_weight: int, through: typing.Literal[False, "NS", "EW"] = False):
        super().__init__(road_a, road_b, destination_weight)
        self.through = through
    def get_observation_size(self) -> int:
        return 0
    def get_action_size(self) -> int:
        return 0
    @classmethod
    def get_icon(cls) -> str:
        return "🛑"
class RoundaboutIntersection(ControlledIntersection):
    def __init__(self, road_a : str, road_b : str, destination_weight: int):
        super().__init__(road_a, road_b, destination_weight)
    def get_observation_size(self) -> int:
        return 0
    def get_action_size(self) -> int:
        return 0
    @classmethod
    def get_icon(cls) -> str:
        return "◯"


# DIRECTIONS, ACTIONS, & PHASES
PHASE_VALIDITY_CHECKING = True

class SomeHiddenEnumMeta(enum.EnumMeta):
    def __iter__(cls) -> typing.Iterator:
        return iter(cls._show())
    def __len__(cls) -> int:
        return len(cls._show())
    def all(cls) -> typing.Iterator:
        return super().__iter__()
    def _show(cls) -> list:
        show = set()
        for member in cls.__members__.values():
            if not (member.value & (member.value - 1)) and member.value > 0:
                show.add(member)
        show = sorted(list(show), key=lambda x: x.value)
        setattr(cls, "_show", lambda: show)
        return show
class Flag(enum.Flag, metaclass=SomeHiddenEnumMeta):
    def __iter__(self) -> typing.Iterator:
        for v in type(self):
            if v in self:
                yield v
    def __len__(self) -> int:
        return len(list(self.__iter__()))
    @classmethod
    def _names(cls) -> dict:
        return {}
    def __str__(self) -> str:
        names_def = self._names()
        current_value = self.value
        names = []
        if len(names_def) > 0:
            for value, name in names_def.items():
                if value.value & current_value == value.value:
                    names.append(name)
                    current_value &= ~value.value
                    if current_value == 0:
                        break
            if len(names) == 0:
                return ""
            elif len(names) == 1:
                return names[0]
            elif len(names) == 2:
                return names[0] + " and " + names[1]
            else:
                return ", ".join(names[:-1]) + ", and " + names[-1]
        else:
            return super().__str__() 

class Direction(Flag):
    NB = 0b0001 # Incoming North-bound
    EB = 0b0010 # Incoming East-bound
    SB = 0b0100 # Incoming South-bound
    WB = 0b1000 # Incoming West-bound

    NS = 0b0101 # North-South
    EW = 0b1010 # East-West

    NONE = 0b0000
    ALL =  0b1111

    @classmethod
    def _names(cls) -> dict['Direction', str]:
        return {
            Direction.ALL: "All",
            Direction.NS: "North-South",
            Direction.EW: "East-West",
            Direction.NB: "North-bound",
            Direction.EB: "East-bound",
            Direction.SB: "South-bound",
            Direction.WB: "West-bound",
            Direction.NONE: "No"
        }

    def left(self, n=1) -> 'Direction':
        shifted = self.value << (n % 4)
        v = (shifted // 16) + (shifted % 16)
        return Direction(v)
    def oncoming(self, n=1) -> 'Direction':
        return self.left(n*2)
    def right(self, n=1) -> 'Direction':
        return self.left(n*3)
    def same(self) -> 'Direction':
        return self

    def make(self, mask: typing.Union['Action', int]) -> int:
        if type(mask) is Action:
            mask = mask.value
        mask_0 = f"{0:0{Action.get_width()}b}"
        mask_1 = f"{mask:0{Action.get_width()}b}"
        return int(bin(self.value).replace("0b", "").replace("0", mask_0).replace("1", mask_1), base=2)
    
    def __mul__(self, other: typing.Union['Action', 'Phase'], do_cast=True) -> typing.Union['Phase', int]:
        if type(other) is Action:
            locator = self.make(Action.LOCATOR)
            if do_cast:
                return Phase(locator * other.value)
            return locator * other.value
        elif type(other) is Phase:
            mask = self.make(Action.MASK)
            return mask & other.value
class Action(Flag):
    LEFT_TURNS =    0b0001
    LEFT_YIELDS =   0b0010
    RIGHT_TURNS =   0b0100
    STRAIGHTS =     0b1000
    
    ALL =           0b1101
    NONE =          0b0000
    MASK =          0b1111
    LOCATOR =       0b0001

    @classmethod
    def get_width(cls) -> int:
        return 4

    @classmethod
    def _names(cls) -> dict['Action', str]:
        return {
            Action.ALL: "All actions",
            Action.LEFT_TURNS: "Left turns",
            Action.LEFT_YIELDS: "Left turn yields",
            Action.RIGHT_TURNS: "Right turns",
            Action.STRAIGHTS: "Straight ahead",
            Action.NONE: "No actions",
        }

    def __invert__(self):
        if Action.LEFT_YIELDS in self:
            raise ValueError("Cannot invert Action.LEFT_YIELDS")
        return super().__invert__() & Action.ALL
    def __mul__(self, other: Direction) -> 'Phase':
        return other.__mul__(self)
class Phase(Flag):
    _ignore_ = ["PhasePart", "direction", "action"]
    PhasePart = vars()
    for direction in Direction.all():
        if direction is Direction.NONE:
            continue
        for action in Action:
            PhasePart[direction.name + "_" + action.name] = direction.__mul__(action, do_cast=False)
    PhasePart["ALL"] = Direction.ALL.__mul__(Action.ALL, do_cast=False)
    PhasePart["NONE"] = Direction.NONE.__mul__(Action.NONE, do_cast=False)

    @classmethod
    def _names(cls) -> dict['Phase', str]:
        names = OrderedDict()
        for direction, d_name in Direction._names().items():
            for action, a_name in Action._names().items():
                if action.value == 0 and direction.value != 0:
                    continue
                names[direction * action] = d_name + " traffic: " + a_name
        setattr(cls, "_names", lambda cls: names)
        return names

    def get_exit_directions(self) -> dict[Direction, int]:
        directions_out = {}
        def add(direction):
            if direction not in directions_out:
                directions_out[direction] = 1
            else:
                directions_out[direction] += 1
        for direction in Direction:
            for action in self // direction:
                if Action.LEFT_TURNS in action:
                    add(direction.left())
                if Action.RIGHT_TURNS in action:
                    add(direction.right())
                if Action.STRAIGHTS in action:
                    add(direction.oncoming())
        return directions_out

    def __truediv__(self, other: typing.Union[Direction, Action, 'Phase']) -> typing.Union[dict[Direction, Action], list[Direction], 'Phase']:
        if type(other) is Direction:
            result = {}
            for direction in other:
                mask = direction.make(Action.MASK)
                locator = direction.make(Action.LOCATOR)
                result[direction] = Action((self.value & mask) // locator)
            return result
        elif type(other) is Action:
            result = []
            for direction in Direction:
                locator = direction.make(Action.LOCATOR) * other.value
                if self.value & locator > 0:
                    result.append(direction)
            return result
        return super().__truediv__(other)

    def __floordiv__(self, other: typing.Union[Direction, Action, 'Phase']) -> typing.Union[Action, Direction, 'Phase']:
        old_results = self.__truediv__(other)
        if type(other) is Direction:
            result = Action.NONE
            for old_result in old_results.values():
                result |= old_result
            return result
        elif type(other) is Action:
            result = Direction.NONE
            for old_result in old_results:
                result |= old_result
            return result
        return self.__truediv__(other)

    def __and__(self, other: typing.Union[Direction, Action, 'Phase']) -> 'Phase':
        if type(other) is Direction:
            return Phase(self.value & other.make(Action.MASK))
        elif type(other) is Action:
            return Phase(self.value & (other * Direction.ALL))
        return super().__and__(other)

    def __contains__(self, other: typing.Union[Direction, Action, 'Phase']) -> bool:
        if type(other) is Direction:
            for direction in Direction:
                v = self // direction

        elif type(other) is Action:
            pass
        return super().__contains__(other)

    def is_valid(self) -> bool:
        # Check if outputs collide
        enters = self.get_exit_directions()
        for v in enters.values():
            if v > 1:
                return False

        # Check if paths collide
        for direction, actions in (self / Direction.ALL).items():
            oncoming = direction.oncoming()
            cross = direction.left() | direction.right()
            if actions not in (Action.RIGHT_TURNS, Action.NONE):
                cross_actions = self // cross # Actions done by either the left or the right
                if cross_actions not in (Action.RIGHT_TURNS, Action.NONE):
                    return False
                if Action.LEFT_TURNS in actions and Action.STRAIGHTS in self // oncoming:
                    return False

        return True
class PhaseSet(list[Phase]):
    def __init__(self, *phases : Phase, require_all_phases: bool = True):
        super().__init__(phases)
        if PHASE_VALIDITY_CHECKING:
            if require_all_phases and not self.has_all_phases():
                raise ValueError("Phase set does not hit all phases")
            for phase in self:
                if not phase.is_valid():
                    raise ValueError(f"Invalid phase in phase set:\n{str(phase)}")

    def has_all_phases(self) -> bool:
        contained = {}
        for direction in Direction:
            contained[direction] = {"straights": False, "lefts": False}
        for phase in self:
            for direction in Direction:
                action = phase // direction
                if Action.STRAIGHTS in action:
                    contained[direction]["straights"] = True
                if Action.LEFT_TURNS in action or Action.LEFT_YIELDS in action:
                    contained[direction]["lefts"] = True
        for direction in Direction:
            if not (contained[direction]["straights"] and contained[direction]["lefts"]):
                return False
        return True

ACTION_PHASE_SETS: dict[str, PhaseSet] = {
    "side-by-side": PhaseSet(
        Phase.NB_LEFT_TURNS | Phase.NB_STRAIGHTS,
        Phase.EB_LEFT_TURNS | Phase.EB_STRAIGHTS,
        Phase.SB_LEFT_TURNS | Phase.SB_STRAIGHTS,
        Phase.WB_LEFT_TURNS | Phase.WB_STRAIGHTS,
    ),
    "straights-first": PhaseSet(
        Phase.NS_STRAIGHTS,
        Phase.NS_LEFT_TURNS,
        Phase.EW_STRAIGHTS,
        Phase.EW_LEFT_TURNS,
    ),
    "lefts-first": PhaseSet(
        Phase.NS_LEFT_TURNS,
        Phase.NS_STRAIGHTS,
        Phase.EW_LEFT_TURNS,
        Phase.EW_STRAIGHTS,
    ),
    "straights-first-yield": PhaseSet(
        Phase.NS_STRAIGHTS | Phase.NS_LEFT_YIELDS,
        Phase.NS_LEFT_TURNS,
        Phase.EW_STRAIGHTS | Phase.EW_LEFT_YIELDS,
        Phase.EW_LEFT_TURNS,
    ),
    "lefts-first-yield": PhaseSet(
        Phase.NS_LEFT_TURNS,
        Phase.NS_STRAIGHTS | Phase.NS_LEFT_YIELDS,
        Phase.EW_LEFT_TURNS,
        Phase.EW_STRAIGHTS | Phase.EW_LEFT_YIELDS,
    ),
    "offset-lefts-a": PhaseSet(
        Phase.NB_LEFT_TURNS | Phase.NB_STRAIGHTS,
        Phase.NS_STRAIGHTS,
        Phase.SB_LEFT_TURNS | Phase.SB_STRAIGHTS,
        Phase.EB_LEFT_TURNS | Phase.EB_STRAIGHTS,
        Phase.EW_STRAIGHTS,
        Phase.WB_LEFT_TURNS | Phase.WB_STRAIGHTS,
    ),
    "offset-lefts-b": PhaseSet(
        Phase.SB_LEFT_TURNS | Phase.SB_STRAIGHTS,
        Phase.NS_STRAIGHTS,
        Phase.NB_LEFT_TURNS | Phase.NB_STRAIGHTS,
        Phase.WB_LEFT_TURNS | Phase.WB_STRAIGHTS,
        Phase.EW_STRAIGHTS,
        Phase.EB_LEFT_TURNS | Phase.EB_STRAIGHTS,
    ),
}

USE_ALL_PHASE_SETS = False
if USE_ALL_PHASE_SETS:
    PHASE_VALIDITY_CHECKING = False
    class AllPhaseSet(PhaseSet):
        def __getitem__(self, *args, **kwargs):
            return Phase.ALL
        def __iter__(self):
            yield Phase.ALL
    ALL_PHASE_SET = AllPhaseSet(Phase.ALL)
    class AllPhaseSetSet:
        def __getitem__(self, *args, **kwargs):
            return ALL_PHASE_SET
    ACTION_PHASE_SETS = AllPhaseSetSet()