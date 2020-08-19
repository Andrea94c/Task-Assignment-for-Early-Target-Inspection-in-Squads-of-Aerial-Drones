"""
Owner: Andrea Coletta
Version v1.0
Release code for TMC : 10.1109/TMC.2020.2994529 and its ICDCS conference version : 10.1109/ICDCS.2019.00209

Please cite these works in case of use.


File content:
This file contains all the TRAJectory planning ENTITIES (TRAJENTITIES). Classes and relative functions needed to run the
AC-GaP and AC-TCP code.

The main classes are:
    Tours -> a wrapper which represent a tour for a drone.
    Drone -> a drone entity that has a given speed and available energy
    MultiRoundSolution -> An assignment of tours to drones to cover a given set of points

The core is based on networkx and python3.8.
The code consider 2d coordinates for point-of-interest (targets); 3D is not supported by this version.
All tours, solutions and whatever should refers to node or edges of a graph that represents the set of targets points
    and the only viable edges.

For simplicity of plotting we consider the Area Of Interest as a rectangle with an area (width * height).
Notice that any area with no-fly zone can be still considered: the output solutions will only considers the input target points and given viable edges/paths,
    regardeless width and height of area.

"""

import networkx as nx
import math

from matplotlib import pyplot as plt

from src.util.trajplot import AoIPlotManager, ToursPlotManager


def euclidean_distance(point1, point2):
    """

    :param point1: the first 2D point/node
    :param point2: the second 2D point/node
    :return: float -> the euclidean distance between two points
    """
    return math.sqrt(math.pow(point2[0] - point1[0], 2)
                     + math.pow(point2[1] - point1[1], 2))

# -----------------------------------------------------------#
#
#           ___ _  _ _____ ___ _____ ___ ___ ___
#          | __| \| |_   _|_ _|_   _|_ _| __/ __|
#          | _|| .` | | |  | |  | |  | || _|\__ \
#          |___|_|\_| |_| |___| |_| |___|___|___/
#
# -----------------------------------------------------------#

# the input Area of Interest with several targets point to inspect
class AoI():

    def __init__(self, depots: list, target_points: list, width: int, height: int,
                 node_hovering_time: int = 0, viable_paths: list = None):
        """
        :param depots: a list of depots (drones base stations) represented as 2D coordinates. E.g., [(x1, y1), (x2, y2), ..., (xn; yn)]
        :param target_points: a list of target points represented as 2D coordinates. E.g., [(x1, y1), (x2, y2), ..., (xn; yn)]
        :param width: the width of the area of interest. It does not affect solutions, only plots.
        :param height: the height of the area of interest. It does not affect solutions, only plots.
        :param node_hovering_time: the required hovering time in seconds for each target (default : 0 seconds)
        :param viable_paths : the only paths viables. A path is a tuple of two points. Input e.g., [(p1,p2), (p2,p3), ...]
                            where p1 = (x1, y1). If None all the paths (edegs) between points are considered viable (default: None)
        """
        # assert no duplicates target points and depots
        assert len(set(target_points)) == len(target_points), "the target points should not have duplicates " \
                                                              "(same coordinates are found in the given " \
                                                              "list) "
        assert len(set(depots)) == len(depots), "the depots should not have duplicates (same coordinates " \
                                                "are found in the given list) "
        self.depots = depots
        self.target_points = target_points
        self.width = width
        self.height = height
        self.n_depots = len(depots)
        self.n_targets = len(target_points)
        self.node_hovering_time = node_hovering_time
        self.viable_paths = viable_paths
        self.graph = self.__build_graph()

    def __build_graph(self):
        """ build the internal represents of the AoI using a graph with networkx module.

            Each graph has 3 attributes:
                pos -> tuple : the 2-d coordinates of the depot/target
                weight -> float : the hovering time (in seconds) to inspect that target; this time is set to 0 for depots
                depot -> boolean : whether the graph node is a target or a depot.

            Each edge is weighted:
                weight -> float : euclidean distance between the two nodes
         """
        # generate the empty Graph
        G = nx.Graph()
        G.add_nodes_from(range(0, self.n_targets + self.n_depots))

        # add nodes
        for i in range(0, self.n_targets):
            G.nodes[i]["pos"] = self.target_points[i]
            G.nodes[i]["weight"] = self.node_hovering_time
            G.nodes[i]["depot"] = 0

        # add depot with no weight
        for i in range(self.n_depots):
            i_node = i + self.n_targets
            G.nodes[i_node]["pos"] = self.depots[i]
            G.nodes[i_node]["weight"] = 0
            G.nodes[i_node]["depot"] = 1

        # add depot to all nodes
        all_nodes = self.target_points + self.depots

        # add weighted edges between nodes
        if self.viable_paths is None:
            for i in range(0, len(all_nodes)):
                for j in range(i + 1, len(all_nodes)):
                    w_ij = euclidean_distance(all_nodes[j], all_nodes[i])
                    G.add_edge(i, j, weight=w_ij)
        else:
            for viable_edge in self.viable_paths:
                coord_i, coord_j = viable_edge
                i, j = all_nodes.index(coord_i), all_nodes.index(coord_j)
                w_ij = euclidean_distance(coord_i, coord_j)
                G.add_edge(i, j, weight=w_ij)

        # the graph, by construction, should respect the triangle inequality
        return G

    def __str__(self):
        return "{}x{} Aoi with {} depots and {} target".format(self.width, self.height, self.n_depots, self.n_targets)

    def __repr__(self):
        return self.__str__()

    def plot(self, labels: bool = False, edges: bool = False):
        """
        plot the graph with edges or not, upon input
        :param labels: if True each viable path (edge) will have a lebel with the weight; Otherwise no labels on edges (default=False)
        :param edges: if True the viable paths/edges will be printed; otherwise only the targets and depots are plotted (default=False)
        """
        plot_builder = AoIPlotManager(self, labels=labels, edges=edges)
        plot_builder.show()

    def save(self, path: str, labels: bool = False, edges: bool = False):
        """
        plot the graph with edges or not, upon input
        :param path : where save the plot
        :param labels: if True each viable path (edge) will have a lebel with the weight; Otherwise no labels on edges (default=False)
        :param edges: if True the viable paths/edges will be printed; otherwise only the targets and depots are plotted (default=False)
        """
        plot_builder = AoIPlotManager(self, labels=labels, edges=edges)
        plot_builder.save(path)

    def node_index(self, coords: tuple):
        """ return the node index in the internal graph structures

            :param coords : the coordinates of target or depot. e.g., (x1, y1)
            :return an int that correspend to the graph node index
        """
        if coords in self.target_points:
            return self.target_points.index(coords)
        elif coords in self.depots:
            return self.n_targets + self.depots.index(coords)
        else:
            raise ValueError('The input coords does not exist (both in depots and targets)')


# A tour made of edges
class Tour:

    def __init__(self, aoi: AoI, edges_w_coords: list):
        """ build a tour with a given set of edges

        :param aoi: the input AoI where the tour is employed
        :param edges_w_coords: a list of tuple e.g., [(node1, node2), (node2, node3), ...]
                    where each node is represented as coords (x1,y1)
        """
        self.aoi = aoi
        self.edges_w_coords = edges_w_coords

        # set depots and edges with indexex of graph structure
        self.targets_indexes = []  # must be ordered
        self.targets_coords = []  # must be ordered

        self.edges_w_indexes = []
        for ec in self.edges_w_coords:
            if ec[0] in self.aoi.depots:
                self.depot_index = self.aoi.node_index(ec[0])
                self.depot_coord = ec[0]
            else:
                self.targets_indexes.append(self.aoi.node_index(ec[0]))
                self.targets_coords.append(ec[0])

            edge_w_indexes = (self.aoi.node_index(ec[0]), self.aoi.node_index(ec[1]))
            self.edges_w_indexes.append(edge_w_indexes)

        self.len_tour_meters = None
        self.hovering_time_tour_seconds = None

        self.nnodes = len(self.targets_coords) + 1  # plus depot

    def time_tour(self, speed: float):
        """
            speed : m/s speed of drones to computer the required time to complete the tour
                including hovering
        """
        time_cost = self.len_tour() / speed
        return time_cost + self.hovering_time_tour()

    def len_tour(self):
        """
            return the len of the tour (meters)
        """
        if self.len_tour_meters is None:
            tlen = 0
            for edge in self.edges_w_indexes:
                tlen += self.aoi.graph.edges[edge[0], edge[1]]['weight']
            self.len_tour_meters = tlen

        return self.len_tour_meters

    def hovering_time_tour(self):
        """
            return the total hovevering time for the tour (seconds)
        """
        if self.hovering_time_tour_seconds is None:
            hlen = 0
            for edge in self.edges_w_indexes:
                hlen += self.aoi.graph.nodes[edge[1]]['weight']
            self.hovering_time_tour_seconds = hlen

        return self.hovering_time_tour_seconds

    @classmethod
    def from_coordinates(cls, aoi: AoI, edges_w_coords: list):
        """ build a tour with a given set of edges

        :param aoi: the input AoI where the tour is employed
        :param edges_w_coords: a list of tuple e.g., [(node1, node2), (node2, node3), ...]
                    where each node is represented as coords (x1,y1)
        """
        return cls(aoi, edges_w_coords)

    @classmethod
    def from_graph_indexes(cls, aoi: AoI, edges_w_indexes: list):
        """ build a tour with a given set of edges

        :param aoi: the input AoI where the tour is employed
        :param edges_w_indexes: a list of tuple e.g., [(node1, node2), (node2, node3), ...]
                    where each node is the index of graph internal structure node

                node1 is in self.aoi.graph.nodes
        """
        return cls(aoi, [(aoi.graph.nodes[x[0]]["pos"], aoi.graph.nodes[x[1]]["pos"])
                            for x in edges_w_indexes])

    def inspection_times(self, speed: float):
        """ return the inspection times for each target,
            considering the input speed of a drone

            return a list of inspection times (ordered)
        """
        times = []
        time_cost = 0
        for i in range(len(self.edges_w_indexes) - 1):  # last edge don't partecipates
            edge = self.edges_w_indexes[i]
            time_cost += (self.aoi.graph.edges[edge[0], edge[1]]['weight'] / speed
                          + self.aoi.graph.nodes[edge[1]]["weight"])
            times.append(time_cost)
        return times

    def __str__(self):
        out = "Tour w depot: "
        out += str(self.depot_coord) + ", nodes: "
        out += str(self.targets_coords)
        out += "\n Tour edges: " + str(self.edges_w_coords)
        return out

    def __repr__(self):
        return "Tour w depot: " + str(self.depot_coord) + ", nnodes: " + str(len(self.edges_w_coords))

    def __eq__(self, other):
        if not isinstance(other, Tour):
            return False
        else:
            return self.edges_w_indexes == other.edges_w_indexes


""" a utility class to represent a drone """
class Drone():
    obj_id = 0

    def __init__(self, autonomy : int, speed : float):
        """
        :param autonomy: the autonomy in seconds for the drones
        :param speed: the average speed of drone (m/s). This versione does not support acceleration/deceleration
        """
        self.__id()
        self.autonomy = autonomy
        self.speed = speed

    def __id(self):
        """ set a unique for the obj """
        self.id = Drone.obj_id
        Drone.obj_id += 1

    def __str__(self):
        out = "Drone: " + str(self.id)
        return out

    def __repr__(self):
        return "Drone: " + str(self.id)

    def __eq__(self, other):
        if not isinstance(other, Drone):
            return False
        else:
            return other.id == self.id

    def __hash__(self):
        return hash(self.id)

"""
A utility class to create the multi round solution round by round, drone by drone, iteratively 
"""
class MultiRoundSolutionBuilder:

    def __init__(self, aoi : AoI):
        """

        :param aoi: the input area of interest with targets
        """
        self.aoi = aoi
        self.drone_and_tours = {}  # internal structure that save for each drone its tours ordered (from first to last round)
        self.drones = set()

    def add_drone(self, drone : Drone):
        """
        Add a drone in the solution without tours

        :param drone: the input drone to add
        :return: the object itself
        """
        assert drone not in self.drone_and_tours, "drone already exist in the multi-round solution"
        self.drones.add(drone)
        self.drone_and_tours[drone] = []  # list of tours, empty at the beginning
        return self

    def append_tour(self, drone: Drone, tour : Tour):
        """
        Add a tour to the drone; append it in next round of the solution

        :param drone: the input drone to add
        :param tour: the next tour of the drone
        :return: the object itself
        """
        assert drone in self.drone_and_tours, "drone does not exist in the multi-round solution"
        self.drone_and_tours[drone].append(tour)  # list of tours, empty at the beginning
        return self

    def add_drone_with_tours(self, drone: Drone, tours : list):
        """
        Add a tour to the drone; append it in next round of the solution

        :param drone: the input drone to add
        :param tours: all the ordered tours of the drone
        :return: the object itself
        """
        assert drone not in self.drone_and_tours, "drone already exist in the multi-round solution"
        self.drones.add(drone)
        self.drone_and_tours[drone] = list(tours)  # list of tours, empty at the beginning
        return self

    def build(self):
        """ effectively build the MultiRoundSolution object """
        return MultiRoundSolution(self)


"""
A utility class to store/save and get metrics of a multi round solution over a given AoI 
"""
class MultiRoundSolution:

    def __init__(self, builder : MultiRoundSolutionBuilder):
        """
        Do not use this method, only internal use. Please refer to builder.

        :param builder: the builder with all data, tours and depot.
        """
        self.aoi = builder.aoi
        self.drones = builder.drones
        self.drone_and_tours = builder.drone_and_tours
        self.ndrones = len(self.drone_and_tours.keys())  # the number of drone used in the solution
        self.max_rounds = 0

        self.covered_graph_nodes = set()  # all the indexes of the nodes graph that are covered by some tour/drone
        self.covered_graph_targets = set()  # all the indexes of the targets that are covered by some tour/drone

        for drone in self.drone_and_tours.keys():
            self.max_rounds = max(self.max_rounds, len(self.drone_and_tours[drone]))
            for tour in self.drone_and_tours[drone]:
                for e0, e1 in tour.edges_w_indexes:
                    self.covered_graph_nodes.add(e0)
                    coords_e0 = self.aoi.graph.nodes[e0]["pos"]
                    if coords_e0 not in self.aoi.depots:  # skip depots
                        self.covered_graph_targets.add(coords_e0)

        self.__targets_covered_on_each_round()

    def __targets_covered_on_each_round(self):
        """ compute and fill:
                self.targets_covered_on_each_round
        """
        self.targets_covered_on_each_round = dict()
        already_visited = set()
        for round in range(self.max_rounds):
            self.targets_covered_on_each_round[round] = set()
            for drone in self.drone_and_tours.keys():
                if round >= len(self.drone_and_tours[drone]):  # the drone does not use all the rounds
                    continue
                for t in self.drone_and_tours[drone][round].targets_coords:
                    if t not in already_visited:
                        self.targets_covered_on_each_round[round].add(t)
                        already_visited.add(t)

    def __plot(self, title=None):
        """ plot the multi-round solution """
        tours = []
        tour_labels = []
        for drone in self.drone_and_tours.keys():
            round_counter = 0
            for tour in self.drone_and_tours[drone]:
                round_counter += 1
                tours.append(tour)
                tour_labels.append(str(drone) + " Round " + str(round_counter))

        if title is None:
            plotter = ToursPlotManager(self.aoi, tours, labels=True, tour_labels=tour_labels)
        else:
            plotter = ToursPlotManager(self.aoi, tours, title=title, labels=True, tour_labels=tour_labels)
        return plotter

    def plot(self, title=None):
        """ plot the multi-round solution

        :param title: the title of the plot
        """
        plotter = self.__plot(title)
        plotter.show()

    def save_plot(self, path, title=None):
        """
        save the plot of the multi-round solution

        :param path: where save the plot
        :param title: the title of the plot
        """
        plotter = self.__plot(title)
        plotter.save(path)

    def coverage_score(self):
        """ return the number of covered target points """
        return len(self.covered_graph_targets)

    def plot_cumulative_coverage_for_round(self, title="Cumulative Coverage"):
        """ plot the cumulative round coverage """
        x = []
        y = []
        all_covered_up_to_round = 0
        for round in range(self.max_rounds):
            cov_on_that_round = len(self.targets_covered_on_each_round[round])
            all_covered_up_to_round += cov_on_that_round
            # data for plot
            x.append(round + 1)
            y.append(all_covered_up_to_round)

        print(title, "cum-coverage for rounds:")
        print(x)
        print(y)
        print("----")
        plt.plot(x, y)
        plt.ylabel("Cumulative Coverage")
        plt.xlabel("Round i-th")
        plt.title(title)
        plt.show()

