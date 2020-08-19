"""
Owner: Andrea Coletta
Version v1.0
Release code for TMC : 10.1109/TMC.2020.2994529 and its ICDCS conference version : 10.1109/ICDCS.2019.00209

Please cite these works in case of use.

This file contains code to build a set of candidates trajectories

In particular, Algorithm DroneTrajGeneration (Alg.2) starts from a TSP to build a set of possible trajectories.
For more details on the pseudo-code please refer to "Algorithm 2: Drone-trajectory generation" (TMC : 10.1109/TMC.2020.2994529).

"""

from src.entities.trajenties import AoI, Drone, Tour
from src.util.utility import Christofides

import networkx as nx


class DroneTrajGeneration():
    """
    Algorithm 2 TMC. It generates a set of feasible trajectories for an input drone
    """

    def __init__(self, aoi: AoI):
        """
        :param aoi: the input aoi with targets and depots
        """
        self.aoi = aoi

    def compute_trajectories(self, drone: Drone, depot_coords: tuple):
        """
        Actually compute the set of feasible trajectories accordina drone speed and energy.

        :param drone: the drone for the feasible trajectories set
        :param depot_coords: the depot coordinates for the input drone, where the trajectories start and end. e.g., (x1, y1)
        :return: a list of tours. e.g., [tour1, tour2, ..., tourN] starting and ending at input depot. (tour : src.entities.Tour)
        """
        assert depot_coords in self.aoi.depots, "Depot should be included in the given AoI"
        depot_index = self.aoi.node_index(depot_coords)

        # remove unused and unreachable nodes
        graph = self.__remove_nodes(self.aoi.graph, depot_coords, drone)
        assert graph.number_of_nodes() > 1, "Drone {} has not enough energy to visit any node".format(drone)

        # compute the initial TSP
        tsp_tour = Christofides.compute(graph, depot_index)
        tsp_nodes = [x[0] for x in tsp_tour]  # ordered visited nodes by TSP
        assert len(tsp_nodes) == graph.number_of_nodes()

        nnodes = len(tsp_nodes)
        output_tours = []
        for i in range(1, nnodes):  # index of first nodes of sub-tour
            output_tours.extend(self.__compute_subtours(i, tsp_nodes, drone, depot_index))

        return output_tours

    def __compute_subtours(self, tsp_node_index: int, tsp_nodes: list, drone: Drone, depot_index: int) -> list:
        """

        :param tsp_node_index: the current node, the first node to visit in the tour -> tsp_nodes[tsp_node_index]
        :param tsp_nodes: the ordered nodes of the tsp tour
        :param drone: the drone for the feasible trajectories set
        :param depot_index: the index of the drone (index of graph)
        :return: a list of tour : src.entities.trajentities.Tour from tsp_node_index to the last nodes visitable according drone energy
        """
        out_tours = []

        # initial tour
        current_subtour = [(depot_index, tsp_nodes[tsp_node_index])]  # by construction this must be visitable
        return_edge = (tsp_nodes[tsp_node_index], depot_index)
        tour = Tour.from_graph_indexes(self.aoi, current_subtour + [return_edge])
        assert tour.time_tour(
            drone.speed) < drone.autonomy, "This is a bug in the multipath build, the drone should have enough energy " \
                                           "to visit this point and come back to depot! "
        out_tours.append(tour)  # add this first slice of tour

        for i in range(tsp_node_index + 1, len(tsp_nodes)):  # last nodes in sub-tour
            current_subtour += [(tsp_nodes[i - 1], tsp_nodes[i])]
            return_edge = (tsp_nodes[i], depot_index)
            tour = Tour.from_graph_indexes(self.aoi, current_subtour + [return_edge])
            if tour.time_tour(drone.speed) < drone.autonomy:
                out_tours.append(tour)  # add this first slice of tour
            else:
                break  # bigger sub-tours will have cost > autonomy by triangle inequality

        return out_tours

    def __remove_nodes(self, graph: nx.Graph, depot_coords: tuple, drone: Drone) -> nx.Graph:
        """ remove unreachble nodes:
                targets too far (the drone has not enough energy)
                depots different from the input one (depots != depots_coords)

        :param graph: the input graph to remove nodes and depots
        :param depot_coords: the coordaintes of the unique depot to use
        :param drone: the drone for the feasible trajectories set
        :return: a copy of the input graph without some nodes
        """
        graph = graph.copy()
        node_indexes_to_remove = []

        # remove other depots from the graph as we don't want to visit them
        for __depot_coords in self.aoi.depots:
            if __depot_coords != depot_coords:
                node_indexes_to_remove.append(self.aoi.node_index(__depot_coords))

        depot_index = self.aoi.node_index(depot_coords)
        # remove un-reachble nodes for the input drone
        for n in graph.nodes():
            if n == depot_index:
                continue
            if graph.nodes[n]["weight"] + (2 * graph.edges[depot_index, n]["weight"] / drone.speed) > drone.autonomy:
                node_indexes_to_remove.append(n)  # remove non reachable nodes (round trip time is too much)

        graph.remove_nodes_from(node_indexes_to_remove)
        return graph
