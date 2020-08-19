"""
Owner: Andrea Coletta
Version v1.0
Release code for TMC : 10.1109/TMC.2020.2994529 and its ICDCS conference version : 10.1109/ICDCS.2019.00209

Please cite these works in case of use.


File content:
This file contains all the main utility functions needed to run the AC-GaP and AC-TCP code.
"""

import networkx as nx
import numpy as np

from src.entities.trajenties import Tour, AoI, MultiRoundSolution, MultiRoundSolutionBuilder


def pruning_multiroundsolution(mrs : MultiRoundSolution) -> MultiRoundSolution:
    """

    :param mrs: the inptu multi round solution to prune (remove redundant visits)
    :return: a MultiRoundSolution without redundant targets
    """
    # new pruned solution
    pruned_solution = MultiRoundSolutionBuilder(mrs.aoi)
    for drone in mrs.drone_and_tours.keys():
        pruned_solution.add_drone(drone)

    # prune and build new solution
    already_covered_nodes = set()
    for round in range(mrs.max_rounds):
        for drone in mrs.drone_and_tours.keys():
            if round >= len(mrs.drone_and_tours[drone]):  # the drone does not use all the rounds
                continue

            actual_tour = mrs.drone_and_tours[drone][round]
            new_tour_nodes = [actual_tour.depot_index]
            for node in actual_tour.targets_indexes:
                if node not in already_covered_nodes:
                    new_tour_nodes.append(node)

            already_covered_nodes |= set(actual_tour.targets_indexes)

            if len(new_tour_nodes) > 1:  # otherwise we removed all the nodes (all already visit)
                pruned_solution.append_tour(drone,
                                            Tour.from_graph_indexes(mrs.aoi,
                                                                    build_tour_from_ordered_nodes(new_tour_nodes)))

    return pruned_solution.build()


def build_random_aoi(width_area:int, height_area :int, n_target :int, n_depots :int, hovering_time :int, seed:int=None) -> AoI:
    """

    :param width_area: the size of area of interset where put targets and depots
    :param height_area: the size of area of interset where put targets and depots
    :param n_target: the number of targets
    :param n_depots:  the number of depots
    :param hovering_time: the hovering time for each node (aassuming they are uniform) in seconds
    :param seed: the seed, if any, to use as random generator seed
    :return:  the area of itnereste : AoI
    """
    if seed is not None:
        np.random.seed(seed)

    # build area and targets
    random_target_points = [(np.random.randint(0, width_area),
                             np.random.randint(0, height_area))
                            for i in range(n_target)]
    random_depots_points_on_x = [(np.random.randint(0, width_area),
                                  100)  # fixed y
                                 for i in range(n_depots)]
    return AoI(random_depots_points_on_x, random_target_points, width_area, height_area, node_hovering_time=5)


def build_tour_from_ordered_nodes(nodes: list):
    """

    :param nodes: a list of nodes
    :return: a tour made of [(node[0], node[1]), (node[1], node[2]), .... , (node[n], node[0])]
    """
    tour = []
    l_nodes = len(nodes)
    for t_i in range(l_nodes):
        if t_i == l_nodes - 1:
            tour += [(nodes[t_i], nodes[0])]
        else:
            tour += [(nodes[t_i], nodes[t_i + 1])]
    return tour


class Christofides():
    """
    A class for compute the approximated solution of TSP by Christofides
    """

    @classmethod
    def compute(cls, graph : nx.Graph, depot_index: int):
        """

        :param graph: the graph where compute the TSP tour
        :param: depot_index : the index of node which is referred as depot (start and end of the tour)
        :return: the tsp Tour [e1,e2,...,en] with e1 with indexes of graph
        """
        graph = graph.copy()

        # first step -> MST of graph
        mst = nx.minimum_spanning_tree(graph)

        # even
        odd_nodes = Christofides.odd_nodes(mst)

        # induced subgraph of odd nodes
        odd_graph = graph.subgraph(odd_nodes).copy()

        # minimum weighted matching
        perfect_match = Christofides.min_weight_matching(odd_graph)

        # build Eulerian Graph: mst + perfect match
        eu_graph = nx.MultiGraph()
        for e0, e1 in list(mst.edges) + list(perfect_match):
            eu_graph.add_node(e0, pos=graph.nodes[e0]["pos"])
            eu_graph.add_node(e1, pos=graph.nodes[e1]["pos"])
            eu_graph.add_edge(e0, e1, weight=graph.edges[e0, e1]["weight"])

        # Assert a eulerian graph
        assert nx.is_eulerian(eu_graph), "The mst + perfect matching of Christofides -> not an eulerian graph "

        # eulerian tour
        eu_tour = list(nx.eulerian_circuit(eu_graph, source=depot_index, keys=False))

        # shortcut tour to have a 1.5-TSP
        tsp_tour = Christofides.shorted_tour(eu_tour)
        return tsp_tour

    @classmethod
    def min_weight_matching(cls, graph: nx.Graph):
        """
        :param graph: NetworkX Graph -  the graph where compute the minium weight matching .
        :return: a list of edges of the perfect match
        """
        temp_graph = graph.copy()
        # reverse weight to use built-in function of networkx -> max_weight_matching
        for edge in temp_graph.edges():
            temp_graph.edges[edge[0], edge[1]]["weight"] = 1.0 / temp_graph.edges[edge[0], edge[1]]["weight"]

        # list of edges for a perfect matching graph
        return nx.max_weight_matching(temp_graph)

    @classmethod
    def odd_nodes(cls, mst: nx.Graph):
        """

        :param mst: NetworkX Graph -  A minimum spanning tree.
        :return:
        """
        odd_vert = []  # list containing vertices with odd degree
        for i in mst.nodes():
            if mst.degree(i) % 2 != 0:
                odd_vert.append(i)  # if the degree of the vertex is odd, then append it to odd_vert list
        return odd_vert

    @classmethod
    def shorted_tour(cls, eu_tour: list) -> list:
        """
        sub-routine for TSP-algorithm of Christofides. Takes input
        an eulerian tour and remove the nodes already visited in the tour
        and return an instance of tour [e1,e2,...,en].

        :param eu_tour: Takes input an eulerian tour [e1,e2,... ] edges.
        :return: the 1.5-TSP from the shortcut of eulerian tour
        """
        ordered_unique_nodes_in_tour = []
        for node0, node1 in eu_tour:
            if node0 not in ordered_unique_nodes_in_tour:
                ordered_unique_nodes_in_tour.append(node0)

        return build_tour_from_ordered_nodes(ordered_unique_nodes_in_tour)

