"""
Owner: Andrea Coletta
Version v1.0
Release code for TMC : 10.1109/TMC.2020.2994529 and its ICDCS conference version : 10.1109/ICDCS.2019.00209

Please cite these works in case of use.

This file contains the approximation algorithms --- Greedy-And-Prune Algorithms (GaP) ---, i.e., TC-GaP and AC-GaP.
See above papers per specs of the two algorithms.

They take input AoI, tours and max number of rounds. Return a multi round solution.
"""

from src.entities.trajenties import AoI, Tour, MultiRoundSolutionBuilder, MultiRoundSolution
from src.util import utility
from abc import ABCMeta, abstractmethod



# """ Constructor for the Greedy-And-Prune Algorithm (GaP), input the graph that
#    must be covered, the list of uavs that will be used, for each
#    uav the respective root will be in the roots list and
#    the feasible tours in the uavs_tours: the i-th uav (uavs[i])
#    will have uavs_tours[i] feasbile tours and the root in roots[i].
#    max_rounds indicates the max number of rounds for the graph cover.
#
#    the tours in uavs_tours[i] must be instance of class utility.Tour
#    the uavs must be istance of class utility.Drone
# """

# -----------------------------------------------------------------------------
#
# Abstract Class Model for GaP algorithms
#
# -----------------------------------------------------------------------------

class AbstractGreedyAndPrune():
    ''' An abstract model for GaP path Planning algorithm '''
    __metaclass__ = ABCMeta

    def __init__(self, aoi: AoI, uavs_tours: dict, max_rounds: int, debug: bool = True):
        """
        Constructor for the GaP Path Coverage Algorihms (TC-GaP and AC-GaP).

        :param aoi: the input area of interest with targets and depots
        :param uavs_tours: a dictionary that maps available drones to their available tours. e.g. {drone1 : [tour1, tour2, ...], drone2 : [tour1, tour2, ..], ..}
                                Each key drone should an object -> drone : trajenties.Drone
                                Each tour of a drone should be an object -> tour : trajenties.Tour
                            We assume that each tour is feasble for the given drone.
                            Note: each drone should leave always from same depot! All the tours from same drone should have same depot, the drone cannot exchange depots!
        :param max_rounds: the maximum number of rounds to perform (max number of multi trips)
        :param debug: whether print or not debug stuff (iteration etc..) (default True).
        """
        self.aoi = aoi
        self.max_rounds = max_rounds
        self.debug = debug

        # other help variables
        self.graph = aoi.graph
        self.nnodes = self.aoi.n_targets  # len(aoi.graph.nodes())  # all nodes in the graphs, it includes depots
        self.uavs = list(uavs_tours.keys())
        self.nuavs = len(self.uavs)

        # uav_tours should use integer index for the model
        self.uavs_tours = {i: uavs_tours[self.uavs[i]] for i in range(self.nuavs)}

        # check depots (each drone should leave always from same depot)
        self.__check_depots()

        # all nodes in input tours
        self.reachable_points = self.__reachable_points()

    def __check_depots(self):
        """ asserts that each drone leaves always from same depots:
            all its associated tours are referred to same depot
        """
        for drone in self.uavs_tours.keys():
            depots = set()
            for tour in self.uavs_tours[drone]:
                depots.add(tour.depot_coord)
            assert len(depots) == 1, "A drone cannot leave and start from multiple depots!"

    def __reachable_points(self):
        """  return a set of all points
             that are reachable by the input tours
        """
        r_points = set()
        for u in range(self.nuavs):
            u_uav_tours = self.uavs_tours[u]
            for t in u_uav_tours:
                r_points |= set(t.targets_indexes)
        return r_points

    @abstractmethod
    def local_optimal_choice(self, visited_points : set, residual_ntours_to_assign : dict):
        """

        :param visited_points: the already visited points
        :param residual_ntours_to_assign: a disctionary {drone : number of residual tours to assign}
        :return:   a tuple (index_uav, index_tour) that is optimal greedy choice for the step
        """
        pass

    def greedy_stop_condition(self, visited_points : set, tour_to_assign : int) -> bool:
        """ stop condition of greedy algorithm

        :param visited_points: the already visited points
        :param tour_to_assign: the number of remaining tours to assign
        :return: whether the algorithm must stop or not
        """
        return tour_to_assign == 0 or len(self.reachable_points - visited_points) == 0

    def __pruning(self, mr_solution: MultiRoundSolution) -> MultiRoundSolution:
        """
        TODO: i'm working on it
        :param mr_solution: the computed solution from the greedy phase
        :return: the solution with purning of redundant tagerts/visits
        """
        return utility.pruning_multiroundsolution(mr_solution)

    def solution(self) -> MultiRoundSolution:
        """ run the algorithm and build a multi-round solution until the stop condition is reached """
        # multi-round solution to build
        mrs_builder = MultiRoundSolutionBuilder(self.aoi)
        for uav in self.uavs:
            mrs_builder.add_drone(uav)

        # counters and set of visited points
        residual_ntours_to_assign = {i : self.max_rounds for i in range(self.nuavs)}
        tour_to_assign = self.max_rounds * self.nuavs
        visited_points = set()

        while not self.greedy_stop_condition(visited_points, tour_to_assign):
            itd_uav, ind_tour = self.local_optimal_choice(visited_points, residual_ntours_to_assign)
            residual_ntours_to_assign[itd_uav] -= 1
            tour_to_assign -= 1
            opt_tour = self.uavs_tours[itd_uav][ind_tour]
            visited_points |= set(opt_tour.targets_indexes)  # update visited points
            mrs_builder.append_tour(self.uavs[itd_uav], opt_tour)

        return self.__pruning(mrs_builder.build())


# -------------------------------------------------------------------
#
# Cumulative Greedy Coverage Class for path coverage
#
# -------------------------------------------------------------------
class CumulativeGreedyCoverage(AbstractGreedyAndPrune):
    ''' Cumulative Greedy Coverage class for cumulative coverage
        path based problem with drones and multiple tours/rounds
    '''

    def local_optimal_choice(self, visited_points: set, residual_ntours_to_assign: dict):
        """

        :param visited_points: the already visited points
        :param residual_ntours_to_assign: a disctionary {drone : number of residual tours to assign}
        :return:   a tuple (index_uav, index_tour) that is optimal greedy choice for the step
        """
        choice_dict = {}
        for ind_uav in range(self.nuavs):
            uav_residual_rounds = residual_ntours_to_assign[ind_uav]
            if uav_residual_rounds > 0:
                uav_tours = self.uavs_tours[ind_uav]
                for ind_tour in range(len(uav_tours)):
                    tour = uav_tours[ind_tour]
                    quality_tour = self.evaluate_tour(tour, uav_residual_rounds, visited_points)
                    choice_dict[quality_tour] = (ind_uav, ind_tour)

        best_value = max(choice_dict, key=int)
        return choice_dict[best_value]

    def evaluate_tour(self, tour : Tour, round_count : int, visited_points : set):
        """ measure of quality of round,
            in terms of cumulative coverage
        """
        new_points = (set(tour.targets_indexes) - visited_points)
        return round_count * len(new_points)

# -------------------------------------------------------------------

#
# Total Greedy Coverage Class for path coverage
#
# -------------------------------------------------------------------
class TotalGreedyCoverage(AbstractGreedyAndPrune):
    ''' Total Greedy Coverage class for simple coverage
        path based problem with drones and multiple tours/rounds
    '''


    def local_optimal_choice(self, visited_points: set, residual_ntours_to_assign: dict):
        """

        :param visited_points: the already visited points
        :param residual_ntours_to_assign: a disctionary {drone : number of residual tours to assign}
        :return:   a tuple (index_uav, index_tour) that is optimal greedy choice for the step
        """
        choice_dict = {}
        for ind_uav in range(self.nuavs):
            uav_residual_rounds = residual_ntours_to_assign[ind_uav]
            if uav_residual_rounds > 0:
                uav_tours = self.uavs_tours[ind_uav]
                for ind_tour in range(len(uav_tours)):
                    tour = uav_tours[ind_tour]
                    q_tour = self.evaluate_tour(tour, visited_points)
                    choice_dict[q_tour] = (ind_uav, ind_tour)

        best_value = max(choice_dict, key=int)
        return choice_dict[best_value]

    def evaluate_tour(self, tour : Tour, visited_points : set):
        """ measure of quality of round,
            in terms of cumulative coverage
        """
        new_points = (set(tour.targets_indexes) - visited_points)
        return len(new_points)

