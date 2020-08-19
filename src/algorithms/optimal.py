"""
Owner: Andrea Coletta
Version v1.0
Release code for TMC : 10.1109/TMC.2020.2994529 and its ICDCS conference version : 10.1109/ICDCS.2019.00209

Please cite these works in case of use.

This file contains the optimal models, i.e., TC-OPT and AC-OPT.
See above papers per specs of the two models.

They take input AoI, tours and max number of rounds. Return a multi round solution.
"""
from src.entities.trajenties import AoI, Tour, MultiRoundSolutionBuilder

from gurobipy import *
from abc import ABCMeta, abstractmethod
from deprecated import deprecated

import networkx as nx
import gurobipy

# """ Constructor for the Path Coverage, input the graph that
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
# Abstract Class Model for path based coverage
#
# -----------------------------------------------------------------------------
class AbstractCoverageModel():
    ''' A gurobi abstract model for coverage path based path Planning problem with drones '''
    __metaclass__ = ABCMeta

    def __init__(self, aoi: AoI, uavs_tours: dict, max_rounds: int, debug: bool = True):
        """
        Constructor for the Optimal Model of the Path Coverage (TC-OPT and AC-OPT).

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

        # optimization model
        self.model = None

    def __check_depots(self):
        """ asserts that each drone leaves always from same depots:
            all its associated tours are referred to same depot
        """
        for drone in self.uavs_tours.keys():
            depots = set()
            for tour in self.uavs_tours[drone]:
                depots.add(tour.depot_coord)
            assert len(depots) == 1, "A drone cannot leave and start from multiple depots!"

    @abstractmethod
    def build(self):
        """
         build the optimization model with constraints, variables and obj function
        """
        pass

    def add_vars(self):
        ''' add all the required variables to the model '''
        traj_vars = self.trajectory_vars()  # z.p.u(n)
        cov_vars = self.coverage_point_vars()  # delta.i(n)
        return traj_vars, cov_vars

    def trajectory_vars(self):
        ''' z_p^u(n) variables - trajectories choices

            p -> tour
            u -> drone/uav
            n -> number of round
        '''
        return self.model.addVars(  # z_p^u(n)
            [(u, p, n)
             for u in range(self.nuavs)
             for p in range(len(self.uavs_tours[u]))
             for n in range(self.max_rounds)],
            vtype=GRB.BINARY,
            name="z.p.u(n)")

    def coverage_point_vars(self):
        ''' deltapiccolo_i(n) variables - cov point var

            i -> target to visit
            n -> round of visit
        '''
        return self.model.addVars(
            [(i, n)
             for i in range(self.nnodes)  # delta_i(n)
             for n in range(self.max_rounds)],
            vtype=GRB.BINARY,
            name="delta.i(n)")

    def add_base_constrs(self, traj_vars, cov_vars):
        ''' add all the required costraints to the model '''
        self.round_target_cov_constr(traj_vars, cov_vars)  # b
        self.oneround_onetour_constr(traj_vars)  # c
        # self.exclusive_target_cov_constr(cov_vars)  #g  NOTE: not included

    def round_target_cov_constr(self, traj_vars, cov_var):
        """ impose that a point is visited only
            from one drone/tour at each round 
        """
        self.model.addConstrs(
            cov_var[i, n] ==
            quicksum([traj_vars.sum(u, p, n)
                      for u in range(self.nuavs)
                      for p in range(len(self.uavs_tours[u]))
                      if i in self.uavs_tours[u][p].targets_indexes
                      ])
            for i in range(self.nnodes)
            for n in range(self.max_rounds)
        )

    def oneround_onetour_constr(self, traj_vars):
        """ impose only a tour for each drone in the same round """
        self.model.addConstrs(
            traj_vars.sum(u, '*', n) <= 1
            for u in range(self.nuavs)
            for n in range(self.max_rounds)
        )

    @deprecated
    def exclusive_target_cov_constr(self, cov_vars):
        """ ensures that a target is covered in only
            one of the rounds
        """
        self.model.addConstrs(cov_vars.sum(i, '*') == 1
                              for i in range(self.nnodes)
                              )

    def console_debug(self):
        if not self.debug:
            self.model.Params.outputFlag = 0

    def optimize(self):
        self.console_debug()
        self.model.optimize()
        if self.model.getAttr('Status') == GRB.OPTIMAL:
            self.extract_solution()

    def extract_solution(self):
        ''' extract the values from variables 
            and the tours produced by the optimization 
        '''
        self.strsolution = ""
        for variable in self.model.getVars():
            self.strsolution += (str(variable.varName)
                                 + " - "
                                 + str(variable.x)
                                 + "\n")  # x -> results of optimization
        self.strsolution += "Objetive function value: "
        self.strsolution += str(self.model.objVal)

        mrs_builder = MultiRoundSolutionBuilder(self.aoi)
        for iu in range(self.nuavs):
            mrs_builder.add_drone(self.uavs[iu])
            for n in range(self.max_rounds):  # round
                tour = Tour(self.aoi, [])  # in case the solution do not use this round, we place an empty tour
                for p in range(len(self.uavs_tours[iu])):
                    if (self.model.getVarByName('z.p.u(n)[%d,%d,%d]'
                                                % (iu, p, n)).X >= 0.5):
                        tour = self.uavs_tours[iu][p]
                mrs_builder.append_tour(self.uavs[iu], tour)

        self.solution = mrs_builder.build()


class CumulativeCoverageModel(AbstractCoverageModel):
    ''' A gurobi model for cumulative Coverage (AC-Opt) Model path based with drones '''

    def build(self):
        self.model = Model("cumulative_coverage")
        traj_vars, cov_vars, cum_cov_vars = self.add_vars()
        self.add_constrs(traj_vars, cov_vars, cum_cov_vars)
        self.objective_function(cum_cov_vars)

    def add_vars(self):
        ''' add all the required variables to the model '''
        traj_vars, cov_vars = super(CumulativeCoverageModel,
                                    self).add_vars()
        cum_cov_vars = self.cumulative_cov_vars()  # DELTA.i(n)
        return traj_vars, cov_vars, cum_cov_vars

    def cumulative_cov_vars(self):
        ''' deltagrande_i(n) variables - cumulative coverage var '''
        return self.model.addVars(  # DELTA_i(n)
            [(i, n)
             for i in range(self.nnodes)
             for n in range(self.max_rounds)],
            vtype=GRB.BINARY,
            name="DELTA_i(n)")

    def add_constrs(self, traj_vars, cov_vars, cum_cov_vars):
        ''' add all the required costraints to the model
        '''
        super(CumulativeCoverageModel,
              self).add_base_constrs(traj_vars, cov_vars)
        self.cumulative_cov_constr(cov_vars, cum_cov_vars)  # constr d

    def cumulative_cov_constr(self, cov_vars, cum_cov_vars):
        """ variable for cumulative coverage for each point i
            and for each round
        """
        for i in range(self.nnodes):
            for n in range(self.max_rounds):
                cost = quicksum([cov_vars[i, k] for k in range(n + 1)])
                self.model.addConstr(cum_cov_vars[i, n] <= cost,
                                     "cum_cov_constr")

    def objective_function(self, cumulative_cov_vars):
        """ objective function of opt model """
        self.model.setObjective(cumulative_cov_vars.sum('*', '*'),
                                GRB.MAXIMIZE)


class TotalCoverageModel(AbstractCoverageModel):
    ''' A gurobi model for Total Coverage (TC-Opt) Model path based with drones '''

    def build(self):
        self.model = Model("total_coverage")
        traj_vars, cov_vars, tot_cov_vars = self.add_vars()
        self.add_constrs(traj_vars, cov_vars, tot_cov_vars)
        self.objective_function(tot_cov_vars)

    def add_vars(self):
        ''' add all the required variables to the model '''
        traj_vars, cov_vars = super(TotalCoverageModel,
                                    self).add_vars()
        tot_cov_vars = self.total_cov_vars()  # DELTA.i(n)
        return traj_vars, cov_vars, tot_cov_vars

    def total_cov_vars(self):
        ''' deltagrande_i variables - simple total coverage var 
            indicated wherever the point i is covered or not
            in at least one round    
        '''
        return self.model.addVars(  # DELTA_i(n)
            [i for i in range(self.nnodes)],
            vtype=GRB.BINARY,
            name="DELTA_i")

    def add_constrs(self, traj_vars, cov_vars, tot_cov_vars):
        ''' add all the required costraints to the model '''
        super(TotalCoverageModel,
              self).add_base_constrs(traj_vars, cov_vars, )
        self.total_cov_constr(cov_vars, tot_cov_vars)  # d

    def total_cov_constr(self, cov_vars, tot_cov_vars):
        """ variable for cumulative coverage for each point i
            and for each round
        """
        for i in range(self.nnodes):
            cost = quicksum([cov_vars[i, k] for k in range(self.max_rounds)])
            self.model.addConstr(tot_cov_vars[i] <= cost,
                                 "tot_cov_constr")

    def objective_function(self, tot_cov_vars):
        """ objective function of opt model """
        self.model.setObjective(tot_cov_vars.sum('*'),
                                GRB.MAXIMIZE)
