"""
Owner: Andrea Coletta
Version v1.0
Release code for TMC : 10.1109/TMC.2020.2994529 and its ICDCS conference version : 10.1109/ICDCS.2019.00209

Please cite these works in case of use.

This files is intended to have some test case and example of code execution.
"""

from src.entities.trajenties import AoI, Tour, Drone, MultiRoundSolutionBuilder
from src.util import config, utility
from src.util.trajplot import ToursPlotManager
from src.algorithms.optimal import CumulativeCoverageModel, TotalCoverageModel
from src.algorithms.approxalg import CumulativeGreedyCoverage, TotalGreedyCoverage
from src.algorithms.trajbuilder import DroneTrajGeneration

from argparse import ArgumentParser

import numpy as np


def build_random_tour(aoi : AoI, depot_coords : tuple, ntargets_tour : int, seed:int=None):
    """
     build a random tour in the given AoI using the input depot
        the tour will have ntargets_tour as input.

    :param aoi: the input area of interest for the tour
    :param depot_coords: the depot coordinates of the tours, where tour start and end
    :param ntargets_tour: the number of targets that the tour will visit
    :param seed: the random seed to build the tour
    :return: the built tour : trajenties.Tour
    """
    assert depot_coords in aoi.depots, "The input depot does not exist in the area of interest!"

    if seed is not None:
        np.random.seed(seed)
    targets = aoi.target_points
    n_target = aoi.n_targets

    # ordered list of targets
    list_of_targets = [targets[np.random.randint(n_target)] for ti in range(ntargets_tour)]

    #build tour
    edges_tour = utility.build_tour_from_ordered_nodes([depot_coords] + list_of_targets)
    return Tour.from_coordinates(aoi, edges_tour)

def test1(plot=True):
    """ run the first code example (test)
        - it builds a AoI with random n-target points and x depots
        - plot the area with and w/o edges
        - save the area with and w/o edges
        - return the AoI, targets and depots

        plot : whether plot or not
    """
    seed = 50
    n_target = 20
    n_depots = 2
    width_area = 2000 # meters
    height_area = 2000  # meters
    np.random.seed(seed)

    #------------------------------------------------------------------------------------------------------

    random_target_points = [(np.random.randint(0, width_area),
                            np.random.randint(0, height_area))
                                for i in range(n_target)]
    random_depots_points_on_x = [(np.random.randint(0, width_area),
                                    100)  # fixed y
                                for i in range(n_depots)]
    aoi = AoI(random_depots_points_on_x, random_target_points, width_area, height_area, 5)

    if plot:
        aoi.plot()
        aoi.save(config.PATH_EXAMPLE_PLOTS + "test1_aoi_wo_edges.png")
        aoi.save(config.PATH_EXAMPLE_PLOTS + "test1_aoi_w_edges.png", edges=True)
        aoi.save(config.PATH_EXAMPLE_PLOTS + "test1_aoi_w_edges_and_labels.png", labels=True, edges=True)

    return aoi, random_target_points, random_depots_points_on_x


def test2(plot=True):
    """ run the first code example (test)
        - it builds a AoI with random n-target points and x depots
        - creates three input tours and plot them along the AoI
        - plot the area with the tours
        - save the tours with and w/o labels
        - return the tours and the area

        plot : whether plot or not
    """
    seed = 50
    n_target = 20
    n_depots = 2
    width_area = 2000 # meters
    height_area = 2000  # meters
    np.random.seed(seed)

    # ------------------------------------------------------------------------------------------------------

    # build area and targets
    random_target_points = [(np.random.randint(0, width_area),
                            np.random.randint(0, height_area))
                                for i in range(n_target)]
    random_depots_points_on_x = [(np.random.randint(0, width_area),
                                    100)  # fixed y
                                for i in range(n_depots)]
    aoi = AoI(random_depots_points_on_x, random_target_points, width_area, height_area, 5)

    #------------------------------------------------------------------------------------------------------

    # example 1 of build a tour using 2d euclidean coordinates
    #build a random tour of 4 edges
    choice_a_depot = random_depots_points_on_x[0]
    list_of_targets = [choice_a_depot, random_target_points[np.random.randint(n_target)],
                       random_target_points[np.random.randint(n_target)],
                       random_target_points[np.random.randint(n_target)]]
    tour = [(list_of_targets[0], list_of_targets[1]),
            (list_of_targets[1], list_of_targets[2]),
            (list_of_targets[2], list_of_targets[3]),
            (list_of_targets[3], list_of_targets[0])]
    example_tour_1 = Tour.from_coordinates(aoi, tour)

    # ------------------------------------------------------------------------------------------------------

    # example 2 of build a tour using node indexes from aoi graph structure
    choice_a_depot = len(random_target_points) + 1
    # the depots are enumerated by ntargets + id_depot, we select the second depot in this case
    # the targets are enumerated by 0, ntargets
    list_of_targets = [choice_a_depot, np.random.randint(n_target),
                       np.random.randint(n_target),
                       np.random.randint(n_target)]
    indexes_tour = [(list_of_targets[0], list_of_targets[1]),
            (list_of_targets[1], list_of_targets[2]),
            (list_of_targets[2], list_of_targets[3]),
            (list_of_targets[3], list_of_targets[0])]
    example_tour_2 = Tour.from_graph_indexes(aoi, indexes_tour)

    # ------------------------------------------------------------------------------------------------------

    # example 3 of build a tour using node indexes from aoi graph structure, starting by coordinates
    choice_a_depot = random_depots_points_on_x[0]
    list_of_targets = [choice_a_depot, random_target_points[np.random.randint(n_target)],
                       random_target_points[np.random.randint(n_target)],
                       random_target_points[np.random.randint(n_target)]]
    tour = [(list_of_targets[0], list_of_targets[1]),
            (list_of_targets[1], list_of_targets[2]),
            (list_of_targets[2], list_of_targets[3]),
            (list_of_targets[3], list_of_targets[0])]
    from_tour_to_indexes_tour = []
    for edge in tour:
        # convert each coordinates to node graph index
        index_edge = (aoi.node_index(edge[0]), aoi.node_index(edge[1]))
        from_tour_to_indexes_tour.append(index_edge)
    example_tour_3 = Tour.from_graph_indexes(aoi, from_tour_to_indexes_tour)

    # ------------------------------------------------------------------------------------------------------

    if plot:
        # by default edges are printed, no reason to not do it.
        plotter = ToursPlotManager(aoi, [example_tour_1, example_tour_2, example_tour_3])
        plotter.show()
        plotter.save(config.PATH_EXAMPLE_PLOTS + "test2_tours.png")

        plotter = ToursPlotManager(aoi, [example_tour_1, example_tour_2, example_tour_3], labels=True)
        plotter.save(config.PATH_EXAMPLE_PLOTS + "test2_tours_w_labels.png")

    return aoi, [example_tour_1, example_tour_2, example_tour_3]

def test3(plot=True):
    """ build a squad of drones
        print their details
        return them as a list
    """
    seed = 50
    speeds = [4, 8, 16]
    np.random.seed(seed)
    autonomies = [np.random.randint(200, 400), np.random.randint(1000, 1800), np.random.randint(9000)]

    # ------------------------------------------------------------------------------------------------------

    drones = [Drone(autonomy, speed)
                for autonomy in autonomies
                    for speed in speeds]

    if plot:
        print(drones)
    return drones


def test4():
    """
        build and plot a multiround solution
    """
    drones = test3(plot=False)
    # just use two drones
    drones = drones[:2]
    aoi, tour = test2(plot=False)

    mrs_builder = MultiRoundSolutionBuilder(aoi)
    # add drones
    mrs_builder = mrs_builder.add_drone(drones[0]).add_drone(drones[1])
    # add tours for 1st drone
    mrs_builder = mrs_builder.append_tour(drones[0], tour[0]).append_tour(drones[0], tour[2])
    # add tour for 2nd drone
    mrs_builder = mrs_builder.append_tour(drones[1], tour[1])

    # actual solution
    mrs = mrs_builder.build()
    # plot
    mrs.plot()
    # save
    mrs.save_plot(config.PATH_EXAMPLE_PLOTS + "test4_mrs.png")


def test5():
    """
        build a set of drones, tours in an AoI.
        Optimize the total coverage and cumulative coverage with the optimal gurobi models
        Plot the two results (multiround solution)
        return the multiround solution
    """

    seed = 50
    n_target = 50
    n_depots = 2
    width_area = 2000  # meters
    height_area = 2000  # meters
    max_rounds = 5
    input_tours_for_drones = 20
    len_input_tours_for_drones = 7
    np.random.seed(seed)

    # ------------------------------------------------------------------------------------------------------
    # build drones
    drones = test3(plot=False)
    # just use two drones
    drones = drones[:2]

    # ------------------------------------------------------------------------------------------------------
    # build the area of interest
    aoi = utility.build_random_aoi(width_area, height_area, n_target, n_depots, hovering_time=5, seed=seed)
    depots = aoi.depots

    # ------------------------------------------------------------------------------------------------------
    # build tours
    # random tours for first drones
    depot_first_drone = depots[0]
    tours_first_drone = [build_random_tour(aoi, depot_first_drone, np.random.randint(len_input_tours_for_drones-5, len_input_tours_for_drones+5))
                                for i in range(input_tours_for_drones)]  # build input_tours_for_drones input tours for the first drone
    # random tours for first drones
    depot_second_drone = depots[1]
    tours_second_drone = [build_random_tour(aoi, depot_first_drone, np.random.randint(len_input_tours_for_drones-5, len_input_tours_for_drones+5))
                         for i in range(input_tours_for_drones)]  # build input_tours_for_drones input tours for the first drone

    uavs_to_tours = {drones[0]: tours_first_drone, drones[1]: tours_second_drone}

    # ------------------------------------------------------------------------------------------------------
    # optimize Total Coverage
    model = TotalCoverageModel(aoi, uavs_to_tours, max_rounds, debug=False)
    model.build()
    model.optimize()  # get multi round solution
    mrs = model.solution
    assert mrs is not None, "optimal solution not found"
    print("TC-OPT covers", mrs.coverage_score(), "targets using", mrs.max_rounds, "rounds")
    # plot
    mrs.plot("TC-OPT") # for big istances (over 200/300 points) remove this plot
    # plot cumulative
    mrs.plot_cumulative_coverage_for_round("TC-OPT")
    # save
    mrs.save_plot(config.PATH_EXAMPLE_PLOTS + "test5_TC_mrs.png") # for big istances (over 200/300 points) remove this plot

    # ------------------------------------------------------------------------------------------------------
    # optimize Cumulative Coverage
    model = CumulativeCoverageModel(aoi, uavs_to_tours, max_rounds, debug=False)
    model.build()
    model.optimize()  # get multi round solution
    mrs = model.solution
    assert mrs is not None, "optimal solution not found"
    # plot
    print("AC-OPT covers", mrs.coverage_score(), "targets using", mrs.max_rounds, "rounds")
    mrs.plot("AC-OPT")  # for big istances (over 200/300 points) remove this plot
    #plot cumulative
    mrs.plot_cumulative_coverage_for_round("AC-OPT")
    # save
    mrs.save_plot(config.PATH_EXAMPLE_PLOTS + "test5_AC_mrs.png")  # for big istances (over 200/300 points) remove this plot


def test6(plot=True):
    """
        build a set of drones adn targets in an AoI.
        Use Algorithm2 : Drone-trajectory generation" (TMC : 10.1109/TMC.2020.2994529) to build a set of feasible trajectories for each drone.
        Plot the trajectories and save them
        return a dicitionarties {drone : trajectories} and the aoi
    """

    seed = 50
    n_target = 50
    n_depots = 2
    width_area = 2000  # meters
    height_area = 2000  # meters
    np.random.seed(seed)

    # ------------------------------------------------------------------------------------------------------
    # build drones (just two)
    drones = test3(plot=False)[:2]

    # ------------------------------------------------------------------------------------------------------
    # build the area of interest
    aoi = utility.build_random_aoi(width_area, height_area, n_target, n_depots, hovering_time=5, seed=seed)
    depots = aoi.depots

    # ------------------------------------------------------------------------------------------------------
    # for each drone compute and plot the trajectories
    out_trajectories = {}
    trajectories_builder = DroneTrajGeneration(aoi)
    for drone in drones:
        trajs = trajectories_builder.compute_trajectories(drone, depots[0])
        out_trajectories[drone] = trajs

        # plot
        if plot:
            # plot trajectories in text
            depot_index =  aoi.node_index(depots[0])
            print("Depot: ", depot_index)

            # by default edges are printed, no reason to not do it.
            for t in range(len(trajs)):
                print("Tour", t, "-", trajs[t].edges_w_indexes)
            plotter = ToursPlotManager(aoi, trajs)
            plotter.show()
            #save
            plotter.save(config.PATH_EXAMPLE_PLOTS + "test6_tours_" + str(drone) + ".png")

    return out_trajectories, aoi


def test7():
    """
        build a set of drones, tours in an AoI.
        Optimize the total coverage and cumulative coverage with the Greedy Algorithm
        Plot the two results (multiround solution)
        return the multiround solution
    """

    seed = 50
    max_rounds = 4
    np.random.seed(seed)

    # ------------------------------------------------------------------------------------------------------
    # build the area of interest, drone and tours
    uavs_to_tours, aoi = test6(plot=False)

    # ------------------------------------------------------------------------------------------------------
    # optimize Total Coverage
    alg = TotalGreedyCoverage(aoi, uavs_to_tours, max_rounds, debug=False)
    mrs = alg.solution()  # get multi round solution
    assert mrs is not None, "solution not found"
    print("TC-GaP covers", mrs.coverage_score(), "targets using", mrs.max_rounds, "rounds")
    # plot
    mrs.plot("TC-GaP") # for big istances (over 200/300 points) remove this plot
    # plot cumulative
    mrs.plot_cumulative_coverage_for_round("TC-GaP")
    # save
    mrs.save_plot(config.PATH_EXAMPLE_PLOTS + "test7_TC_mrs.png") # for big istances (over 200/300 points) remove this plot

    # ------------------------------------------------------------------------------------------------------
    # optimize Cumulative Coverage
    alg = CumulativeGreedyCoverage(aoi, uavs_to_tours, max_rounds, debug=False)
    mrs = alg.solution()  # get multi round solution
    assert mrs is not None, "optimal solution not found"
    # plot
    print("AC-GaP covers", mrs.coverage_score(), "targets using", mrs.max_rounds, "rounds")
    mrs.plot("AC-GaP")  # for big istances (over 200/300 points) remove this plot
    #plot cumulative
    mrs.plot_cumulative_coverage_for_round("AC-GaP")
    # save
    mrs.save_plot(config.PATH_EXAMPLE_PLOTS + "test7_AC_mrs.png")  # for big istances (over 200/300 points) remove this plot


if __name__ == "__main__":
    parser = ArgumentParser()
    
    # MANDATORY
    parser.add_argument('test_id', type=int,
                        help="the index of test to run/perform")

    args = parser.parse_args()
    test_id = args.test_id          

    print("Run test:" + str(test_id))
    if test_id == 1:
        test1()
    elif test_id == 2:
        test2()
    elif test_id == 3:
        test3()
    elif test_id == 4:
        test4()
    elif test_id == 5:
        test5()
    elif test_id == 6:
        test6()
    elif test_id == 7:
        test7()
