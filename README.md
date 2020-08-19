# Task Assignment for Early Target Inspection in Squads of Aerial Drones

## Description and Scope
The repository contains the release code for two conference and journal articles:


1) On Task Assignment for Early Target Inspection in Squads of Aerial Drones
<br /> link: https://ieeexplore.ieee.org/document/8885227 - DOI: 10.1109/ICDCS.2019.00209
<br /> Authors: Novella Bartolini ; Andrea Coletta ; Gaia Maselli

2) A Multi-Trip Task Assignment for Early Target Inspection in Squads of Aerial Drones
<br /> link: https://ieeexplore.ieee.org/document/9093167 - DOI : 10.1109/TMC.2020.2994529
<br /> Authors: Novella Bartolini ; Andrea Coletta ; Gaia Maselli ; Ala' Khalifeh


Owner: Andrea Coletta
<br /> 
Version v1.0
<br /> 
Please cite these works if you use part of this code. 


# Project Structure
The project is implemented in python3.8 using as main libraries networkx (https://networkx.github.io/) and gurobi (https://www.gurobi.com/)
<br /> 
The project has the following structure:
``` bash
.
├── data
│   ├── articles
│   │   ├── [ICDCS]On_task_assignment_for_early_target_inspection_in_squads_of_aerial_drones.pdf
│   │   └── [TMC]A_multi_trip_task_assignment_for_early_target_inspection_in_squads_of_aerial_drones.pdf
│   └── test_plots
│       ├── test1_edges_and_labels.png
│       ├── ****.png
│       └── test5_TC_mrs.png
├── gurobi.sh
├── README.md
└── src
    ├── algorithms
    │   ├── approxalg.py
    │   ├── optimal.py
    │   └── trajbuilder.py
    ├── entities
    │   └── trajenties.py
    ├── tests
    │   └── test_main.py
    └── util
        ├── config.py
        ├── trajplot.py
        └── utility.py

``` 
Where,
The ``src.algorithms`` dir contains all the core code about: 
<br /> 
    - optimal.py contains TC-OPT (gurobi model), AC-OPT (gurobi model)
<br /> 
    - approxalg.py contains AC-GaP and AC-OpT, along as the pruning strategy
<br /> 
    - trajbuilder.py contains Algorithm2 - Drone-trajectory generation to generate a set of feasible trajectories for each drone

The ``src.util`` dir contains all the utility functions and classes:
<br /> 
    - config.py contains static path (where save plots) and static variable used along all the project
<br /> 
    - trajplot.py contains the code needed to print/plot/save tours and solutions
<br /> 
    - utility.py contains all utility functions that are used (e.g., euclidean distance among points)

The ``src.erntities`` dir contains all classes that are used to wrap entities such as Tours, AoI (area of interest), Drones, Solutions, and more.

The ``src.tests`` contains test_main.py which can be used to test and understand the project, further details in the following paragraphs.

The ``data.test_plot`` contains all the png images produced by the ``src.test.test_main.py`` code.

The ``data.articles`` contains the two referred articles TMC and ICDCS (see Sec. Description and Scope)


The project realease is currently ongoing: new code will be updated soon; please notify us about possible bugs and/or errors.

## Run simulations and examples
The user can start using and testing the project by running example on ``src.tests``.

Use ``python3 -m src.tests.test_main test_id`` to run the test number `test_id`; while run ``python3 -m src.tests.test_main -h`` to see all possible and mandatory arguments.
All the other test parameters can be changed direclty on the file ``src.test.test_main``.

The `test_id` = 1:
- builds a AoI with random n-target points and x depots
- plots the area with and w/o edges
- saves the area with and w/o edges
- returns the AoI, targets and depots

The `test_id` = 2:
- builds a AoI with random n-target points and x depots
- creates three input tours and plot them along the AoI
- plots the area with the tours
- saves the tours with and w/o labels
- returns the tours and the area

The `test_id` = 3:
- builds a squad of drones
- prints their details
- returns them as a list

The `test_id` = 4:
- builds a random multi round solution using drones from test3, aoi and tours from test2
- plots the multi round solution
- save the plot of the multi round solution


The `test_id` = 5:
- builds an AoI, a set of round tours (20 tours) and loads drones from test3 
- associates tours to drones 
- runs TC-OPT and AC-OPT to optimize the assingment on 5 rounds
- prints stats about the two solutions
- plots the two solutions and their cumulative coverage along rounds
- saves the plot of the two solutions


The `test_id` = 6:
- builds an AoI and loads drones from test3 
- uses Algorihtm2 to generate, for each drone, a set of feasible trajectories
- prints, plots and saves the trajectories


The `test_id` = 7:
- using the drones, the AoI and trajectories of test-6 find tests the two greedy algorithms.
- runs TC-OPT and AC-OPT to achieve an assingment on 4 rounds
- prints stats about the two solutions
- plots the two solutions and their cumulative coverage along rounds
- saves the plot of the two solutions


## Contacts

For further information contact Andrea Coletta at coletta[AT]di.uniroma1.it

