"""
Owner: Andrea Coletta
Version v1.0
Release code for TMC : 10.1109/TMC.2020.2994529 and its ICDCS conference version : 10.1109/ICDCS.2019.00209

Please cite these works in case of use.


File content:
This file contains classes responsable to print AoI, Tours and Multi-Round Solutions

The main classes are:
    Tours -> a wrapper which represent a tour for a drone.
    Drone -> a drone entity that has a given speed and available energy
    MultiRoundSolution -> An assignment of tours to drones to cover a given set of points

"""

from matplotlib import colors

import networkx as nx
import matplotlib.pyplot as plt

# Size and settings for the plotting
I_DEPOT = {"size": 50, "color": 'b', "label": "depot"}
I_NODE = {"size": 35, "color": 'r', "label": "nodes"}
I_EDGE = {"size": 3, "color": 'g', "label": "edges"}
ELABELS_SIZE = 8


# -----------------------------------------------------------------------------------
#
#							 FUNCTIONS
#
# -----------------------------------------------------------------------------------

def gradient_color(lenght):
    """
	Utility function to generates a set of colors to uniquely identify each tour.
	:param lenght: the number of colors to generates
	:return:
	"""
    t_colors = []
    paired = plt.get_cmap('Paired')
    for i in range(lenght):
        c = paired(i / float(lenght))
        t_colors += [colors.to_hex(c)]
    return t_colors


""" This class is responsable to print and AoI with several targets, viable paths and depots """
class AoIPlotManager:

    def __init__(self, aoi, title: str = "AoI TrajPlan", labels: bool = False, edges: bool = False):
        """

        :param aoi: the input AoI to plot
		:param title: the title of the plot
		:param labels: if True each viable path (edge) will have a lebel with the weight; Otherwise no labels on edges (default=False)
		:param edges: if True the viable paths/edges will be printed; otherwise only the targets and depots are plotted (default=False)
		"""
        self.aoi = aoi
        self.title = title
        self.labels = labels
        self.edges = edges

    def make_plot(self):
        """ make the actual plot using matplotlib and networkx """
        pos = nx.get_node_attributes(self.aoi.graph, 'pos')
        self.plot_nodes(pos)
        if self.edges:
            self.plot_edges(pos)
            if self.labels:
                self.plot_edgelabels(pos)

        self.fix_plot_dim()
        plt.title(self.title)
        ax = plt.gca()
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.legend()

    def fix_plot_dim(self):
        """ fix the plot dimension - width and heigh area """
        plt.axis([0, self.aoi.width, 0, self.aoi.height])

    def plot_nodes(self, pos: dict):
        """ Internal use -  plot the nodes, both depots and targets
				pos : dict -> all the position of nodes in the graph { node : pos, .. }
		"""
        nd_nodes = [x for x in self.aoi.graph.nodes() if  self.aoi.graph.nodes[x]["pos"] not in self.aoi.depots]
        nd_depots = [x for x in self.aoi.graph.nodes() if self.aoi.graph.nodes[x]["pos"] in self.aoi.depots]
        nx.draw_networkx_nodes(self.aoi.graph, pos=pos, nodelist=nd_nodes,
                               node_size=I_NODE["size"],
                               node_color=I_NODE["color"],
                               label=I_NODE['label'])
        nx.draw_networkx_nodes(self.aoi.graph, pos=pos, nodelist=nd_depots,
                               node_size=I_DEPOT["size"],
                               node_color=I_DEPOT["color"],
                               label=I_DEPOT["label"])

    def plot_edges(self, pos: dict):
        """ plot the viable paths / edges of the AoI
				pos : dict -> all the position of nodes in the graph { node : pos, .. }
		"""
        nx.draw_networkx_edges(self.aoi.graph, pos,
                               width=I_EDGE["size"],
                               edge_color=I_EDGE["color"],
                               arrows=False,
                               label=I_EDGE["label"])

    def plot_edgelabels(self, pos: dict):
        """ plot the edge / path details (i.e., len of edge in meters)
			pos : dcit -> all the position of nodes in the graph { node : pos, .. }
		"""
        # can be used also personalized labels
        labels = dict(map(lambda x: ((x[0], x[1]),
                                     str(int(x[2]["weight"]))),
                          self.aoi.graph.edges(data=True)))

        nx.draw_networkx_edge_labels(self.aoi.graph,
                                     font_size=ELABELS_SIZE,
                                     pos=pos,
                                     edge_labels=labels)

    def show(self):
        """ show the plot on dedicated window (block mode) """
        self.make_plot()
        plt.plot()
        plt.show()
        self.close()

    def save(self, path: str):
        """

		:param path: where save the plot (include extension, e.g., data/file.png)
		:return: None
		"""
        self.make_plot()
        if "png" in path:
            plt.savefig(path, dpi=300)
        else:
            plt.savefig(path)
        self.close()

    def close(self):
        """ help method to close a plot and avoid errors """
        plt.clf()
        plt.close()



"""    The class help to plots some tours in the AoI """
class ToursPlotManager(AoIPlotManager):

    def __init__(self, aoi, tours, title: str = "AoI TrajPlan", labels: bool = False, tour_colors:list=None, tour_labels:list=None):
        """
        :param aoi: the input AoI to plot
		:param tours: a list of Tours to plot. each element of the list must be an instance of trajentities.Tour
        :param title: the title of the plot
		:param labels: if True each viable path (edge) will have a lebel with the weight; Otherwise no labels on edges (default=False)
		:param edges: if True the viable paths/edges will be printed; otherwise only the targets and depots are plotted (default=False)
		:param tour_colors: a list of colors, to plot each tour len(tour_colors) == len(tours). If tour_colors =None and colors will be set automatically. (default=None)
        :param tour_labels: a list of labels for each tour (for legend purpose). If None no labels the index are used to identify tours. (default None)
        """
        AoIPlotManager.__init__(self, aoi, title, labels, True)
        self.tours = tours
        # actual colors for the plot
        if tour_colors is None:
            self.tour_colors = gradient_color(len(self.tours))
        else:
            self.tour_colors = tour_colors
        # actual labels for tours
        if tour_labels is None:
            self.tour_labels = ["Tour " + str(i) for i in range(len(self.tours))]
        else:
            self.tour_labels = tour_labels

    def plot_edges(self, pos):
        for itour in range(0, len(self.tours)):
            c = self.tour_colors[itour]
            nx.draw_networkx_edges(self.aoi.graph, pos, width=I_EDGE["size"],
                                   edgelist=self.tours[itour].edges_w_indexes, edge_color=c,
                                   arrows=False, label=self.tour_labels[itour])

    def plot_edgelabels(self, pos: dict):
        """ plot the edge / path details (i.e., len of edge in meters)
			pos : dcit -> all the position of nodes in the graph { node : pos, .. }
		"""
        # can be used also personalized labels
        all_edges = []
        for t in self.tours:
            all_edges.extend(t.edges_w_indexes)
        labels = {}
        for x in self.aoi.graph.edges(data=True):
            if (x[0], x[1]) in all_edges or (x[1], x[0]) in all_edges:
                labels[(x[0], x[1])] = str(int(x[2]["weight"]))

        nx.draw_networkx_edge_labels(self.aoi.graph,
                                     font_size=ELABELS_SIZE,
                                     pos=pos,
                                     edge_labels=labels)


