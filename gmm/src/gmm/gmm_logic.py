import matplotlib.pyplot as plt
import numpy as np
from sklearn.mixture import GaussianMixture
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
from planner_msgs.msg import Graph, Vertex, Edge

class Vertex(): 
    def __init__(self, index, x, y, z,):
        self.id = index
        self.x = x
        self.y = y
        self.z = z 

class Gmm():
    def __init__(self, n_components, own_graph_msg, query_graph_msg):
        self.graph_vertices = [] # (1, n)
        self.query_vertices = [] # (1, m)

        self.dataset = [] # (n, 2)
        self.query = [] # (m, 2)
        
        self.n_components = n_components
        
        self.loadGraphs(own_graph_msg, query_graph_msg)

    def loadGraphs(self, own_graph_msg, query_graph_msg):
        vertices_list = own_graph_msg.vertices
        query_vertices_list = query_graph_msg.vertices
        
        # Prepare vertices for fitting the GMM
        for vertex in vertices_list:
            if vertex.is_frontier:
                continue
            else:
                index = vertex.id
                x = vertex.pose.position.x
                y = vertex.pose.position.y
                z = vertex.pose.position.z
                new_vertex = Vertex(index, x, y, z)
                self.graph_vertices.append(new_vertex)
                self.dataset.append([x,y])

        # Prepare vertices to query
        for vertex in query_vertices_list:
            index = vertex.id
            x = vertex.pose.position.x
            y = vertex.pose.position.y
            z = vertex.pose.position.z
            new_vertex = Vertex(index, x, y, z)
            self.query_vertices.append(new_vertex)
            self.query.append([x,y])

        self.dataset = np.vstack(self.dataset)
        self.query = np.vstack(self.query)

    def draw_ellipse(self, position, covariance, ax, n_std=2.0, **kwargs):
        eigvals, eigvecs = np.linalg.eigh(covariance)
        order = eigvals.argsort()[::-1]
        eigvals, eigvecs = eigvals[order], eigvecs[:, order]
        angle = np.degrees(np.arctan2(*eigvecs[:, 0][::-1]))
        width, height = 2 * n_std * np.sqrt(eigvals)
        ellipse = Ellipse(xy=position,
                            width=width,
                            height=height,
                            angle=angle,
                            **kwargs)

        ax.add_patch(ellipse) 
    
    def plot_gmm_components(self, gmm):
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.scatter(self.dataset[:,0], self.dataset[:,1], s=5, c='black', alpha=0.5)
        
        for i in range(gmm.n_components):
            self.draw_ellipse(gmm.means_[i],
                                gmm.covariances_[i],
                                ax,
                                edgecolor='red',
                                facecolor='none',
                                linewidth=2 )
        ax.set_title("GMM Components")
        plt.savefig("comp_test1.png", bbox_inches='tight')

    def plot_gmm(self, gmm, x_keep=[], y_keep=[], show_points=False):
        # Gather all possible points to find the true "Canvas"
        all_x = self.dataset[:, 0]
        all_y = self.dataset[:, 1]
        
        if show_points and len(x_keep) > 0:
            all_x = np.concatenate([all_x, x_keep])
            all_y = np.concatenate([all_y, y_keep])

        # Calculate bounds with a 5% padding so points aren't cut off at the edge
        x_min, x_max = all_x.min(), all_x.max()
        y_min, y_max = all_y.min(), all_y.max()

        x_pad = (x_max - x_min) * 0.05
        y_pad = (y_max - y_min) * 0.05

        x_min -= x_pad
        x_max += x_pad
        y_min -= y_pad
        y_max += y_pad

        # Create grid based on these GLOBAL padded bounds
        # Increase resolution to 500j for a smoother look over larger areas
        xi, yi = np.mgrid[x_min:x_max:500j, y_min:y_max:500j]
        grid = np.vstack([xi.ravel(), yi.ravel()]).T

        # Score GMM (points outside training data will be ~0 density)
        log_z = gmm.score_samples(grid)
        z = np.exp(log_z).reshape(xi.shape)

        plt.figure(figsize=(12, 9))

        # Plotting with explicit extent and origin
        # IMPORTANT: imshow expects (rows, columns), which is why we transpose z
        plt.imshow(z.T,
                origin='lower',
                extent=[x_min, x_max, y_min, y_max],
                cmap='YlOrRd',
                aspect='auto')

        plt.colorbar(label="Density")

        if show_points:
            plt.scatter(x_keep, y_keep, s=5, c='black', edgecolors='black', linewidth=0.5, alpha=0.8)
            # Explicitly set the limits one last time to be safe
            plt.xlim(x_min, x_max)
            plt.ylim(y_min, y_max)
            plt.title("GMM Density: Global View (Dataset + Filtered Points)")
            plt.savefig("filt_test1.png", bbox_inches='tight', dpi=300)
        else: 
            plt.title("GMM Density: Dataset View")
            plt.savefig("density_tested.png", bbox_inches='tight', dpi=300)

    def fitGaussianMixtureModel(self, threshold_percentile):
        gmm = GaussianMixture(n_components=self.n_components).fit(self.dataset)

        # self.plot_gmm(gmm)
        # self.plot_gmm_components(gmm)

        densities_dataset = np.exp(gmm.score_samples(self.dataset))
        threshold = np.percentile(densities_dataset, threshold_percentile)

        densities_query = np.exp(gmm.score_samples(self.query))

        x_keep = []
        y_keep = []
        keep_query_vertices = []

        # If the density value is higher then the threshold discard the vertex for the merging process
        for i, value in enumerate(densities_query):
            if value > threshold:
                continue
            else:
                x_keep.append(self.query[i,0])
                y_keep.append(self.query[i,1])
                keep_query_vertices.append(self.query_vertices[i])
        
        x_keep = np.array(x_keep)
        y_keep = np.array(y_keep)
        
        # self.plot_gmm(gmm, x_keep, y_keep, show_points=True) 
        
        # Return the list of vertices to keep 
        return keep_query_vertices