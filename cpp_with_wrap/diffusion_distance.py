import numpy as np
from numpy.linalg import matrix_power
import matplotlib.pyplot as plt

import math

def pairwise_Euclidean_distances(point_cloud):
    cols, rows = np.shape(point_cloud)
    Eucl_dist = np.empty((cols, cols))
    for i in range(cols):
        for j in range(cols):
            Eucl_dist[i, j] = np.linalg.norm(point_cloud[i] - point_cloud[j])
    return Eucl_dist


def diffusion_distance(point_cloud, alpha, pathlength):
    # Just for now
    Euclidean_distances = pairwise_Euclidean_distances(point_cloud)

    # 1. Compute transition probability matrix
    p_matrix = np.exp(-Euclidean_distances**2/alpha)
    np.fill_diagonal(p_matrix, 0)

    # Scaling
    for x in range(np.shape(p_matrix)[0]):
        d_X = sum(p_matrix[x])
        p_matrix[x] = 1/d_X * p_matrix[x]

    # 2. Compute powers of p
    t_matrix = matrix_power(p_matrix, pathlength)

    # 3. Compute diffusion distance matrix
    n = np.shape(t_matrix)[0]
    diffusion_distance = np.empty([n, n])
    for i in range(n):
        for j in range(n):
            diffusion_distance[i, j] = np.linalg.norm(t_matrix[i, :] - t_matrix[:, j])

    return diffusion_distance, t_matrix


n = 50

# Set parameters
pathlength = 50 #3*n
alpha = 0.3

# Create data set
l = 1
beta = math.pi/4

mu, sigma = 0, 0.05

set_1 = np.array([[x*np.cos(beta)+np.random.normal(mu, sigma), x*np.sin(beta)+np.random.normal(mu, sigma), 0] for x in np.linspace(0, l, n)])
set_2 = np.array([[-x*np.cos(beta)+np.random.normal(mu, sigma), x*np.sin(beta)+np.random.normal(mu, sigma), 0] for x in np.linspace(0, l, n)])
#set_3 = np.array([[l*np.cos(beta)+np.random.normal(mu, 3*sigma), l*np.sin(beta)+np.random.normal(mu, 3*sigma), 0] for x in np.linspace(0, l, n)])
#set_4 = np.array([[-l*np.cos(beta)+np.random.normal(mu, 3*sigma), l*np.sin(beta)+np.random.normal(mu, 3*sigma), 0] for x in np.linspace(0, l, n)])
set_3 = np.array([[0, l*np.sin(beta), 0]])
"""
set_1 = np.array([[0, 0, 0], [0.2, 0.2, 0], [0.4, 0.4, 0], [0.6, 0.6, 0]])
set_2 = np.array([[-0.2, 0.2, 0], [-0.4, 0.4, 0], [-0.6, 0.6, 0]])
set_3 = np.array([[0.8, 0.8, 0], [1.0, 0.8, 0], [1.2, 0.8, 0],
                  [0.8, 1.0, 0], [1.0, 1.0, 0], [1.2, 1.0, 0],
                  [0.8, 1.2, 0], [1.0, 1.2, 0], [1.2, 1.2, 0]])
set_4 = np.array([[-0.8, 0.8, 0], [-1.0, 0.8, 0], [-1.2, 0.8, 0],
                  [-0.8, 1.0, 0], [-1.0, 1.0, 0], [-1.2, 1.0, 0],
                  [-0.8, 1.2, 0], [-1.0, 1.2, 0], [-1.2, 1.2, 0]])
"""
set_total = np.concatenate((set_1, set_2, set_3))


# Plot data set
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.scatter3D(set_1[:, 0], set_1[:, 1], set_1[:, 2])
ax.scatter3D(set_2[:, 0], set_2[:, 1], set_2[:, 2])
ax.scatter3D(set_3[:, 0], set_3[:, 1], set_3[:, 2])
#ax.scatter3D(set_4[:, 0], set_4[:, 1], set_4[:, 2])
#ax.scatter3D(set_5[:, 0], set_5[:, 1], set_5[:, 2])
plt.show()

# Output: Pairwise diffusion distances
pairwise_diffusion_distance, t_matrix = diffusion_distance(set_total, alpha, pathlength)
print(pairwise_diffusion_distance)



