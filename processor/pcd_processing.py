
import numpy as np
import open3d as o3d
import getopt
import sys
import time

OPTIONS = "hvm:"

is_verbose = False

path = "./data/"
datafile = "point_data.npy"

#
# @brief allpies the most least squares method to the point cloud data
# @param points - the given point cloud
# @param pcd - the given point cloud as a PointCloud object
# @param search_radius - the the radius about each point to search
# @param max_nn - the maximum node to use to approx each point
# @param h - the smoothing parameter (feature size)
#
def apply_mls(points, pcd, search_radius=0.5, max_nn=30, h=10):
	num_points = points.shape[0]

	# new point array
	smoothed_points = np.zeros(points.shape)

	# Use KD-Tree for fast location based lookups
	pcd_tree = o3d.geometry.KDTreeFlann(pcd)

	# Array for storing normals
	normals = np.zeros(points.shape)

	# define weight function as gaussian kernel
	# @param distance - the distance of the point from the reference point
	# @param h - the smoothing parameter (feature size)
	def weight_of(distance,h):
		return np.exp(-(np.asarray(distance)**2)/h**2)

	# Loop through each point
	for i in range(num_points):
		point = points[i]

		# Get the surrounding points within range search_radius and of max count max_nn
		(num_neighbors, indices, distances) = pcd_tree.search_hybrid_vector_3d(point, search_radius, max_nn)

		# if there are not enough neighbors to fit a plane equation, continue
		if num_neighbors < 3:
			smoothed_points[i] = point
			continue
		
		# Get weights and neighbors
		weights = weight_of(distances,h)
		neighbors = points[indices,:]

		# Get the weighted centroid (or mean) of all of the data points (axis=0 indicates that the average will be taken along the cols)
		weighted_centroid = np.average(neighbors, axis=0, weights=weights)

		# Center neighbors about the weighted centroid
		centered_neighbors = neighbors - weighted_centroid

		# Find the normal of the fit plane using the eigen vector associated with the smallest eigen value of the points covariant matrix (Weighted Covariance)
		# weights[:,np.newaxis] ensures that the array (N,) interfaces with the (N,3) array of the centered neighbor points correctly
		cov_mat = np.dot((weights[:, np.newaxis] * centered_neighbors).T, centered_neighbors)
		e_val, e_vec = np.linalg.eigh(cov_mat)
		normal = e_vec[:,0] # Normal is the eighen vector associated with the smallest eigen value

		# Find projected point
		dist_to_plane = np.dot(point - weighted_centroid, normal)
		projected_point = point - (dist_to_plane * normal)

		# Save projected point into smoothed matrix
		smoothed_points[i] = projected_point

		# Save normal of points
		normals[i] = normal
	
	# create new point cloud

	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(smoothed_points)
	pcd.normals = o3d.utility.Vector3dVector(normals)
	pcd.orient_normals_towards_camera_location(camera_location)

	return pcd


if __name__ == "__main__":

	####################
	#	0. Process command-line arguments using getopt
	####################

	mode = "all"

	argv = sys.argv[1:]
	try:
		opts, args = getopt.getopt(argv, OPTIONS)
	except getopt.GetoptError:
		sys.exit(2)
	
	for opt, arg in opts:
		if opt in ("-h", "--help"):
			print("Usage: pcd_processing [OPTIONS]")
			print("\t-m  str\t\tSets the processing mode (\"bpa\",\"bpa_mls\",\"psr\",default=\"all\")")
			print("\t-v\t\tEnables verbose output (outputs data point count and process time)")
			print("\t-h\t\tPrints help text")
			sys.exit(0) # Exit success
		elif opt in ("-v", "--verbose"):
			is_verbose = True
			print("Running application in verbose mode")
		elif opt in ("-m", "--mode"):
			if arg == None:
				print("Missing argument")
				sys.exit(1) # Exit failure
			elif arg not in ("bpa", "bpa_mls", "psr", "all"):
				print("Invalid mode. Try -h for help")
				sys.exit(1) # Exit failure
			else:
				mode = arg
	
	######################
	# 1. Load point data and convert to open3d point cloud object
	######################
	try:
		points = np.load(path + datafile)
	except:
		print(f"failed to open {path + datafile}")
		sys.exit(1) # Exit failure

	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(points)

	if is_verbose: print(f"Dataset contains {points.shape[0]} data points")

	######################
	# 2. Estimate normals with known camera position
	######################

	pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))
	pcd.orient_normals_consistent_tangent_plane(100)
	camera_location = np.array([0.0,0.0,0.0])
	pcd.orient_normals_towards_camera_location(camera_location)
	

	# BALL PIVOTING ALGORITHM
	if mode in ("bpa", "all"):

		if is_verbose: print("Processing BPA...")

		# Start time
		start_time = time.perf_counter()

		######################
		# 3. Calculate optimal ball radii via average distance between closest point distance of all points
		######################

		distances = pcd.compute_nearest_neighbor_distance()
		avg_dist = np.mean(distances)
		# All of the differance ball radii to use
		radii = [avg_dist * 0.5, avg_dist * 1.0, avg_dist * 2.0, avg_dist * 4.0]


		######################
		# 4. Use BPA to create mesh
		######################

		mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
			pcd, o3d.utility.DoubleVector(radii)
		)

		# end timer
		end_time = time.perf_counter()

		######################
		# 5. Output mesh to file
		######################

		if is_verbose: print(f"Processing BPA completed in {(end_time - start_time):.4f} seconds")
		meshname = path + "bpa_mesh.ply"
		if is_verbose: print(f"Writting \"{meshname}\" to file...")
		o3d.io.write_triangle_mesh(meshname, mesh)

	# BALL PIVOTING ALGORITHM WITH MOVING LEAST SQUARES
	if mode in ("bpa_mls", "all"):

		if is_verbose: print("Processing BPA with MLS...")

		# Start time
		start_time = time.perf_counter()

		######################
		# 3. Use MLS to smooth point cloud
		######################
		smooth_pcd = apply_mls(points, pcd)

		if is_verbose: print("Finished MLS...")

		######################
		# 4. Calculate optimal ball radii via average distance between closest point distance of all points
		######################

		distances = smooth_pcd.compute_nearest_neighbor_distance()
		avg_dist = np.mean(distances)
		# All of the differance ball radii to use
		radii = [avg_dist * 0.5, avg_dist * 1.0, avg_dist * 2.0, avg_dist * 4.0]


		######################
		# 5. Use BPA to create mesh
		######################

		mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
			smooth_pcd, o3d.utility.DoubleVector(radii)
		)

		# end timer
		end_time = time.perf_counter()

		######################
		# 6. Output mesh to file
		######################

		if is_verbose: print(f"Processing BPA with MLS completed in {(end_time - start_time):.4f} seconds")
		meshname = path + "bpa_mls_mesh.ply"
		if is_verbose: print(f"Writting \"{meshname}\" to file...")
		o3d.io.write_triangle_mesh(meshname, mesh)

	# POISSON SURFACE RECONSTRUCTION
	if mode in ("psr", "all"):

		if is_verbose: print("Processing PSR...")

		# Start time
		start_time = time.perf_counter()

		######################
		# 3. Run Poisson Surface Reconstruction
		######################

		mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
			pcd, depth=10
		)

		######################
		# 4. Clean low-density noise out of mesh as poisson can hallucinate unneeded artifacts
		######################

		densities = np.array(densities)
		verticies_to_remove = densities < np.quantile(densities, 0.05)
		mesh.remove_vertices_by_mask(verticies_to_remove)

		# end timer
		end_time = time.perf_counter()

		######################
		# 5. Output mesh to file
		######################

		if is_verbose: print(f"Processing PSR completed in {(end_time - start_time):.4f} seconds")
		meshname = path + "psr_mesh.ply"
		if is_verbose: print(f"Writting \"{meshname}\" to file...")
		o3d.io.write_triangle_mesh(meshname, mesh)