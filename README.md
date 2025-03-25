# Iterative Closest Point (ICP) for Point Cloud Registration

This project implements the Iterative Closest Point (ICP) algorithm for rigid transformation estimation between two 3D point clouds. ICP is widely used for aligning two datasets, such as those obtained from 3D sensors, and is essential in fields like robotics and computer vision for tasks like object recognition and motion estimation.

## Project Overview

In this project, the goal is to estimate the relative transformation (rotation and translation) between two 3D point clouds using the ICP algorithm. The algorithm alternates between finding point correspondences and solving for the optimal transformation that minimizes the alignment error.

### Key Features:
- **Point Cloud Transformation**: The project involves transforming one point cloud into the frame of another using an initial guess and iterating until the transformation is optimized.
- **ICP Implementation**: The core of the project is the ICP algorithm, which iteratively refines the rigid transformation (rotation and translation).
- **Root Mean Squared Error (RMSE)**: After estimating the transformation, the RMSE is calculated to evaluate the quality of the registration.

## Files

- **`ICP.m`**: Contains the main function for the Iterative Closest Point algorithm, which refines the transformation (rotation and translation) iteratively.
- **`compute_optimal_registration.m`**: This helper function calculates the optimal rigid transformation using the Kabsch algorithm, given point correspondences.
- **`estimate_correspondences.m`**: This function computes point correspondences between two point clouds based on the current estimate of the transformation.
- **`main.m`**: A script to execute the ICP algorithm, process the results, and plot the point clouds.
- **`pclX.txt`, `pclY.txt`**: Example 3D point clouds that will be registered using ICP. These are used as input files for the algorithm.

## Installation

Clone this repository to your local machine:

```bash
git clone https://github.com/yourusername/ICP-PointCloud-Registration.git
cd ICP-PointCloud-Registration


![image](https://github.com/user-attachments/assets/099ae62f-7935-4ee6-9b86-9e39fb1f021e)
