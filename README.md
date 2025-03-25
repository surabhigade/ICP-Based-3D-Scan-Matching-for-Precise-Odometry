# Iterative Closest Point (ICP) for Point Cloud Registration

This project implements the Iterative Closest Point (ICP) algorithm to estimate the rigid transformation between two 3D point clouds. ICP is crucial in fields like robotics and computer vision for tasks such as object recognition and motion estimation.

## Project Overview

In this project, the goal is to estimate the relative transformation (rotation and translation) between two 3D point clouds using the ICP algorithm. The algorithm alternates between finding point correspondences and solving for the optimal transformation that minimizes the alignment error.

### Key Features:
- **Point Cloud Transformation**: The project involves transforming one point cloud into the frame of another using an initial guess and iterating until the transformation is optimized.
- **ICP Implementation**: The core of the project is the ICP algorithm, which iteratively refines the rigid transformation (rotation and translation).
- **Root Mean Squared Error (RMSE)**: After estimating the transformation, the RMSE is calculated to evaluate the quality of the registration.

## Files

- **`ICP.m`**: Contains the main function for the Iterative Closest Point algorithm, which refines the transformation (rotation and translation) iteratively.
- **`generate_data.m`**: This  script is responsible for generating synthetic 3D point cloud data that can be used for testing the ICP algorithm. This file creates two point clouds with a known rigid transformation (rotation and translation), which allows you to validate the performance of the ICP algorithm by comparing the estimated transformation with the known true values.
- **`pclX.txt`, `pclY.txt`**: Example 3D point clouds that will be registered using ICP. These are used as input files for the algorithm.


![image](https://github.com/user-attachments/assets/099ae62f-7935-4ee6-9b86-9e39fb1f021e)
