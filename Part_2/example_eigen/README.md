# example_eigen

Example program to illustrate use of Eigen methods.  This node creates random, nearly-planar data, then applies a covariance
eigenvalue/eigenvector method to find the best-fit plane parameters.  Note that it does not contain any ROS-specific code.

## Example usage

With `local_install.bash` sourced, run:

`$ example_eigen_plane_fit`

In the code, can change the values of the prescribed plane normal and offset and synthetic data noise, then examine the 
resulting identified parameters.  For zero noise, the result should be virtually perfect.
