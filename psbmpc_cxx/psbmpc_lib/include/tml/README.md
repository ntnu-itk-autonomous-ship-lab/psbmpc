# TML: Trym's (shitty) matrix library

This is a matrix library created for use in CUDA when running the PSB-MPC on the GPU, mostly because the Eigen CUDA support at the time was not satisfactory. It is based on the Curiously Recurring Template Pattern (CRTP) idiom. The library is not in any way complete, and the functionality implemented could have been much better coded (in hindsight) using expression templates (as e.g. Eigen uses). Thus, it is only tested and meant for the calculations performed as of now in the CPE, Ownship and CB_Cost_Functor classes.

The library accepts any data type, but is best suited for floating point numbers. It has the following inheritance structure

- Matrix_Base: Base class which implements methods such as inverse, determinant etc. that all derived classes can use (due to CRTP).
		- Static_Matrix : Used for fixed-size vectors and matrices.
		- PDMatrix (Pseudo_Dynamic_Matrix) : All-purpose matrix/vector container, which acts like a dynamic matrix, but in practice has a max number of rows and columns, such that it is statically allocated.

Move with care using this library for other stuff than what it is used for as of now in this code base. There will definitely be undiscovered/unexpected bugs when attempting to use the containers for operations they have not been tested for.

Trym Tengesdal, 21. November 2021.
