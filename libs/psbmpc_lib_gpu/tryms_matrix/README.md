# Cuda Matrix Library (Trym's ad hoc matrix library)
<p> This is a matrix library created for use in CUDA when running the PSB-MPC on the GPU, mostly because the Eigen CUDA support at the time was not satisfactory. It is based on the Curiously Recurring Template Pattern (CRTP) idiom. The library is not in any way complete, and the functionality implemented could have been much better coded (in hindsight) using expression templates (as e.g. Eigen uses). Thus, it is only tested and meant for the calculations performed as of now in the CPE, Ownship and CB_Cost_Functor classes.  </p>

<p> Move with care using this library for other stuff than what it is used for as of now in this code base. </p>


<p> Trym Tengesdal, 8. Oktober 2020.  </p>
