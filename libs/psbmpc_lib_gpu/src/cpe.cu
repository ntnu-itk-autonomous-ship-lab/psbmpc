/****************************************************************************************
*
*  File name : cpe.cu
*
*  Function  : Class functions for the collision probability estimator
*
*  
*            ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2020 Trym Tengesdal, NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengesdal
*
*  Modified  : 
*
*****************************************************************************************/

#include "cpe.cuh"
#include "utilities.cuh"

#include <thrust/device_vector.h>
#include <stdio.h>
#include <cuda.h>
#include <curand_kernel.h>
#include "assert.h"

//#define CUDA_CALL(x) do { if((x)!=cudaSuccess) { \printf("Error at %s:%d\n",__FILE__,__LINE__);\return EXIT_FAILURE;}} while(0)
//#define CURAND_CALL(x) do { if((x)!=CURAND_STATUS_SUCCESS) { \printf("Error at %s:%d\n",__FILE__,__LINE__);\return EXIT_FAILURE;}} while(0)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD M_PI / 180.0f
#define RAD2DEG 180.0f / M_PI

/****************************************************************************************
*  Name     : CPE
*  Function : Class constructor, initializes parameters and variables
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ CPE::CPE(
    const CPE_Method cpe_method,                                    // In: Method to be used
    const double dt                                                 // In: Time step of calling function simulation environment
    ) :
    method(cpe_method)
{
    // CE pars

    n_CE = 1000;
    
    sigma_inject = 1.0 / 3.0; // dependent on d_safe wrt obstacle i => set in CE-method

    alpha_n = 0.9;

    // The gate is the CE parameter for the 1 - alpha_p confidense ellipse
    // Predefined inverse chi squared values (chi2inv(p, v) in Matlab) are used here, to
    // escape the need for defining and using chi squared distributions
    // gate = 3.218875824868202;    // for 1 - alpha_p = 0.8
    // gate = 3.794239969771763;    // for 1 - alpha_p = 0.85
    // gate = 4.605170185988092;    // for 1 - alpha_p = 0.9
    // gate = 5.991464547107981;    // for 1 - alpha_p = 0.95
    gate = 11.618285980628054;      // for 1 - alpha_p = 0.997

    rho = 0.9;

    max_it = 10;

    converged_last = false;

    // MCSKF4D pars
    
    n_MCSKF = 500;

    r = 0.001;
    q = 8e-4;

    dt_seg = dt;

    resize_matrices();
}

/****************************************************************************************
*  Name     : initialize
*  Function : Sets up the initial values for the collision probability estimator before
*             before a new run
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ void CPE::initialize(
    const TML::PDVector6d &xs_os,                                                     // In: Own-ship state vector
    const TML::PDVector4d &xs_i,                                                      // In: Obstacle i state vector
    const TML::PDVector16d &P_i,                                                      // In: Obstacle i covariance flattened into n^2 x 1
    const double d_safe_i                                                             // In: Safety zone around own-ship when facing obstacle i
    )
{
    assert(xs_os.get_rows() >= 4 && xs_os.get_rows() <= 6 && xs_i.get_rows() == 4 && P_i.get_rows() == 16 && P_i.get_cols() == 1);

    // The estimation is done considering one obstacle at the time, so the d_safe parameter
    // is initialized accordingly
    d_safe = d_safe_i;
    switch (method)
    {
    case CE:
        converged_last = false;
        // Heuristic initialization
        mu_CE_last = 0.5 * (xs_os.get_block<2, 1>(0, 0, 2, 1) + xs_i.get_block<2, 1>(0, 0, 2, 1)); 
        
        P_CE_last = pow(d_safe, 2) * TML::Matrix2d::identity() / 3.0;
        
        break;
    case MCSKF4D :
        P_c_p = 0; P_c_upd = 0;
        // Ad hoc variance for the probability
        var_P_c_p = 0.3; var_P_c_upd = 0; 
        break;
    default:
        // Throw
        break;
    }
}

/****************************************************************************************
*  Name     : estimate
*  Function : Input parameters have different size depending on if CE or MCSKF4D are used.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ double CPE::estimate(
	const TML::PDMatrix<double, 6, MAX_N_SEG_SAMPLES> &xs_os,                       // In: Own-ship state vector(s)
    const TML::PDMatrix<double, 4, MAX_N_SEG_SAMPLES> &xs_i,                        // In: Obstacle i state vector(s)
    const TML::PDMatrix<double, 16, MAX_N_SEG_SAMPLES> &P_i                          // In: Obstacle i covariance(s), flattened into n² x n_cols
    )
{
    assert(xs_os.get_rows() >= 4 && xs_os.get_rows() <= 6 && xs_i.get_rows() == 4 && P_i.get_rows() == 16);

    TML::Vector2d p_os, p_i;
    TML::Matrix2d P_i_2D;

    double P_c;
    switch (method)
    {
        case CE :
            // The first column gives the current prediction time information
            p_os = xs_os.get_block<2, 1>(0, 0, 2, 1);
            p_i = xs_i.get_block<2, 1>(0, 0, 2, 1);

            P_i_2D = reshape<16, 1, 4, 4>(P_i.get_col(0), 4, 4).get_block<2, 2>(0, 0, 2, 2);

            P_c = CE_estimate(p_os, p_i, P_i_2D);
            break;
        case MCSKF4D :
            P_c = MCSKF4D_estimate(xs_os, xs_i, P_i);
            break;
        default :
            P_c = -1;
            // Throw
            break;
    }
    
    return P_c;
}

/****************************************************************************************
*  Name     : estimate_over_trajectories
*  Function : Takes in own-ship and obstacle trajectories, plus the associated obstacle
*             covariances, and estimates the collision probabilities using a chosen 
*             method.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ void CPE::estimate_over_trajectories(
		TML::PDMatrix<double, 1, MAX_N_SAMPLES> &P_c_i,                              // In/out: Collision probability row vector: 1 x n_samples
		const TML::PDMatrix<double, 6, MAX_N_SAMPLES> &xs_p,                         // In: Ownship predicted trajectory
		const TML::PDMatrix<double, 4, MAX_N_SAMPLES> &xs_i_p,                       // In: Obstacle i predicted trajectory
		const TML::PDMatrix<double, 16, MAX_N_SAMPLES> &P_i_p,                       // In: Obstacle i associated predicted covariances
        const double d_safe_i,                                                       // In: Safety zone around own-ship when facing obstacle i,
        const double dt                                                              // In: Prediction time step
    )
{
    assert(xs_p.get_rows() >= 4 && xs_p.get_rows() <= 6 && xs_i_p.get_rows() == 4 && P_i_p.get_rows() == 16);

    int n_samples = xs_p.get_cols();

    int n_seg_samples = std::round(dt_seg / dt) + 1, k_j_(0), k_j(0);
	TML::PDMatrix<double, 6, MAX_N_SEG_SAMPLES> xs_os_seg(6, n_seg_samples);
    TML::PDMatrix<double, 4, MAX_N_SEG_SAMPLES> xs_i_seg(4, n_seg_samples);
    TML::PDMatrix<double, 16, MAX_N_SEG_SAMPLES> P_i_seg(16, n_seg_samples);

    TML::Vector2d p_os, p_i;
    TML::Matrix2d P_i_2D;
    
    initialize(xs_p.get_col(0), xs_i_p.get_col(0), P_i_p.get_col(0), d_safe_i);

    for (int k = 0; k < n_samples; k++)
    {
        switch(method)
        {
            case CE :	
                p_os = xs_p.get_block<2, 1>(0, 0, 2, 1);
                p_i = xs_i_p.get_block<2, 1>(0, 0, 2, 1);

                P_i_2D = reshape<16, 1, 4, 4>(P_i_p.get_col(0), 4, 4).get_block<2, 2>(0, 0, 2, 2);

                P_c_i(0, k) = CE_estimate(xs_p.get_col(k), xs_i_p.get_col(k), P_i_p.get_col(k));
                break;
            case MCSKF4D :                
                if (fmod(k, n_seg_samples - 1) == 0 && k > 0)
                {
                    k_j_ = k_j; k_j = k;
                    xs_os_seg = xs_p.get_block<6, MAX_N_SEG_SAMPLES>(0, k_j_, 6, n_seg_samples);
                    xs_i_seg = xs_i_p.get_block<4, MAX_N_SEG_SAMPLES>(0, k_j_, 4, n_seg_samples);

                    P_i_seg = P_i_p.get_block<16, MAX_N_SEG_SAMPLES>(0, k_j_, 16, n_seg_samples);

                    P_c_i(0, k_j_) = MCSKF4D_estimate(xs_os_seg, xs_i_seg, P_i_seg);
                    // Collision probability on this active segment are all equal
                    P_c_i.set_block(0, k_j_, 1, n_seg_samples, P_c_i(0, k_j_) * TML::PDMatrix<double, 1, MAX_N_SEG_SAMPLES>::ones(1, k_j - k_j_ + 1));
                }	
                break;
            default :
                // Throw
                break;
        }
    }
}

/****************************************************************************************
	Private functions
****************************************************************************************/
/****************************************************************************************
*  Name     : resize_matrices
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
__host__ __device__ void CPE::resize_matrices()
{
    switch (method)
    {
        case CE :
            samples.resize(2, n_CE);
            elite_samples.resize(2, n_CE);
            valid.resize(1, n_CE);
            L.resize(2, 2);
            break;
        case MCSKF4D :
            samples.resize(4, n_MCSKF);
            valid.resize(1, n_MCSKF);
            L.resize(4, 4);
            break;
        default :
            // Throw
            break; 
    }  
}

/****************************************************************************************
*  Name     : update_L
*  Function : Updates the lower triangular matrix L based on cholesky decomposition 
*             formulas for the input matrix. For 2x2 and 4x4 matrices only, hence 
*             why eigen is not used.
*             Formulas: 
*             L_jj = sqrt(A_jj - sum_k=0^j-1 (L_jk)^2)
*             L_ij = (1 / L_jj) * (A_jj) - sum_k=0^j-1 L_ik * L_jk)
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ inline void CPE::update_L(
    const TML::PDMatrix4d &in                                                     // In: Matrix in consideration
    )
{
    assert(in.get_rows() == in.get_cols());

    int n = in.get_rows();
    L.set_zero();
    double sum;
    for (int i = 0; i < n; i++) { 
        for (int j = 0; j <= i; j++) 
        { 
            sum = 0.0; 
            if (j == i)
            { 
                for (int k = 0; k < j; k++) 
                {
                    sum += pow(L(j, k), 2); 
                }
                L(j, j) = sqrt(in(j, j) - sum); 
            } 
            else if (i > j)
            { 
                for (int k = 0; k < j; k++) 
                {
                    sum += (L(i, k) * L(j, k)); 
                }
                L(i, j) = (in(i, j) - sum) / L(j, j); 
            } 
            else
            {
                L(i, j) = 0;
            }
        } 
    } 
}

/****************************************************************************************
*  Name     : norm_pdf_log
*  Function : Calculates the logarithmic value of the multivariate normal distribution
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ inline void CPE::norm_pdf_log(
    TML::PDMatrix<double, 1, MAX_N_CPE_SAMPLES> &result,                        // In/out: Resulting vector of pdf values
    const TML::PDVector4d &mu,                                                  // In: Expectation of the MVN
    const TML::PDMatrix4d &Sigma                                                // In: Covariance of the MVN
    )
{
    assert(mu.get_rows() <=4 && mu.get_cols() == 1 && Sigma.get_rows() <= 4 && Sigma.get_rows() == Sigma.get_cols() && result.get_rows() == mu.get_rows());

    int n = samples.get_rows(), n_samples = samples.get_cols();

    double exp_val = 0;
    double log_val = - (n / 2.0) * log(2 * M_PI) - log(Sigma.determinant()) / 2.0;
    for (int i = 0; i < n_samples; i++)
    {
        exp_val = (samples.get_col(i) - mu).transposed() * Sigma.inverse() * (samples.get_col(i) - mu);

        exp_val = - exp_val / 2.0;

        result(i) = log_val + exp_val;
    }
}

/****************************************************************************************
*  Name     : generate_norm_dist_samples
*  Function : Generates samples from the MVN distribution with given mean and covariance
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ inline void CPE::generate_norm_dist_samples(
    const TML::PDVector4d &mu,                                                  // In: Expectation of the MVN
    const TML::PDMatrix4d &Sigma                                                // In: Covariance of the MVN
    )
{
    assert(mu.get_rows() <= 4 && mu.get_cols() == 1 && Sigma.get_rows() == Sigma.get_cols() && Sigma.get_rows() <= 4);

    int n = samples.get_rows(), n_samples = samples.get_cols();

    update_L(Sigma);
    
    for (int c = 0; c < n; c++)
    {
        for(int i = 0; i < n_samples; i++)
        {
            samples(c, i) = curand_normal_double(&prng_state);
        }
    }
    // Box-muller transform
    samples = L * samples + mu;
}

/****************************************************************************************
*  Name     : calculate_roots_2nd_order
*  Function : Simple root calculation for a 2nd order polynomial Ax² + Bx + C = 0
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ void CPE::calculate_roots_2nd_order(
    TML::Vector2d &r,                                                   // In: vector of roots to find
    bool &is_complex,                                                   // In: Indicator of real/complex roots
    const double A,                                                     // In: Coefficient in polynomial 
    const double B,                                                     // In: Coefficient in polynomial 
    const double C                                                      // In: Coefficient in polynomial 
    )
{
    assert(r.get_rows() == 2 && r.get_cols() == 1); 

    is_complex = 0;

    double d = pow(B, 2) - 4 * A * C;
    // Distinct real roots
    if (d > 0)          
    {
        r(0) = - (B + sqrt(fabs(d))) / (2 * A);
        r(1) = - (B - sqrt(fabs(d))) / (2 * A);
    }
    // Repeated real roots
    else if (d == 0)
    {
        r(0) = - B / (2 * A);
        r(1) = r(0);
    }
    // Complex conjugated roots, dont care solution
    else
    {
        is_complex = 1;
        r(0) = - 1e10;
        r(1) = 1e10;
    }
}

/****************************************************************************************
*  Name     : produce_MCS_estimate
*  Function : Uses Monte Carlo Simulation to produce a collision probability "measurement"
*             for the MCSKF4D method
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ double CPE::produce_MCS_estimate(
	const TML::Vector4d &xs_i,                                                // In: Obstacle state vector
	const TML::Matrix4d &P_i,                                                 // In: Obstacle covariance
	const TML::Vector2d &p_os_cpa,                                            // In: Position of own-ship at cpa
	const double t_cpa                                                         // In: Time to cpa
    )
{
    assert(xs_i.get_rows() == 4 && xs_i.get_cols() == 1 && P_i.get_rows() == P_i.get_cols() && P_i.get_rows() == 4 && 
            p_os_cpa.get_rows() == 2 && p_os_cpa.get_cols() == 1);

    double P_c;

    generate_norm_dist_samples(xs_i, P_i);
    
    determine_sample_validity_4D(p_os_cpa, t_cpa);

    // The estimate is taken as the ratio of samples inside the integration domain, 
    // to the total number of samples, i.e. the mean of the validity vector
    P_c = valid.rwise_mean()(0);
    if (P_c > 1) { return 1; }
    else         { return P_c; }      
}

/****************************************************************************************
*  Name     : determine_sample_validity_4D
*  Function : Determine if a sample is valid for use in estimation for the MCSKF4D.
*             See "On Collision Risk Assessment for Autonomous Ships Using Scenario-based"
*             MPC for more information.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ void CPE::determine_sample_validity_4D(
    const TML::Vector2d &p_os_cpa,                                           // In: Position of own-ship at cpa
    const double t_cpa                                                       // In: Time to cpa
    )
{
    assert(p_os_cpa.get_rows() == 1 && p_os_cpa.get_cols() == 1);

    int n_samples = samples.get_cols();

    bool complex_roots;
    TML::Vector2d p_i_sample, v_i_sample;
    double A, B, C;
    TML::Vector2d r;
    for (int j = 0; j < n_samples; j++)
    {
        valid(j) = 0;

        p_i_sample = samples.get_block<2, 1>(0, j, 2, 1);
        v_i_sample = samples.get_block<2, 1>(2, j, 2, 1);

        A = v_i_sample.dot(v_i_sample);
        B = 2 * (p_i_sample - p_os_cpa).transposed() * v_i_sample;
        C = p_i_sample.dot(p_i_sample) - 2 * p_os_cpa.dot(p_i_sample) + p_os_cpa.dot(p_os_cpa) - pow(d_safe, 2);

        calculate_roots_2nd_order(r, complex_roots, A, B, C);

        // Distinct real positive or pos+negative roots: 2 crossings, possibly only one in
        // t >= t0, checks if t_cpa occurs inside this root interval <=> inside safety zone
        if (!complex_roots && r(0) != r(1) && t_cpa >= 0 && (r(0) <= t_cpa && t_cpa <= r(1)))
        {
            valid(j) = 1;
        }
        // Repetitive real positive roots: 1 crossing, this is only possible if the sampled
        // trajectory is tangent to the safety zone at t_cpa, checks if t_cpa = cross time
        // in this case
        else if(!complex_roots && r(0) == r(1) && t_cpa >= 0 && t_cpa == r(0))
        {
            valid(j) = 1;
        }
        // Negative roots are not considered, as that implies going backwards in time..
    }
}

/****************************************************************************************
*  Name     : MCSKF4D_estimation
*  Function : Collision probability estimation using Monte Carlo Simulation and 
*             Kalman-filtering, considering the 4D obstacle uncertainty along piece-wise
*             linear segments of the vessel trajectories. See "Risk-based Autonomous 
*             Maritime Collision Avoidance Considering Obstacle Intentions" for more info.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ double CPE::MCSKF4D_estimate(
	const TML::PDMatrix<double, 6, MAX_N_SEG_SAMPLES> &xs_os,                                               // In: Own-ship states for the active segment
    const TML::PDMatrix<double, 4, MAX_N_SEG_SAMPLES> &xs_i,                                                // In: Obstacle i states for the active segment
    const TML::PDMatrix<double, 16, MAX_N_SEG_SAMPLES> &P_i                                                 // In: Obstacle i covariance for the active segment
    )
{
    assert(xs_os.get_rows() >= 4 && xs_os.get_rows() <= 6 && xs_i.get_rows() == 4 && P_i.get_rows() == 16);

    // Collision probability "measurement" from MCS
    double y_P_c_i;

    /*****************************************************
    * Find linear segment representative state vectors
    *****************************************************/
   int n_seg_samples = xs_os.get_cols();
    // Vectors representing the linear segments
    TML::Vector4d xs_os_sl, xs_i_sl;
    xs_os_sl = xs_os.get_col(0);
    xs_i_sl = xs_i.get_col(0);

    if (n_seg_samples > 1)
    {
        // Define speed and course for the vessels along their linear segments
        double U_os_sl, U_i_sl, psi_os_sl, psi_i_sl;
    
        // Own-ship segment
        // Find average velocity along segment
        U_os_sl = xs_os.get_block<2, MAX_N_SEG_SAMPLES>(3, 0, 2, n_seg_samples).rwise_mean().norm();
        // Find angle of the segment
        psi_os_sl = atan2(xs_os(1, n_seg_samples - 1) - xs_os(1, 0), xs_os(0, n_seg_samples - 1) - xs_os(0, 0));
        // Set initial position to be that of the own-ship at the start of the segment
        xs_os_sl(0) = xs_os(0, 0); 
        xs_os_sl(1) = xs_os(1, 0);
        // Rotate velocity vector to be parallel to the straight line
        xs_os_sl(2) = U_os_sl * cos(psi_os_sl);
        xs_os_sl(3) = U_os_sl * sin(psi_os_sl);

        // Obstacle segment
        // Same procedure as every year James
        U_i_sl = xs_i.get_block<2, MAX_N_SEG_SAMPLES>(2, 0, 2, n_seg_samples).rwise_mean().norm();
        psi_i_sl = atan2(xs_i(1, n_seg_samples - 1) - xs_i(1, 0), xs_i(0, n_seg_samples - 1) - xs_i(0, 0));

        xs_i_sl(0) = xs_i(0, 0); 
        xs_i_sl(1) = xs_i(1, 0);
        xs_i_sl(2) = U_i_sl * cos(psi_i_sl);
        xs_i_sl(3) = U_i_sl * sin(psi_i_sl);
    }
    TML::Matrix4d P_i_sl;
    P_i_sl = reshape<16, MAX_N_SEG_SAMPLES, 4, 4>(P_i.get_col(0), 4, 4);

    TML::Vector2d p_os_cpa;
    double t_cpa, d_cpa;
    calculate_cpa(p_os_cpa, t_cpa, d_cpa, xs_os_sl, xs_i_sl);

    /*****************************************************
    * Generate Monte Carlo Simulation estimate of P_c
    * for use in the Kalman-filter
    *****************************************************/
    // Constrain the collision probability estimation to the interval [t_j-1, t_j], 
    // which is of length dt_seg. This is done to only consider vessel positions on
    // their discretized trajectories, and not beyond that. 
    if (t_cpa > dt_seg)
    {
        y_P_c_i = produce_MCS_estimate(xs_i_sl, P_i_sl, xs_os.get_block<2, 1>(0, n_seg_samples - 1, 2, 1), dt_seg);
    }
    else
    {
        y_P_c_i = produce_MCS_estimate(xs_i_sl, P_i_sl, p_os_cpa, t_cpa);
    }

    /*****************************************************
    * Kalman-filtering for simple markov chain model
    * P_c^i_k+1 = P_c^i_k + v^i_k
    * y^i_k     = P_c^i_k + w^i_k
    ******************************************************
    * Update using measurement y_P_c
    *****************************************************/
    double K;
    K = var_P_c_p / (var_P_c_p + r);

    P_c_upd = P_c_p + K * (y_P_c_i - P_c_p);

    if (P_c_upd > 1) P_c_upd = 1;

    var_P_c_upd = (1 - K) * var_P_c_p;

    /*****************************************************
    * Predict
    *****************************************************/
    P_c_p = P_c_upd;
    if (P_c_p > 1) P_c_p = 1;

    var_P_c_p = var_P_c_upd + q;
    //*****************************************************
    
    return P_c_upd;
}

/****************************************************************************************
*  Name     : determine_sample_validity_2D
*  Function : Determine valid samples for 2D collision probability estimation methods
*             (CE-method)
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ void CPE::determine_sample_validity_2D(
	const TML::Vector2d &p_os                                                // In: Own-ship position vector
    )
{
    assert(p_os.get_rows() == 2 && p_os.get_cols() == 1);

    int n_samples = samples.get_cols();
    bool inside_safety_zone;
    for (int j = 0; j < n_samples; j++)
    {
        valid(j) = 0;
        inside_safety_zone = (samples.get_col(j) - p_os).dot(samples.get_col(j) - p_os) <= pow(d_safe, 2);
        if (inside_safety_zone)
        {
            valid(j) = 1;
        }
    }
}

/****************************************************************************************
*  Name     : determine_best_performing_samples
*  Function : Determine the elite samples for the CE method for a given iteration
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ void CPE::determine_best_performing_samples(
    const TML::Vector2d &p_os,                                                    // In: Own-ship position vector
    const TML::Vector2d &p_i,                                                     // In: Obstacle i position vector
    const TML::Matrix2d &P_i                                                      // In: Obstacle i positional covariance
    )
{
    assert(p_os.get_rows() == 2 && p_os.get_cols() == 1 && p_i.get_rows() == 2 && p_i.get_cols() == 1 &&
            P_i.get_rows() == 2 && P_i.get_rows() == P_i.get_cols());

    N_e = 0;
    bool inside_safety_zone, inside_alpha_p_confidence_ellipse;
    for (int j = 0; j < n_CE; j++)
    {
        valid(j) = 0;
        inside_safety_zone = (samples.get_col(j) - p_os).dot(samples.get_col(j) - p_os) <= pow(d_safe, 2);

        inside_alpha_p_confidence_ellipse = 
            (samples.get_col(j) - p_i).transposed() * P_i.inverse() * (samples.get_col(j) - p_i) <= gate;


        if (inside_safety_zone && inside_alpha_p_confidence_ellipse)
        {
            valid(j) = 1;
            N_e++;
        }
    }
}

/****************************************************************************************
*  Name     : CE_estimation
*  Function : Collision probability estimation using the Cross-Entropy method
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ double CPE::CE_estimate(
	const TML::Vector2d &p_os,                                                // In: Own-ship position vector
    const TML::Vector2d &p_i,                                                 // In: Obstacle i position vector
    const TML::Matrix2d &P_i                                                  // In: Obstacle i positional covariance
    )
{            
    double P_c(0.0);
    /******************************************************************************
    * Check if it is necessary to perform estimation
    ******************************************************************************/
    double d_0i = (p_i - p_os).norm();
    double var_P_i_largest(0.0);
    
    if (P_i(0, 0) > P_i(1, 1)) { var_P_i_largest = P_i(0, 0); }
    else                       { var_P_i_largest = P_i(1, 1); }

    // This large a distance usually means no effective conflict zone, as
    // approx 99.7% of probability mass inside 3.5 * standard deviations
    // (assuming equal std dev in x, y (assumption))
    if (d_0i > d_safe + 3.5 * sqrt(var_P_i_largest)) 
    { 
        return P_c; 
    }

    /******************************************************************************
    * Convergence depedent initialization prior to the run at time t_k
    ******************************************************************************/
    TML::Vector2d mu_CE_prev, mu_CE;
    TML::Matrix2d P_CE_prev, P_CE;
    sigma_inject = d_safe / 3;
    if (converged_last)
    {
        mu_CE_prev = mu_CE_last; mu_CE = mu_CE_last;

        P_CE_prev = P_CE_last; 
        P_CE = P_CE_last + pow(sigma_inject, 2) * TML::Matrix2d::identity();
    }
    else
    {
        mu_CE_prev = 0.5 * (p_i + p_os); mu_CE = mu_CE_prev;
        P_CE_prev = pow(d_safe, 2) * TML::Matrix2d::identity() / 3;
        P_CE = P_CE_prev;
    }

    /******************************************************************************
    * Iterative optimization to find the near-optimal importance density
    ******************************************************************************/
    for (int it = 0; it < max_it; it++)
    {
        printf("hereinCEopt\n");
        generate_norm_dist_samples(mu_CE, P_CE);

        determine_best_performing_samples(p_os, p_i, P_i);

        elite_samples.resize(2, N_e);
        e_count = 0;
        for (int j = 0; j < n_CE; j++)
        {
            if (valid(j) == 1)
            {
                elite_samples.set_col(e_count, samples.get_col(j));
                e_count++;
            }
        }

        // Terminate iterative optimization if enough elite samples are collected
        if (N_e >= n_CE * rho) { converged_last = true; break;}
        // Otherwise, improve importance density parameters (given N_e > 3 to prevent zero-matrix 
        // in P_CE and/or negative definite matrix if no smoothing is used, and Pcoll spikes due
        // to insufficient sample amounts)
        else if (N_e > 3)
        {
            mu_CE_prev = mu_CE;
            P_CE_prev = P_CE;

            mu_CE = elite_samples.rwise_mean();

            P_CE.set_zero();
            for (int j = 0; j < N_e; j++)
            {
                P_CE += (elite_samples.get_col(j) - mu_CE) * (elite_samples.get_col(j) - mu_CE).transposed();
            }
            P_CE = P_CE / (double)N_e;

            // Smoothing to aid in preventing degeneration
            mu_CE = alpha_n * mu_CE + (1 - alpha_n) * mu_CE_prev;
            P_CE =  alpha_n * P_CE  + (1 - alpha_n) * P_CE_prev;
        }
    }
    mu_CE_last = mu_CE; P_CE_last = P_CE;

    /******************************************************************************
    * Estimate collision probability with samples from the final importance
    * density from the optimization
    ******************************************************************************/
    printf("hereafterCEopt\n");
    generate_norm_dist_samples(mu_CE, P_CE);

    determine_sample_validity_2D(p_os);

    TML::PDMatrix<double, 1, MAX_N_CPE_SAMPLES> weights(1, n_CE), integrand(1, n_CE), importance(1, n_CE);
    norm_pdf_log(integrand, p_i, P_i);
    norm_pdf_log(importance, mu_CE, P_CE);
    
    // Calculate importance weights for estimating the integral \Int_S_2 {p^i(x, y, t_k) dx dy}
    // where p^i = Norm_distr(p_i, P_i; t_k) is the obstacle positional uncertainty (or combined uncertainty
    // if the own-ship uncertainty is also considered). 
    // Divide using log-values as this is more robust against underflow
    weights = (integrand - importance).exp();
    weights = weights.cwise_product(valid);

    P_c = weights.rwise_mean();
    if (P_c > 1) return 1;
    else return P_c;
}