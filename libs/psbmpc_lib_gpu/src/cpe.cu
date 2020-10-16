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
    const float dt                                                 // In: Time step of calling function simulation environment
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
    const TML::PDVector6f &xs_os,                                                     // In: Own-ship state vector
    const TML::PDVector4f &xs_i,                                                      // In: Obstacle i state vector
    const TML::PDVector16f &P_i,                                                      // In: Obstacle i covariance flattened into n^2 x 1
    const float d_safe_i                                                             // In: Safety zone around own-ship when facing obstacle i
    )
{
    // The estimation is done considering one obstacle at the time, so the d_safe parameter
    // is initialized accordingly
    d_safe = d_safe_i;
    switch (method)
    {
    case CE:
        converged_last = false;
        // Heuristic initialization
        mu_CE_last = 0.5 * (xs_os.get_block<2, 1>(0, 0, 2, 1) + xs_i.get_block<2, 1>(0, 0, 2, 1)); 
        
        P_CE_last = powf(d_safe, 2) * TML::Matrix2f::identity() / 3.0;
        
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
*             NOT USED CURRENTLY.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ float CPE::estimate(
	const TML::PDMatrix<float, 6, MAX_N_SEG_SAMPLES> &xs_os,                       // In: Own-ship state vector(s)
    const TML::PDMatrix<float, 4, MAX_N_SEG_SAMPLES> &xs_i,                        // In: Obstacle i state vector(s)
    const TML::PDMatrix<float, 16, MAX_N_SEG_SAMPLES> &P_i                          // In: Obstacle i covariance(s), flattened into n² x n_cols
    )
{
    n_seg_samples = xs_os.get_cols();
    TML::Vector2f p_os, p_i;
    TML::Matrix2f P_i_2D;

    P_c_est = 0.0;
    switch (method)
    {
        case CE :
            // The last column gives the current prediction time information
            p_os = xs_os.get_block<2, 1>(0, n_seg_samples - 1, 2, 1);
            p_i = xs_i.get_block<2, 1>(0, n_seg_samples - 1, 2, 1);

            P_i_2D = reshape<16, 1, 4, 4>(P_i.get_col(n_seg_samples - 1), 4, 4).get_block<2, 2>(0, 0, 2, 2);

            P_c_est = CE_estimate(p_os, p_i, P_i_2D);
            break;
        case MCSKF4D :
            P_c_est = MCSKF4D_estimate(xs_os, xs_i, P_i);
            break;
        default :
            P_c_est = -1;
            // Throw
            break;
    }
    
    return P_c_est;
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
		TML::PDMatrix<float, 1, MAX_N_SAMPLES> &P_c_i,                              // In/out: Collision probability row vector: 1 x n_samples
		const TML::PDMatrix<float, 6, MAX_N_SAMPLES> &xs_p,                         // In: Ownship predicted trajectory
		const TML::PDMatrix<float, 4, MAX_N_SAMPLES> &xs_i_p,                       // In: Obstacle i predicted trajectory
		const TML::PDMatrix<float, 16, MAX_N_SAMPLES> &P_i_p,                       // In: Obstacle i associated predicted covariances
        const float d_safe_i,                                                       // In: Safety zone around own-ship when facing obstacle i,
        const float dt                                                              // In: Prediction time step
    )
{
    int n_traj_samples = xs_p.get_cols();

    int n_seg_samples_temp = std::round(dt_seg / dt) + 1, k_j_(0), k_j(0);
	TML::PDMatrix<float, 6, MAX_N_SEG_SAMPLES> xs_os_seg(6, n_seg_samples_temp);
    TML::PDMatrix<float, 4, MAX_N_SEG_SAMPLES> xs_i_seg(4, n_seg_samples_temp);
    TML::PDMatrix<float, 16, MAX_N_SEG_SAMPLES> P_i_seg(16, n_seg_samples_temp);
    
    initialize(xs_p.get_col(0), xs_i_p.get_col(0), P_i_p.get_col(0), d_safe_i);

    for (int k = 0; k < n_traj_samples; k++)
    {
        switch(method)
        {
            case CE :	
                p_os = xs_p.get_block<2, 1>(0, k, 2, 1);
                p_i = xs_i_p.get_block<2, 1>(0, k, 2, 1);

                P_i_2D = reshape<16, 1, 4, 4>(P_i_p.get_col(k), 4, 4).get_block<2, 2>(0, 0, 2, 2);

                P_c_i(0, k) = CE_estimate(p_os, p_i, P_i_2D);
                break;
            case MCSKF4D :                
                if (fmod(k, n_seg_samples_temp - 1) == 0 && k > 0)
                {
                    k_j_ = k_j; k_j = k;
                    xs_os_seg = xs_p.get_block<6, MAX_N_SEG_SAMPLES>(0, k_j_, 6, n_seg_samples_temp);
                    xs_i_seg = xs_i_p.get_block<4, MAX_N_SEG_SAMPLES>(0, k_j_, 4, n_seg_samples_temp);

                    P_i_seg = P_i_p.get_block<16, MAX_N_SEG_SAMPLES>(0, k_j_, 16, n_seg_samples_temp);

                    P_c_i(0, k_j_) = MCSKF4D_estimate(xs_os_seg, xs_i_seg, P_i_seg);
                    // Collision probability on this active segment are all equal
                    P_c_i.set_block(0, k_j_, 1, n_seg_samples_temp, P_c_i(0, k_j_) * TML::PDMatrix<float, 1, MAX_N_SEG_SAMPLES>::ones(1, k_j - k_j_ + 1));
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
    const TML::PDMatrix4f &in                                                     // In: Matrix in consideration
    )
{
    n = in.get_rows();
    L.set_zero();
    for (int i = 0; i < n; i++) { 
        for (int j = 0; j <= i; j++) 
        { 
            sum = 0.0; 
            if (j == i)
            { 
                for (int k = 0; k < j; k++) 
                {
                    sum += powf(L(j, k), 2); 
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
    TML::PDMatrix<float, 1, MAX_N_CPE_SAMPLES> &result,                        // In/out: Resulting vector of pdf values
    const TML::PDVector4f &mu,                                                  // In: Expectation of the MVN
    const TML::PDMatrix4f &Sigma                                                // In: Covariance of the MVN
    )
{
    n = samples.get_rows();
    n_samples = samples.get_cols();
    
    exp_val = 0;
    log_val = - (n / 2.0) * log(2 * M_PI) - log(Sigma.determinant()) / 2.0;
    Sigma_inverse = Sigma.inverse();
    for (int i = 0; i < n_samples; i++)
    {
        exp_val = (samples.get_col(i) - mu).transposed() * Sigma_inverse * (samples.get_col(i) - mu);

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
    const TML::PDVector4f &mu,                                                  // In: Expectation of the MVN
    const TML::PDMatrix4f &Sigma                                                // In: Covariance of the MVN
    )
{
    n = samples.get_rows();
    n_samples = samples.get_cols();

    update_L(Sigma);
    
    for (int c = 0; c < n; c++)
    {
        for(int i = 0; i < n_samples; i++)
        {
            samples(c, i) = curand_normal(&prng_state);
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
__device__ void CPE::calculate_roots_2nd_order()
{
    complex_roots = false;

    d = powf(B, 2) - 4 * A * C;
    // Distinct real roots
    if (d > 0)          
    {
        roots(0) = - (B + sqrt(fabs(d))) / (2 * A);
        roots(1) = - (B - sqrt(fabs(d))) / (2 * A);
    }
    // Repeated real roots
    else if (d == 0)
    {
        roots(0) = - B / (2 * A);
        roots(1) = roots(0);
    }
    // Complex conjugated roots, dont care solution
    else
    {
        complex_roots = true;
        roots(0) = - 1e10;
        roots(1) = 1e10;
    }
}

/****************************************************************************************
*  Name     : produce_MCS_estimate
*  Function : Uses Monte Carlo Simulation to produce a collision probability "measurement"
*             for the MCSKF4D method
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ float CPE::produce_MCS_estimate(
	const TML::Vector4f &xs_i,                                                // In: Obstacle state vector
	const TML::Matrix4f &P_i,                                                 // In: Obstacle covariance
	const TML::Vector2f &p_os_cpa,                                            // In: Position of own-ship at cpa
	const float t_cpa                                                         // In: Time to cpa
    )
{
    y_P_c = 0.0;

    generate_norm_dist_samples(xs_i, P_i);
    
    determine_sample_validity_4D(p_os_cpa, t_cpa);

    // The estimate is taken as the ratio of samples inside the integration domain, 
    // to the total number of samples, i.e. the mean of the validity vector
    y_P_c = valid.rwise_mean();
    if (y_P_c > 1) { return 1; }
    else         { return y_P_c; }      
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
    const TML::Vector2f &p_os_cpa,                                           // In: Position of own-ship at cpa
    const float t_cpa                                                       // In: Time to cpa
    )
{
    n_samples = samples.get_cols();

    for (int j = 0; j < n_samples; j++)
    {
        valid(j) = 0;

        p_i_sample = samples.get_block<2, 1>(0, j, 2, 1);
        v_i_sample = samples.get_block<2, 1>(2, j, 2, 1);

        A = v_i_sample.dot(v_i_sample);
        B = 2 * (p_i_sample - p_os_cpa).transposed() * v_i_sample;
        C = p_i_sample.dot(p_i_sample) - 2 * p_os_cpa.dot(p_i_sample) + p_os_cpa.dot(p_os_cpa) - powf(d_safe, 2);

        calculate_roots_2nd_order();

        // Distinct real positive or pos+negative roots: 2 crossings, possibly only one in
        // t >= t0, checks if t_cpa occurs inside this root interval <=> inside safety zone
        if (!complex_roots && roots(0) != roots(1) && t_cpa >= 0 && (roots(0) <= t_cpa && t_cpa <= roots(1)))
        {
            valid(j) = 1;
        }
        // Repetitive real positive roots: 1 crossing, this is only possible if the sampled
        // trajectory is tangent to the safety zone at t_cpa, checks if t_cpa = cross time
        // in this case
        else if(!complex_roots && roots(0) == roots(1) && t_cpa >= 0 && t_cpa == roots(0))
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
__device__ float CPE::MCSKF4D_estimate(
	const TML::PDMatrix<float, 6, MAX_N_SEG_SAMPLES> &xs_os,                                               // In: Own-ship states for the active segment
    const TML::PDMatrix<float, 4, MAX_N_SEG_SAMPLES> &xs_i,                                                // In: Obstacle i states for the active segment
    const TML::PDMatrix<float, 16, MAX_N_SEG_SAMPLES> &P_i                                                 // In: Obstacle i covariance for the active segment
    )
{
    /*****************************************************
    * Find linear segment representative state vectors
    *****************************************************/
    n_seg_samples = xs_os.get_cols();
    // Vectors representing the linear segments
    
    // First column is the start of the segment
    xs_os_sl = xs_os.get_block<4, 1>(0, 0, 4, 1);
    xs_i_sl = xs_i.get_col(0);

    if (n_seg_samples > 1)
    {    
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
    P_i_sl = reshape<16, MAX_N_SEG_SAMPLES, 4, 4>(P_i.get_col(0), 4, 4);

    printf("xs_os_sl = %.1f, %.1f, %.1f, %.1f\n", xs_os_sl(0, 0), xs_os_sl(1, 0), xs_os_sl(2, 0), xs_os_sl(3, 0));
    printf("xs_i_sl = %.1f, %.1f, %.1f, %.1f\n", xs_i_sl(0, 0), xs_i_sl(1, 0), xs_i_sl(2, 0), xs_i_sl(3, 0));

    calculate_cpa(p_os_cpa, t_cpa, d_cpa, xs_os_sl, xs_i_sl);

    printf("p_os_cpa = %.1f, %.1f\n", p_os_cpa(0), p_os_cpa(1));
    printf("t_cpa = %.1f\n", t_cpa);

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
    printf("y_P_c_i = %.1f\n", y_P_c_i);

    /*****************************************************
    * Kalman-filtering for simple markov chain model
    * P_c^i_k+1 = P_c^i_k + v^i_k
    * y^i_k     = P_c^i_k + w^i_k
    ******************************************************
    * Update using measurement y_P_c
    *****************************************************/
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
	const TML::Vector2f &p_os                                                // In: Own-ship position vector
    )
{
    n_samples = samples.get_cols();
    for (int j = 0; j < n_samples; j++)
    {
        valid(j) = 0;
        inside_safety_zone = (samples.get_col(j) - p_os).dot(samples.get_col(j) - p_os) <= powf(d_safe, 2);
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
    const TML::Vector2f &p_os,                                                    // In: Own-ship position vector
    const TML::Vector2f &p_i,                                                     // In: Obstacle i position vector
    const TML::Matrix2f &P_i                                                      // In: Obstacle i positional covariance
    )
{
    N_e = 0;
    for (int j = 0; j < n_CE; j++)
    {
        valid(j) = 0;
        inside_safety_zone = (samples.get_col(j) - p_os).dot(samples.get_col(j) - p_os) <= powf(d_safe, 2);

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
*  Name     : update_importance_density
*  Function : Improves the current importance density in the Cross-Entropy iterative
*             optimization.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ void CPE::update_importance_density(
    TML::Vector2f &mu_CE,                                                   // In/out: Expectation of the current importance density
    TML::Matrix2f &P_CE,                                                    // In/out: Covariance of the current importance density
    TML::Vector2f &mu_CE_prev,                                              // In/out: Expectation of the previous importance density
    TML::Matrix2f &P_CE_prev                                                // In/out: Covariance of the previous importance density
    )
{
    // Set previous importance density parameters to the old current ones
    mu_CE_prev = mu_CE;
    P_CE_prev = P_CE;

    // Update the current parameters using the elite samples
    mu_CE = elite_samples.rwise_mean();

    P_CE.set_zero();
    for (int j = 0; j < N_e; j++)
    {
        P_CE += (elite_samples.get_col(j) - mu_CE) * (elite_samples.get_col(j) - mu_CE).transposed();
    }
    P_CE /= (float)N_e;

    // Smoothing to aid in preventing degeneration
    mu_CE = alpha_n * mu_CE + (1 - alpha_n) * mu_CE_prev;
    P_CE =  alpha_n * P_CE  + (1 - alpha_n) * P_CE_prev;
}

/****************************************************************************************
*  Name     : CE_estimation
*  Function : Collision probability estimation using the Cross-Entropy method
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ float CPE::CE_estimate(
	const TML::Vector2f &p_os,                                                // In: Own-ship position vector
    const TML::Vector2f &p_i,                                                 // In: Obstacle i position vector
    const TML::Matrix2f &P_i                                                  // In: Obstacle i positional covariance
    )
{            
    P_c_CE = 0.0;
    /******************************************************************************
    * Check if it is necessary to perform estimation
    ******************************************************************************/
    d_0i = (p_i - p_os).norm();
    
    if (P_i(0, 0) > P_i(1, 1)) { var_P_i_largest = P_i(0, 0); }
    else                       { var_P_i_largest = P_i(1, 1); }

    // This large a distance usually means no effective conflict zone, as
    // approx 99.7% of probability mass inside 3.5 * standard deviations
    // (assuming equal std dev in x, y (assumption))
    if (d_0i > d_safe + 3.5 * sqrt(var_P_i_largest)) 
    { 
        return P_c_CE; 
    }

    /******************************************************************************
    * Convergence depedent initialization prior to the run at time t_k
    ******************************************************************************/
    sigma_inject = d_safe / 3;
    if (converged_last)
    {
        mu_CE_prev = mu_CE_last; mu_CE = mu_CE_last;

        P_CE_prev = P_CE_last; 
        P_CE = P_CE_last + powf(sigma_inject, 2) * TML::Matrix2f::identity();
    }
    else
    {
        mu_CE_prev = p_i + p_os; mu_CE_prev *= 0.5; 
        mu_CE = mu_CE_prev;

        P_CE_prev = powf(d_safe, 2) * TML::Matrix2f::identity(); P_CE_prev /= (float)3;
        P_CE = P_CE_prev;
    }
    /* printf("mu_CE = %.1f, %.1f\n", mu_CE(0), mu_CE(1));

    printf("P_CE = %.1f, %.1f\n", P_CE(0, 0), P_CE(0, 1));
	printf("	   %.1f, %.1f\n", P_CE(1, 0), P_CE(1, 1)); */

    /******************************************************************************
    * Iterative optimization to find the near-optimal importance density
    ******************************************************************************/
    for (int it = 0; it < max_it; it++)
    {
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
        if (N_e >= n_CE * rho) { converged_last = true; break; }
        // Otherwise, improve importance density parameters (given N_e > 3 to prevent zero-matrix 
        // in P_CE and/or negative definite matrix if no smoothing is used, and Pcoll spikes due
        // to insufficient sample amounts)
        else if (N_e > 3)
        {
            update_importance_density(mu_CE, P_CE, mu_CE_prev, P_CE_prev);
        }
    }
    mu_CE_last = mu_CE; P_CE_last = P_CE;

    /******************************************************************************
    * Estimate collision probability with samples from the final importance
    * density from the optimization
    ******************************************************************************/
    generate_norm_dist_samples(mu_CE, P_CE);
    
    determine_sample_validity_2D(p_os);

    TML::PDMatrix<float, 1, MAX_N_CPE_SAMPLES> weights(1, n_CE), integrand(1, n_CE), importance(1, n_CE);

    norm_pdf_log(integrand, p_i, P_i);
    norm_pdf_log(importance, mu_CE, P_CE);

    // Calculate importance weights for estimating the integral \Int_S_2 {p^i(x, y, t_k) dx dy}
    // where p^i = Norm_distr(p_i, P_i; t_k) is the obstacle positional uncertainty (or combined uncertainty
    // if the own-ship uncertainty is also considered). 
    // Divide using log-values as this is more robust against underflow
    weights = (integrand - importance).exp();
    weights = weights.cwise_product(valid);

    P_c_CE = weights.rwise_mean();
    if (P_c_CE > 1) return 1;
    else return P_c_CE;

    return 0;
}