/****************************************************************************************
*
*  File name : cpe_gpu.cu
*
*  Function  : Class functions for the GPU collision probability estimator
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

#include "gpu/cpe_gpu.cuh"
#include "gpu/utilities_gpu.cuh"
#include <stdio.h>
#include "assert.h"

namespace PSBMPC_LIB
{    
namespace GPU
{

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

    n_CE = 500;
    
    sigma_inject = 1.0f / 3.0f; // dependent on d_safe wrt obstacle i => set in CE-method

    alpha_n = 0.9f;

    // The gate is the CE parameter for the 1 - alpha_p confidense ellipse
    // Predefined inverse chi squared values (chi2inv(p, v) in Matlab) are used here, to
    // escape the need for defining and using chi squared distributions
    // gate = 3.218875824868202;    // for 1 - alpha_p = 0.8
    // gate = 3.794239969771763;    // for 1 - alpha_p = 0.85
    // gate = 4.605170185988092;    // for 1 - alpha_p = 0.9
    // gate = 5.991464547107981;    // for 1 - alpha_p = 0.95
    gate = 11.618285980628054f;      // for 1 - alpha_p = 0.997

    rho = 0.9f;

    max_it = 10;

    converged_last = false;

    // MCSKF4D pars
    
    n_MCSKF = 500;

    r = 0.001f;
    q = 8e-4f;

    dt_seg = dt;

    d_safe = 50.0f;

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
    const TML::PDVector4f &xs_os,                                                     // In: Own-ship state vector
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
        mu_CE_last = xs_os.get_block<2, 1>(0, 0, 2, 1) + xs_i.get_block<2, 1>(0, 0, 2, 1);
        mu_CE_last *= 0.5f; 
        
        P_CE_last = powf(d_safe, 2) * TML::Matrix2f::identity() / 3.0f;
        
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
            valid.resize(1, n_CE);
            L.resize(2, 2);
            weights.resize(1, n_CE); pdf_values.resize(1, n_CE);
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
*             why eigen is not used. Two overloads for CE and MCSKF4D, respectively.
*             Formulas: 
*             L_jj = sqrt(A_jj - sum_k=0^j-1 (L_jk)^2)
*             L_ij = (1 / L_jj) * (A_jj) - sum_k=0^j-1 L_ik * L_jk)
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ void CPE::update_L(
    const TML::Matrix2f &in                                                     // In: Matrix in consideration
    )
{
    n = 2;
    L.set_zero();
    for (int i = 0; i < n; i++) 
    { 
        for (int j = 0; j <= i; j++) 
        { 
            sum = 0.0f; 
            if (j == i)
            { 
                for (int k = 0; k < j; k++) 
                {
                    sum += powf(L(j, k), 2); 
                }
                L(j, j) = sqrtf(in(j, j) - sum); 
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
                L(i, j) = 0.0f;
            }
        } 
    } 
}
__host__ __device__ void CPE::update_L(
    const TML::Matrix4f &in                                                     // In: Matrix in consideration
    )
{
    n = 4;
    L.set_zero();
    for (int i = 0; i < n; i++) 
    { 
        for (int j = 0; j <= i; j++) 
        { 
            sum = 0.0f; 
            if (j == i)
            { 
                for (int k = 0; k < j; k++) 
                {
                    sum += powf(L(j, k), 2); 
                }
                L(j, j) = sqrtf(in(j, j) - sum); 
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
                L(i, j) = 0.0f;
            }
        } 
    } 
}

/****************************************************************************************
*  Name     : norm_pdf_log
*  Function : Calculates the logarithmic value of the multivariate normal distribution.
*             Only used by CE-method so-far.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ inline void CPE::norm_pdf_log(
    TML::PDMatrix<float, 1, MAX_N_CPE_SAMPLES> &result,                       // In/out: Resulting vector of pdf values
    const TML::Vector2d &mu,                                                  // In: Expectation of the MVN
    const TML::Matrix2d &Sigma                                                // In: Covariance of the MVN
    )
{
    n = samples.get_rows();
    n_samples = samples.get_cols();
    
    exp_val = 0.0;
    log_val = - ((double)n / 2.0) * log(2.0 * M_PI) - log(Sigma.determinant()) / 2.0;
    Sigma_2D_inv = Sigma.inverse();
    for (int i = 0; i < n_samples; i++)
    {
        sample_innovation = samples.get_col(i) - mu;

        exp_val = sample_innovation.transposed() * Sigma_2D_inv * sample_innovation;
        exp_val = - exp_val / 2.0;

        result(i) = log_val + exp_val;
    }
}

/****************************************************************************************
*  Name     : generate_norm_dist_samples
*  Function : Generates samples from the MVN distribution with given mean and covariance.
*             Two overloads for CE and MCSKF4D, respectively.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ inline void CPE::generate_norm_dist_samples(
    const TML::Vector2f &mu,                                                  // In: Expectation of the MVN
    const TML::Matrix2f &Sigma                                                // In: Covariance of the MVN
    )
{
    n = 2;
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

__device__ inline void CPE::generate_norm_dist_samples(
    const TML::Vector4f &mu,                                                  // In: Expectation of the MVN
    const TML::Matrix4f &Sigma                                                // In: Covariance of the MVN
    )
{
    n = 4;
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
__host__ __device__ void CPE::calculate_roots_2nd_order()
{
    complex_roots = false;

    d = pow(B, 2) - 4 * A * C;
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
    y_P_c = 0.0f;

    generate_norm_dist_samples(xs_i, P_i);
    
    determine_sample_validity_4D(p_os_cpa, (double)t_cpa);

    // The estimate is taken as the ratio of samples inside the integration domain, 
    // to the total number of samples, i.e. the mean of the validity vector
    y_P_c = valid.rwise_mean();
    if (y_P_c > 1.0f)   { return 1.0f; }
    else                { return y_P_c; }      
}

/****************************************************************************************
*  Name     : determine_sample_validity_4D
*  Function : Determine if a sample is valid for use in estimation for the MCSKF4D.
*             See "On Collision Risk Assessment for Autonomous Ships Using Scenario-based"
*             MPC for more information.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ __device__ void CPE::determine_sample_validity_4D(
    const TML::Vector2d &p_os_cpa,                                           // In: Position of own-ship at cpa
    const double t_cpa                                                       // In: Time to cpa
    )
{
    n_samples = samples.get_cols();
    constant = p_os_cpa.dot(p_os_cpa) - pow(50.0, 2);
    for (int j = 0; j < n_samples; j++)
    {
        valid(j) = 0.0f;

        p_i_sample = samples.get_block<2, 1>(0, j, 2, 1);
        v_i_sample = samples.get_block<2, 1>(2, j, 2, 1);

        A = v_i_sample.dot(v_i_sample);
        B = (p_i_sample - p_os_cpa).transposed() * v_i_sample;
        B *= 2.0;
        C = p_i_sample.dot(p_i_sample) - 2.0 * p_os_cpa.dot(p_i_sample) + constant;

        calculate_roots_2nd_order();

        // Distinct real positive or pos+negative roots: 2 crossings, possibly only one in
        // t >= t0, checks if t_cpa occurs inside this root interval <=> inside safety zone
        if (!complex_roots && roots(0) != roots(1) && t_cpa >= 0 && (roots(0) <= t_cpa && t_cpa <= roots(1)))
        {
            valid(j) = 1.0f;
        }
        // Repetitive real positive roots: 1 crossing, this is only possible if the sampled
        // trajectory is tangent to the safety zone at t_cpa, checks if t_cpa = cross time
        // in this case
        else if(!complex_roots && roots(0) == roots(1) && t_cpa >= 0 && t_cpa == roots(0))
        {
            valid(j) = 1.0f;
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
	const TML::PDMatrix<float, 4, MAX_N_SEG_SAMPLES> &xs_os,                                               // In: Own-ship states for the active segment
    const TML::PDMatrix<float, 4, MAX_N_SEG_SAMPLES> &xs_i,                                                // In: Obstacle i states for the active segment
    const TML::PDMatrix<float, 16, MAX_N_SEG_SAMPLES> &P_i                                                 // In: Obstacle i covariance for the active segment
    )
{
    /*****************************************************
    * Find linear segment representative state vectors
    *****************************************************/
    n_seg_samples = xs_os.get_cols();

    if (n_seg_samples > 1)
    {    
        // Own-ship segment
        // Find average velocity along segment
        U = xs_os.get_block<2, MAX_N_SEG_SAMPLES>(2, 0, 2, n_seg_samples).rwise_mean().norm();
        // Find angle of the segment
        psi = atan2(xs_os(1, n_seg_samples - 1) - xs_os(1, 0), xs_os(0, n_seg_samples - 1) - xs_os(0, 0));
        // Set initial position to be that of the own-ship at the start of the segment
        xs_os_sl(0) = xs_os(0, 0); 
        xs_os_sl(1) = xs_os(1, 0);
        // Rotate velocity vector to be parallel to the straight line
        xs_os_sl(2) = U * cos(psi);
        xs_os_sl(3) = U * sin(psi);

        // Obstacle segment
        // Same procedure as every year James
        U = xs_i.get_block<2, MAX_N_SEG_SAMPLES>(2, 0, 2, n_seg_samples).rwise_mean().norm();
        psi = atan2(xs_i(1, n_seg_samples - 1) - xs_i(1, 0), xs_i(0, n_seg_samples - 1) - xs_i(0, 0));

        xs_i_sl(0) = xs_i(0, 0); 
        xs_i_sl(1) = xs_i(1, 0);
        xs_i_sl(2) = U * cos(psi);
        xs_i_sl(3) = U * sin(psi);
    }
    else 
    {
        // Throw, this should not happen
        return 0;
    }
    
    P_i_sl = reshape<16, MAX_N_SEG_SAMPLES, 4, 4>(P_i.get_col(0), 4, 4);

    calculate_cpa(p_os_cpa, t_cpa, d_cpa, xs_os_sl, xs_i_sl);

    /* printf("xs_os_sl = %.1f, %.1f, %.1f, %.1f\n", xs_os_sl(0), xs_os_sl(1), xs_os_sl(2), xs_os_sl(3));
    printf("xs_i_sl = %.1f, %.1f, %.1f, %.1f\n", xs_i_sl(0), xs_i_sl(1), xs_i_sl(2), xs_i_sl(3));

    printf("p_os_cpa = %.2f, %.2f\n", p_os_cpa(0), p_os_cpa(1));
    printf("t_cpa = %.4f\n", t_cpa); */

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
    //printf("y_P_c_i = %.6f\n", y_P_c_i);

    /*****************************************************
    * Kalman-filtering for simple markov chain model
    * P_c^i_k+1 = P_c^i_k + v^i_k
    * y^i_k     = P_c^i_k + w^i_k
    ******************************************************
    * Update using measurement y_P_c
    *****************************************************/
    K = var_P_c_p / (var_P_c_p + r);

    P_c_upd = P_c_p + K * (y_P_c_i - P_c_p);

    if (P_c_upd > 1.0f) P_c_upd = 1.0f;

    var_P_c_upd = (1.0f - K) * var_P_c_p;

    /*****************************************************
    * Predict
    *****************************************************/
    P_c_p = P_c_upd;
    if (P_c_p > 1.0f) P_c_p = 1.0f;

    var_P_c_p = var_P_c_upd + q;
    //*****************************************************
    /* printf("P_c_p = %.6f | P_c_upd = %.6f\n", P_c_p, P_c_upd);
    printf("var_P_c_p = %.6f | var_P_c_upd = %.6f\n", var_P_c_p, var_P_c_upd); */
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
    n_samples = samples.get_cols();

    constant = pow((double)d_safe, 2);
    for (int j = 0; j < n_samples; j++)
    {
        valid(j) = 0.0f;

        sample_innovation = samples.get_col(j) - p_os;

        inside_safety_zone = sample_innovation.dot(sample_innovation) <= constant;
        if (inside_safety_zone)
        {
            valid(j) = 1.0f;
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
    const TML::Matrix2d &P_i_inv                                                  // In: Obstacle i positional covariance
    )
{
    N_e = 0;

    constant = pow((double)d_safe, 2);
    for (int j = 0; j < n_CE; j++)
    {
        valid(j) = 0.0f;

        sample_innovation = samples.get_col(j) - p_os;

        inside_safety_zone = (sample_innovation).dot(sample_innovation) <= constant;

        sample_innovation = samples.get_col(j) - p_i;

        inside_alpha_p_confidence_ellipse = 
            sample_innovation.transposed() * P_i_inv * sample_innovation <= gate;

        if (inside_safety_zone && inside_alpha_p_confidence_ellipse)
        {
            valid(j) = 1.0f;
            N_e += 1;
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
__device__ void CPE::update_importance_density()
{
    // Set previous importance density parameters to the old current ones
    mu_CE_prev = mu_CE;
    P_CE_prev = P_CE;

    // Update the current parameters using the elite samples (those which are valid/best performing)
    mu_CE.set_zero();
    for (int j = 0; j < n_CE; j++)
    {
        if (valid(j) > 0.99f)
        {
            mu_CE += samples.get_col(j);
        }
    }
    mu_CE /= (float)N_e;

    P_CE.set_zero();
    for (int j = 0; j < n_CE; j++)
    {
        if (valid(j) > 0.99f)
        {
            sample_innovation = samples.get_col(j) - mu_CE;
            P_CE += sample_innovation * sample_innovation.transposed();
        }
    }
    P_CE /= (float)N_e;

    // Smoothing to aid in preventing degeneration
    mu_CE = alpha_n * mu_CE + (1.0f - alpha_n) * mu_CE_prev;
    P_CE =  alpha_n * P_CE  + (1.0f - alpha_n) * P_CE_prev;
}

/****************************************************************************************
*  Name     : CE_estimation
*  Function : Collision probability estimation using the Cross-Entropy method
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ float CPE::CE_estimate(
	const TML::Vector2f &p_os,                                                  // In: Own-ship position vector
    const TML::Vector2f &p_i,                                                   // In: Obstacle i position vector
    const TML::Matrix2f &P_i,                                                   // In: Obstacle i positional covariance
    const TML::Vector2f &v_os_prev,                                             // In: Previous time step own-ship north-east velocity (or zero if k = 0)
    const TML::Vector2f &v_i_prev,                                              // In: Previous time step obstacle north-east velocity (or zero if k = 0)
    const float dt                                                              // In: Time step
    )
{            
    P_c_CE = 0.0f;
    /******************************************************************************
    * Check if it is necessary to perform estimation
    ******************************************************************************/
    d_0i = (p_i - p_os).norm();
    
    if (P_i(0, 0) > P_i(1, 1)) { var_P_i_largest = P_i(0, 0); }
    else                       { var_P_i_largest = P_i(1, 1); }

    // This large a distance usually means no effective conflict zone, as
    // approx 99.7% of probability mass inside 3.5 * standard deviations
    // (assuming equal std dev in x, y (assumption))
    if (d_0i > d_safe + 3.5f * sqrtf(var_P_i_largest)) 
    { 
        return P_c_CE; 
    }

    /******************************************************************************
    * Convergence dependent initialization prior to the run at time t_k
    ******************************************************************************/
    sigma_inject = d_safe / 3.0f;
    if (converged_last)
    {
        mu_CE_prev = mu_CE_last; 
        mu_CE = v_os_prev + v_i_prev; 
        mu_CE *= dt; 
        mu_CE += mu_CE_last;

        P_CE_prev = P_CE_last; 
        P_CE = P_CE_last + powf(sigma_inject, 2) * TML::Matrix2f::identity();
    }
    else
    {
        mu_CE_prev = p_i + p_os; mu_CE_prev *= 0.5f; 
        mu_CE = mu_CE_prev;

        P_CE_prev = powf(d_safe, 2) * TML::Matrix2f::identity(); P_CE_prev /= 3.0f;
        P_CE = P_CE_prev;
    }
    /* printf("mu_CE = %.1f, %.1f\n", mu_CE(0), mu_CE(1));

    printf("P_CE = %.1f, %.1f\n", P_CE(0, 0), P_CE(0, 1));
	printf("	   %.1f, %.1f\n", P_CE(1, 0), P_CE(1, 1)); */

    /******************************************************************************
    * Iterative optimization to find the near-optimal importance density
    ******************************************************************************/
    P_i_inv = P_i; 
    P_i_inv = P_i_inv.inverse();
    for (int it = 0; it < max_it; it++)
    {
        generate_norm_dist_samples(mu_CE, P_CE);

        determine_best_performing_samples(p_os, p_i, P_i_inv);

        // Terminate iterative optimization if enough elite samples are collected
        if (N_e >= n_CE * rho) { converged_last = true; break; }
        // Otherwise, improve importance density parameters (given N_e > 3 to prevent zero-matrix 
        // in P_CE and/or negative definite matrix if no smoothing is used, and Pcoll spikes due
        // to insufficient sample amounts)
        else if (N_e > 3)
        {
            update_importance_density();
        }
    }
    mu_CE_last = mu_CE; P_CE_last = P_CE;

    /******************************************************************************
    * Estimate collision probability with samples from the final importance
    * density from the optimization
    ******************************************************************************/
    generate_norm_dist_samples(mu_CE, P_CE);
    
    determine_sample_validity_2D(p_os);

    // Use the last set of samples to:
    // Fill weights vector with the values for the integrand pdf
    norm_pdf_log(weights, p_i, P_i);
    // Fill "pdf_values" vector with the values for the importance pdf
    norm_pdf_log(pdf_values, mu_CE, P_CE);

    // Calculate importance weights (weights = integrand / importance) for estimating the integral \Int_S_2 {p^i(x, y, t_k) dx dy}
    // where p^i = Norm_distr(p_i, P_i; t_k) is the obstacle positional uncertainty (or combined uncertainty
    // if the own-ship uncertainty is also considered). 
    // Divide using log-values as this is more robust against underflow
    weights = (weights - pdf_values).exp();
    weights = weights.cwise_product(valid);

    P_c_CE = weights.rwise_mean();
    if (P_c_CE > 1) return 1.0f;
    else return P_c_CE;

    return 0;
}

}
}