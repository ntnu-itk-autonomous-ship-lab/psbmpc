/****************************************************************************************
*
*  File name : cpe.cpp
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

#include "cpe.h"
#include <iostream>

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
CPE::CPE(
    const CPE_Method cpe_method,                                    // In: Method to be used
    const int n_CE,                                                 // In: Number of samples for the Cross-Entropy method
    const int n_MCSKF,                                              // In: Number of samples for the Monte Carlo Simulation + Kalman-filtering method
    const double d_safe,                                            // In: Safety zone radius around own-ship
    const double dt                                                 // In: Time step of calling function simulation environment
    ) :
    method(cpe_method), n_CE(n_CE), n_MCSKF(n_MCSKF), generator(seed()), std_norm_pdf(std::normal_distribution<double>(0, 1)), d_safe(d_safe)
{
    set_number_of_obstacles(1);
    // CE pars
    sigma_inject = 0.9;

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
    p = 0.001;

    q = 0.003;

    dt_seg = 2 * dt;
}

/****************************************************************************************
*  Name     : set_number_of_obstacles
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void CPE::set_number_of_obstacles(
    const int n_obst                                            // In: Number of obstacles
    ) 
{ 
    this->n_obst = n_obst; 
    switch (method)
    {
    case CE:
        mu_CE_last.resize(n_obst);
        P_CE_last.resize(n_obst);
        break;

    case MCSKF4D :
        P_c_p.resize(n_obst); P_c_upd.resize(n_obst);
        var_P_c_p.resize(n_obst); var_P_c_upd.resize(n_obst);
        break;
    
    default:
        std::cout << "Invalid method, reset it!" << std::endl;
        break;
    }
}

/****************************************************************************************
*  Name     : initialize
*  Function : Sets up the initial values for the collision probability estimator before
*             before a new run
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void CPE::initialize(
    const Eigen::Matrix<double, 6, 1> &xs_os,                                   // In: Own-ship state vector
    const Eigen::Vector4d &xs_i,                                                // In: Obstacle i state vector
    const Eigen::Matrix4d &P_i,                                                 // In: Obstacle i covariance
    const int i                                                                 // In: Index of obstacle i
    )
{
    switch (method)
    {
    case CE:
        converged_last = false;
        for (int i = 0; i < n_obst; i++)
        {
            // Heuristic initialization
            mu_CE_last[i] = 0.5 * (xs_os.block<2, 1>(0, 0) + xs_i); 
            
            P_CE_last[i] = pow(d_safe, 2) * Eigen::Matrix2d::Identity() / 3;
        }
        break;
    case MCSKF4D :
        for (int i = 0; i < n_obst; i++)
        {
            P_c_p(i) = 0; P_c_upd(i) = 0;
            // Ad hoc "guess" of variance for the probability
            var_P_c_p(i) = 0.3; var_P_c_upd(i) = 0;
        }
        break;
    default:
        std::cout << "Invalid method, reset it!" << std::endl;
        break;
    }
}

/****************************************************************************************
*  Name     : reset
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void CPE::reset()
{
    switch (method)
    {
        case CE :
        {
            converged_last = false;

            break;
        }
        case MCSKF4D :
        {
            for (int i = 0; i < n_obst; i++)
            {
                P_c_p(i) = 0; P_c_upd(i) = 0;
                // Ad hoc "guess" of variance for the KF
                var_P_c_p(i) = 0.3; var_P_c_upd(i) = 0;
            }
            break;
        }
        default :
        {
            std::cout << "Invalid estimation method" << std::endl;
            break;
        }
    }
}

/****************************************************************************************
*  Name     : estimate
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double CPE::estimate(
	const Eigen::Matrix<double, 6, 1> &xs_os,                                   // In: Own-ship state vector
    const Eigen::Vector4d &xs_i,                                                // In: Obstacle i state vector
    const Eigen::Matrix4d &P_i,                                                 // In: Obstacle i covariance
    const int i                                                                 // In: Index of obstacle i
    )
{
    double P_c;
    switch (method)
    {
        case CE :
        {
            P_c = CE_estimation(xs_os.block<2, 1>(0, 0), xs_i.block<2, 1>(0, 0), P_i.block<2, 2>(0, 0), i);
            break;
        }
        case MCSKF4D :
        {
            P_c = MCSKF4D_estimation(xs_os, xs_i, P_i, i);
            break;
        }
        default :
        {
            std::cout << "Invalid estimation method" << std::endl;
            break;
        }
    }
    return P_c;
}


/****************************************************************************************
	Private functions
****************************************************************************************/

/****************************************************************************************
*  Name     : norm_pdf_log
*  Function : Calculates the logarithmic value of the multivariate normal distribution
*             for samples.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void CPE::norm_pdf_log(
    Eigen::VectorXd &result,                                                    // In/out: Resulting vector of pdf values
    const Eigen::MatrixXd &xs,                                                  // In: Samples of states from a normally distributed RV
    const Eigen::VectorXd &mu,                                                  // In: Expectation of the MVN
    const Eigen::MatrixXd &Sigma                                                // In: Covariance of the MVN
    )
{
    int n = xs.rows(), n_samples = xs.cols();
    result.resize(n_samples);
    double exp_val = 0;
    double log_val = - pow(log(2 * M_PI), n / 2) * sqrt(Sigma.determinant());
    for (int i = 0; i < n_samples; i++)
    {
        // Consider using Cholesky decomposition if you find the direct inverse of
        // Sigma to be less numerically stable
        exp_val = - (xs.col(i) - mu).transpose() * Sigma.inverse() * (xs.col(i) - mu);
        exp_val = exp_val / 2;

        result(i) = log_val + exp_val;
    }
}

/****************************************************************************************
*  Name     : generate_norm_dist_samples
*  Function : Generates samples from the MVN distribution with given mean and covariance
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void CPE::generate_norm_dist_samples(
    Eigen::MatrixXd &samples,                                                   // In: Samples to fill with generated MVN values, size n x n_samples
    const Eigen::VectorXd &mu,                                                  // In: Expectation of the MVN
    const Eigen::MatrixXd &Sigma                                                // In: Covariance of the MVN
    )
{
    int n = samples.rows(), n_samples = samples.cols();
    Eigen::MatrixXd std_norm_samples(n, n_samples);

    Eigen::LLT<Eigen::MatrixXd> cholesky_decomposition(Sigma);
    Eigen::MatrixXd L = cholesky_decomposition.matrixL();
    // Check if bigger loop within small loop faster than the other way around
    for (int c = 0; c < n; c++)
    {
        for(int i = 0; i < n_samples; i++)
        {
            std_norm_samples(c, i) = std_norm_pdf(generator);
        }
    }
    /*
    for (int i = 0; i < n_samples; i++)
    {
        for(int c = 0; c < n; c++)
        {
            std_norm_samples(c, i) = std_norm_pdf(generator);
        }
    }
    */
    // Box-muller transform
    samples = (L * std_norm_samples).colwise() + mu;
}

/****************************************************************************************
*  Name     : produce_MCS_estimate
*  Function : Uses Monte Carlo Simulation to produce a collision probability "measurement"
*             for the MCSKF4D method
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double CPE::produce_MCS_estimate(
	const Eigen::Vector4d &xs_i,                                                // In: Obstacle state vector
	const Eigen::Matrix4d &P_i,                                                 // In: Obstacle covariance
	const Eigen::Vector2d &p_os_cpa,                                            // In: Position of own-ship at cpa
	const double t_cpa                                                          // In: Time to cpa
    )
{

}

/****************************************************************************************
*  Name     : determine_sample_validity_4D
*  Function : Determine if a sample is valid for use in estimation for the MCSKF4D method
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
bool CPE::determine_sample_validity_4D(
    Eigen::VectorXd &valid,                                                     // In/out: Vector of ones/zeros for the valid samples
    const Eigen::MatrixXd &samples,                                             // In: Samples to be investigated
    const Eigen::Vector2d &p_OS_cpa,                                            // In: Position of own-ship at cpa
    const double t_cpa                                                          // In: Time to cpa
    )
{
    
}

/****************************************************************************************
*  Name     : MCSKF4D_estimation
*  Function : Collision probability estimation using Monte Carlo Simulation and 
*             Kalman-filtering considering the 4D obstacle uncertainty
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double CPE::MCSKF4D_estimation(
	const Eigen::MatrixXd &xs_os,                                               // In: Own-ship states for the active segment
    const Eigen::VectorXd &xs_i,                                                // In: Obstacle i states for the active segment
    const Eigen::Matrix4d &P_i,                                                 // In: Obstacle i covariance for the start of the active segment
    const int i                                                                 // In: Index of obstacle i
    )
{
    
    return P_c_upd(i);
}

/****************************************************************************************
*  Name     : determine_sample_validity_2D
*  Function : Determine valid samples for collision probability estimation for 2D methods
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void CPE::determine_sample_validity_2D(
		Eigen::VectorXd &valid,
		const Eigen::MatrixXd &samples, 
		const Eigen::Vector2d &p_os
    )
{
    int n_samples = samples.cols();
    bool inside_safety_zone;
    for (int i = 0; i < n_samples; i++)
    {
        valid(i) = 0;
        inside_safety_zone = (samples.col(i) - p_os).dot(samples.col(i) - p_os) <= pow(d_safe, 2);
        if (inside_safety_zone)
        {
            valid(i) = 1;
        }
    }
}

/****************************************************************************************
*  Name     : determine_best_performing_samples
*  Function : Determine the elite samples for the CE method for a given iteration
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Eigen::VectorXd CPE::determine_best_performing_samples(
    Eigen::VectorXd &valid,                                                         // In/out: Vector of ones/zeros for the best performing samples in <samples>
    int &N_e,                                                                       // In/out: Number of best performing samples
    const Eigen::MatrixXd &samples,                                                 // In: Samples to be investigated
    const Eigen::Vector2d &p_os,                                                    // In: Own-ship position vector
    const Eigen::Vector2d &p_i,                                                     // In: Obstacle i position vector
    const Eigen::Matrix2d &P_i                                                      // In: Obstacle i positional covariance
    )
{
    N_e = 0;
    bool inside_safety_zone, inside_alpha_p_confidence_ellipse;
    for (int i = 0; i < n_CE; i++)
    {
        valid(i) = 0;
        inside_safety_zone = (samples.col(i) - p_os).dot(samples.col(i) - p_os) <= pow(d_safe, 2);

        inside_alpha_p_confidence_ellipse = 
            (samples.col(i) - p_i).transpose() * P_i.inverse() * (samples.col(i) - p_i) <= gate;

        if (inside_safety_zone && inside_alpha_p_confidence_ellipse)
        {
            valid(i) = 1;
            N_e++;
        }
    }
    return valid;
}

/****************************************************************************************
*  Name     : CE_estimation
*  Function : Collision probability estimation using the Cross-Entropy method
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double CPE::CE_estimation(
	const Eigen::Vector2d &p_os,                                                // In: Own-ship position vector
    const Eigen::Vector2d &p_i,                                                 // In: Obstacle i position vector
    const Eigen::Matrix2d &P_i,                                                 // In: Obstacle i positional covariance
    const int i                                                                 // In: Index of obstacle i
    )
{
    double P_c;
    // Check if it is necessary to perform estimation
    double d_0i = (p_i - p_os).norm();
    double var_P_i_largest = 0;
    
    if (P_i(0, 0) > P_i(1, 1)) { var_P_i_largest = P_i(0, 0); }
    else                       { var_P_i_largest = P_i(1, 1); }

    // This large a distance usually means no effective conflict zone
    if (d_0i > d_safe + 5 * var_P_i_largest) { P_c = 0; return P_c; }

    Eigen::Vector2d mu_CE_prev, mu_CE;
    Eigen::Matrix2d P_CE_prev, P_CE;

    // Convergence depedent initialization prior to the run at time t_k
    if (converged_last)
    {
        mu_CE_prev = mu_CE_last[i]; mu_CE = mu_CE_last[i];

        P_CE_prev = P_CE_last[i]; 
        P_CE = P_CE_last[i] + pow(sigma_inject, 2) * Eigen::Matrix2d::Identity();
    }
    else
    {
        mu_CE_prev = 0.5 * (p_i + p_os);
        mu_CE = mu_CE_prev;
        P_CE_prev = pow(d_safe, 2) * Eigen::Matrix2d::Identity() / 3;
        P_CE = P_CE_prev;
    }

    int N_e, e_count = 0;
    Eigen::MatrixXd samples(2, n_CE), elite_samples;
    Eigen::VectorXd valid(n_CE);
    for (int i = 0; i < max_it; i++)
    {
        generate_norm_dist_samples(samples, mu_CE, P_CE);

        determine_best_performing_samples(valid, N_e, samples, p_os, p_i, P_i);

        elite_samples.resize(2, N_e);
        for (int j = 0; j < n_CE; j++)
        {
            if (valid(j) = 1)
            {
                elite_samples.col(e_count) = samples.col(j);
                e_count++;
            }
        }

        // Terminate iterative optimization if enough elite samples are collected
        if (N_e > n_CE * rho) { converged_last = true; break; }
        // Otherwise, improve importance density parameters
        else
        {
            mu_CE_prev = mu_CE;
            P_CE_prev = P_CE;

            mu_CE = elite_samples.colwise().mean();

            P_CE = Eigen::Matrix2d::Zero();
            for (int j = 0; j < N_e; j++)
            {
                P_CE += (elite_samples.col(j) - mu_CE) * (elite_samples.col(j) - mu_CE);
            }
            P_CE = P_CE / (double)N_e;

            // Smoothing to aid in preventing degeneration
            mu_CE = alpha_n * mu_CE + (1 - alpha_n) * mu_CE_prev;
            P_CE =  alpha_n * P_CE  + (1 - alpha_n) * P_CE_prev;
        }
    }
    mu_CE_last[i] = mu_CE; P_CE_last[i] = P_CE;

    // Estimate collision probability with a final set of samples from the importance density
    generate_norm_dist_samples(samples, mu_CE, P_CE);

    determine_sample_validity_2D(valid, samples, p_os);

    Eigen::VectorXd weights(n_CE), integrand(n_CE), importance(n_CE);
    norm_pdf_log(integrand, samples, p_i, P_i);
    norm_pdf_log(importance, samples, mu_CE, P_CE);

    // Calculate importance weights for estimating the integral \Int_S_2 {p^i(x, y, t_k) dx dy}
    // where p^i = Norm_distr(p_i, P_i; t_k) is the obstacle positional uncertainty (or combined uncertainty
    // if the own-ship uncertainty is also considered)
    for (int i = 0; i < n_CE; i++)
    {
        if (exp(importance(i)) > 0) { weights(i) = exp(integrand(i)) / exp(importance(i)); }   
        else                        { weights(i) = 0; }
    }
    weights = weights.cwiseProduct(valid);
    
    P_c = weights.mean();

    if (P_c > 1) return 1;
    else return P_c;
}