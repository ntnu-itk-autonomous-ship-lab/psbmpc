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
#include "utilities.h"
#include <iostream>
#include "engine.h"

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
    sigma_inject = d_safe / 3;

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
    r = 0.001;
    q = 8e-4;

    dt_seg = dt;
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
    
    mu_CE_last.resize(n_obst);
    P_CE_last.resize(n_obst);
    
    P_c_p.resize(n_obst); P_c_upd.resize(n_obst);
    var_P_c_p.resize(n_obst); var_P_c_upd.resize(n_obst);
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
    const Eigen::VectorXd &P_i,                                                 // In: Obstacle i covariance flattened into n^2 x 1
    const int i                                                                 // In: Index of obstacle i
    )
{
    switch (method)
    {
    case CE:
        converged_last = false;
        // Heuristic initialization
        mu_CE_last[i] = 0.5 * (xs_os.block<2, 1>(0, 0) + xs_i.block<2, 1>(0, 0)); 
        
        P_CE_last[i] = pow(d_safe, 2) * Eigen::Matrix2d::Identity() / 3.0;
        break;
    case MCSKF4D :
        P_c_p(i) = 0; P_c_upd(i) = 0;
        // Ad hoc "guess" of variance for the probability
        var_P_c_p(i) = 0.3; var_P_c_upd(i) = 0; 
        break;
    default:
        std::cout << "Invalid method, reset it!" << std::endl;
        break;
    }
}

/****************************************************************************************
*  Name     : estimate
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double CPE::estimate(
	const Eigen::MatrixXd &xs_os,                                   // In: Own-ship state vector for time steps in [t_k-1, t_k-1 + dt_seg]
    const Eigen::MatrixXd &xs_i,                                    // In: Obstacle i state vector for time steps in [t_k-1, t_k-1 + dt_seg]
    const Eigen::MatrixXd &P_i,                                     // In: Obstacle i covariance for time steps in [t_k-1, t_k-1 + dt_seg], flattened into n² x n_cols
    const int i                                                     // In: Index of obstacle i
    )
{
    Eigen::Matrix2d P_i_2D;
    int n_cols = xs_os.cols();
    double P_c;
    switch (method)
    {
        case CE :
            // If CE is used, (typically when n_cols = 1)
            // The last column gives the current prediction time information
            P_i_2D = reshape(P_i, 4, 4).block<2, 2>(0, n_cols - 1);
            P_c = CE_estimation(xs_os.block<2, 1>(0, n_cols - 1), xs_i.block<2, 1>(0, n_cols - 1), P_i_2D, i);
            break;
        case MCSKF4D :
            P_c = MCSKF4D_estimation(xs_os, xs_i, P_i, i);
            break;
        default :
            std::cout << "Invalid estimation method" << std::endl;
            break;
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
    double log_val = - (n / 2.0) * log(2 * M_PI) - log(Sigma.determinant()) / 2.0;
    for (int i = 0; i < n_samples; i++)
    {
        // Consider using Cholesky decomposition if you find the direct inverse of
        // Sigma to be less numerically stable
        exp_val = - (xs.col(i) - mu).transpose() * Sigma.inverse() * (xs.col(i) - mu);
        exp_val = exp_val / 2.0;

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
*  Name     : calculate_roots_2nd_order
*  Function : Simple root calculation for a 2nd order polynomial Ax² + Bx + C = 0
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void CPE::calculate_roots_2nd_order(
    Eigen::Vector2d &r,                                                 // In: vector of roots to find
    bool &is_complex,                                                   // In: Indicator of real/complex roots
    const double A,                                                     // In: Coefficient in polynomial 
    const double B,                                                     // In: Coefficient in polynomial 
    const double C                                                      // In: Coefficient in polynomial 
    )
{
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
double CPE::produce_MCS_estimate(
	const Eigen::Vector4d &xs_i,                                                // In: Obstacle state vector
	const Eigen::Matrix4d &P_i,                                                 // In: Obstacle covariance
	const Eigen::Vector2d &p_os_cpa,                                            // In: Position of own-ship at cpa
	const double t_cpa                                                          // In: Time to cpa
    )
{
    double P_c;

    Eigen::MatrixXd samples(4, n_MCSKF);
    generate_norm_dist_samples(samples, xs_i, P_i);

    
    Eigen::VectorXd valid(n_MCSKF);
    determine_sample_validity_4D(valid, samples, p_os_cpa, t_cpa);

    // The estimate will be taken as the ratio of samples inside the integration domain, 
    // to the total number of samples, i.e. the mean of the validity vector
    P_c = valid.mean();
    if (P_c > 1) { return 1; }
    else         { return P_c; }      
}

/****************************************************************************************
*  Name     : determine_sample_validity_4D
*  Function : Determine if a sample is valid for use in estimation for the MCSKF4D method
*             See "On Collision Risk Assessment for Autonomous Ships Using Scenario-based"
*             MPC for more information
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
bool CPE::determine_sample_validity_4D(
    Eigen::VectorXd &valid,                                                     // In/out: Vector of ones/zeros for the valid samples
    const Eigen::MatrixXd &samples,                                             // In: Samples to be investigated
    const Eigen::Vector2d &p_os_cpa,                                            // In: Position of own-ship at cpa
    const double t_cpa                                                          // In: Time to cpa
    )
{
    int n_samples = samples.cols();

    bool complex_roots;
    Eigen::Vector2d p_i_sample, v_i_sample;
    double A, B, C;
    Eigen::Vector2d r;
    for (int i = 0; i < n_samples; i++)
    {
        valid(i) = 0;

        p_i_sample = samples.block<2, 1>(0, i);
        v_i_sample = samples.block<2, 1>(2, i);

        A = v_i_sample.dot(v_i_sample);
        B = 2 * (p_i_sample - p_os_cpa).transpose() * v_i_sample;
        C = p_i_sample.dot(p_i_sample) - 2 * p_os_cpa.dot(p_i_sample) + p_os_cpa.dot(p_os_cpa) - pow(d_safe, 2);

        calculate_roots_2nd_order(r, complex_roots, A, B, C);

        // Distinct real positive or pos+negative roots: 2 crossings, possibly only one in
        // t >= t0, checks if t_cpa occurs inside this root interval <=> inside safety zone
        if (!complex_roots && r(0) != r(1) && t_cpa >= 0 && (r(0) <= t_cpa && t_cpa <= r(1)))
        {
            valid(i) = 1;
        }
        // Repetitive real positive roots: 1 crossing, this is only possible if the sampled
        // trajectory is tangent to the safety zone at t_cpa, checks if t_cpa = cross time
        // in this case
        else if(!complex_roots && r(0) == r(1) && t_cpa >= 0 && t_cpa == r(0))
        {
            valid(i) = 1;
        }
        // Negative roots are not considered, as that implies going backwards in time..
    }
}

/****************************************************************************************
*  Name     : MCSKF4D_estimation
*  Function : Collision probability estimation using Monte Carlo Simulation and 
*             Kalman-filtering considering the 4D obstacle uncertainty along piece-wise
*             linear segments of the vessel trajectories. See "Risk-based Autonomous 
*             Maritime Collision Avoidance Considering Obstacle Intentions" for more info.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double CPE::MCSKF4D_estimation(
	const Eigen::MatrixXd &xs_os,                                               // In: Own-ship states for the active segment
    const Eigen::MatrixXd &xs_i,                                                // In: Obstacle i states for the active segment
    const Eigen::MatrixXd &P_i,                                                 // In: Obstacle i covariance for the active segment
    const int i                                                                 // In: Index of obstacle i
    )
{
    int n_seg_samples = xs_os.cols();
    // Collision probability "measurement" from MCS
    double y_P_c;

    // Vectors representing the linear segments
    Eigen::VectorXd xs_os_sl, xs_i_sl;
    xs_os_sl = xs_os.col(0);
    xs_i_sl = xs_i.col(0);

    if (n_seg_samples > 1)
    {
        // Define speed and course for the vessels along their linear segments
        double U_os_sl, U_i_sl, psi_os_sl, psi_i_sl;
    
        // Own-ship segment
        xs_os_sl.resize(4);
        // Find average velocity along segment
        U_os_sl = xs_os.block(3, 0, 2, n_seg_samples).rowwise().mean().norm();
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
        U_i_sl = xs_i.block(2, 0, 2, n_seg_samples).rowwise().mean().norm();
        psi_i_sl = atan2(xs_i(1, n_seg_samples - 1) - xs_i(1, 0), xs_i(0, n_seg_samples - 1) - xs_i(0, 0));

        xs_i_sl(0) = xs_i(0, 0); 
        xs_i_sl(1) = xs_i(1, 0);
        xs_i_sl(2) = U_i_sl * cos(psi_i_sl);
        xs_i_sl(3) = U_i_sl * sin(psi_i_sl);
    }
    std::cout << "xs os sl = " << xs_os_sl << std::endl;
    std::cout << "xs i sl = " << xs_i_sl << std::endl;


    Eigen::Matrix4d P_i_sl;
    P_i_sl = reshape(P_i.col(0), 4, 4);
    std::cout << "P_i_sl = " << std::endl;
    print_matrix(P_i_sl);

    Eigen::Vector2d p_os_cpa;
    double t_cpa, d_cpa;
    calculate_cpa(p_os_cpa, t_cpa, d_cpa, xs_os_sl, xs_i_sl);
    std::cout << "p_os_cpa = " << p_os_cpa << std::endl;
    std::cout << "t_cpa = " << t_cpa << std::endl;
    // Constrain the collision probability estimation to the interval [t_j-1, t_j], 
    // which is of length dt_seg. This is done to only consider vessel positions on
    // their discretized trajectories, and not beyond that. 
    if (t_cpa > dt_seg)
    {
        y_P_c = produce_MCS_estimate(xs_i_sl, P_i_sl, xs_os.block<2, 1>(0, n_seg_samples - 1), dt_seg);
    }
    else
    {
        y_P_c = produce_MCS_estimate(xs_i_sl, P_i_sl, p_os_cpa, t_cpa);
    }

    //*****************************************************
    // Kalman-filtering for simple markov chain model
    // P_c^i_k+1 = P_c^i_k + v^i_k
    // y^i_k     = P_c^i_k + w^i_k
    //*****************************************************
    // Update using measurement y_P_c
    //*****************************************************
    double K;
    K = var_P_c_p(i) / (var_P_c_p(i) + r);
    std::cout << "K = " << K << std::endl;
    P_c_upd(i) = P_c_p(i) + K * (y_P_c - P_c_p(i));
    std::cout << "P_c_p(i) = " << P_c_p(i) << std::endl;
    std::cout << "P_c_upd(i) = " << P_c_upd(i) << std::endl;
    if (P_c_upd(i) > 1) P_c_upd(i) = 1;

    var_P_c_upd(i) = (1 - K) * var_P_c_p(i);
    std::cout << "var_P_c_p(i) = " << var_P_c_p(i) << std::endl;
    std::cout << "var_P_c_upd(i) = " << var_P_c_upd(i) << std::endl;
    //*****************************************************
    // Predict
    //*****************************************************
    P_c_p(i) = P_c_upd(i);
    if (P_c_p(i) > 1) P_c_p(i) = 1;

    var_P_c_p(i) = var_P_c_upd(i) + q;
    std::cout << "var_P_c_p(i) = " << var_P_c_p(i) << std::endl;
    //*****************************************************
    
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
void CPE::determine_best_performing_samples(
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
    
/*     // Matlab engine setup
	Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
	char buffer[100000+1]; */
    
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

    
/*     // Matlab init CE plotting
    mxArray *p_os_ms = mxCreateDoubleMatrix(2, 1, mxREAL);
	mxArray *p_i_ms = mxCreateDoubleMatrix(2, 1, mxREAL);
    mxArray *P_i_ms = mxCreateDoubleMatrix(2, 2, mxREAL);

    mxArray *ms = mxCreateDoubleMatrix(2, n_CE, mxREAL);
	mxArray *ms_elite;

    mxArray *Pce_last = mxCreateDoubleMatrix(2, 2, mxREAL);
    mxArray *Pce = mxCreateDoubleMatrix(2, 2, mxREAL);

    mxArray *mu_ce_last = mxCreateDoubleMatrix(2, 1, mxREAL);
    mxArray *mu_ce = mxCreateDoubleMatrix(2, 1, mxREAL);

    double *p_p_os_ms = mxGetPr(p_os_ms);
	double *p_p_i_ms = mxGetPr(p_i_ms);
    double *p_P_i_ms = mxGetPr(P_i_ms);

	double *p_ms = mxGetPr(ms);
	

    double *p_Pce_last = mxGetPr(Pce_last);
	double *p_Pce = mxGetPr(Pce);

    double *p_mu_ce_last = mxGetPr(mu_ce_last);
	double *p_mu_ce = mxGetPr(mu_ce);

    Eigen::Map<Eigen::MatrixXd> map_p_os_ms(p_p_os_ms, 2, 1);
	map_p_os_ms = p_os;
    Eigen::Map<Eigen::MatrixXd> map_p_i_ms(p_p_i_ms, 2, 1);
	map_p_i_ms = p_i;
    Eigen::Map<Eigen::MatrixXd> map_P_i_ms(p_P_i_ms, 2, 2);
	map_P_i_ms = P_i;

    Eigen::Map<Eigen::MatrixXd> map_Pce_last(p_Pce_last, 2, 2);
	map_Pce_last = P_CE_last[i];

    Eigen::Map<Eigen::MatrixXd> map_mu_ce_last(p_mu_ce_last, 2, 1);
	map_mu_ce_last = mu_CE_last[i];

    engPutVariable(ep, "p_os", p_os_ms);
    engPutVariable(ep, "p_i", p_i_ms);
    engPutVariable(ep, "P_i", P_i_ms);
    engPutVariable(ep, "P_CE_last", Pce_last);
    engPutVariable(ep, "mu_CE_last", mu_ce_last);

    engEvalString(ep, "test_ce_live_plot_init"); */
    

    for (int i = 0; i < max_it; i++)
    {
        generate_norm_dist_samples(samples, mu_CE, P_CE);

        determine_best_performing_samples(valid, N_e, samples, p_os, p_i, P_i);

        elite_samples.resize(2, N_e);
        e_count = 0;
        for (int j = 0; j < n_CE; j++)
        {
            if (valid(j) == 1)
            {
                elite_samples.col(e_count) = samples.col(j);
                e_count++;
            }
        }

/*         ms_elite =  mxCreateDoubleMatrix(2, N_e, mxREAL);
        double *p_ms_elite = mxGetPr(ms_elite);

        // Send stuff to matlab for plotting
        Eigen::Map<Eigen::MatrixXd> map_ms(p_ms, 2, n_CE);
	    map_ms = samples;
        Eigen::Map<Eigen::MatrixXd> map_ms_elite(p_ms_elite, 2, N_e);
	    map_ms_elite = elite_samples;

        Eigen::Map<Eigen::MatrixXd> map_Pce(p_Pce, 2, 2);
	    map_Pce = P_CE;
        Eigen::Map<Eigen::MatrixXd> map_mu_ce(p_mu_ce, 2, 1);
	    map_mu_ce = mu_CE;

        engPutVariable(ep, "samples", ms);
        engPutVariable(ep, "elite_samples", ms_elite);
        engPutVariable(ep, "P_CE", Pce);
        engPutVariable(ep, "mu_CE", mu_ce);
        engEvalString(ep, "test_ce_live_plot_upd"); */

        // Terminate iterative optimization if enough elite samples are collected
        if (N_e > n_CE * rho) { converged_last = true; break;}
        // Otherwise, improve importance density parameters (given N_e > 1 to prevent zero-matrix 
        // in P_CE and/or negative definite matrix if no smoothing is used)
        else if (N_e > 1)
        {
            mu_CE_prev = mu_CE;
            P_CE_prev = P_CE;

            mu_CE = elite_samples.rowwise().mean();

            P_CE = Eigen::Matrix2d::Zero();
            for (int j = 0; j < N_e; j++)
            {
                P_CE += (elite_samples.col(j) - mu_CE) * (elite_samples.col(j) - mu_CE).transpose();
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
    save_matrix_to_file(samples);
    Eigen::VectorXd weights(n_CE), integrand(n_CE), importance(n_CE);
    norm_pdf_log(integrand, samples, p_i, P_i);
    save_matrix_to_file(integrand);
    norm_pdf_log(importance, samples, mu_CE, P_CE);
    save_matrix_to_file(importance);

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

    //engClose(ep);
    if (P_c > 1) return 1;
    else return P_c;
}