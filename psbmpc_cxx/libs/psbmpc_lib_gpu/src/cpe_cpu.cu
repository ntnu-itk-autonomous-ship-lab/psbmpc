/****************************************************************************************
*
*  File name : cpe_cpu.cu
*
*  Function  : Class functions for the CPU collision probability estimator
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
#include <iostream>

/****************************************************************************************
*  Name     : CPE
*  Function : Class constructor, initializes parameters and variables
*  Author   : 
*  Modified :
*****************************************************************************************/
CPE_CPU::CPE_CPU(
    const CPE_Method cpe_method,                                    // In: Method to be used
    const double dt                                                 // In: Time step of calling function simulation environment
    ) :
    method(cpe_method), generator(seed()), std_norm_pdf(std::normal_distribution<double>(0, 1))
{
    // CE pars
    n_CE = 500;
    
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

    max_it = 6;

    converged_last = false;

    // MCSKF4D pars
    n_MCSKF = 500;

    r = 0.001;
    q = 8e-4;

    dt_seg = dt;

    resize_matrices();
}

/****************************************************************************************
*  Name     : CPE
*  Function : Copy constructor, prevents shallow copies and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
CPE_CPU::CPE_CPU(
    const CPE_CPU &other                                                // In: CPE object to copy
    )
{
    assign_data(other);
}

/****************************************************************************************
*  Name     : operator=
*  Function : Assignment operator to prevent shallow assignments and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
CPE_CPU& CPE_CPU::operator=(
    const CPE_CPU &rhs                                                  // In: Rhs CPE object to assign
    )
{
    if (this == &rhs)
    {
        return *this;
    }

    assign_data(rhs);

    return *this;
}

/****************************************************************************************
*  Name     : initialize
*  Function : Sets up the initial values for the collision probability estimator before
*             before a new run
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void CPE_CPU::initialize(
    const Eigen::Matrix<double, 6, 1> &xs_os,                                   // In: Own-ship state vector
    const Eigen::Vector4d &xs_i,                                                // In: Obstacle i state vector
    const Eigen::VectorXd &P_i,                                                 // In: Obstacle i covariance flattened into n^2 x 1
    const double d_safe_i                                                      // In: Safety zone around own-ship when facing obstacle i
    )
{
    d_safe = d_safe_i;
    switch (method)
    {
    case CE:
        converged_last = false;
        // Heuristic initialization
        mu_CE_last = 0.5 * (xs_os.block<2, 1>(0, 0) + xs_i.block<2, 1>(0, 0)); 
        
        P_CE_last = pow(d_safe, 2) * Eigen::Matrix2d::Identity() / 3.0;
        
        break;
    case MCSKF4D :
        P_c_p = 0.0; P_c_upd = 0.0;
        // Ad hoc variance for the probability
        var_P_c_p = 0.3; var_P_c_upd = 0.0; 
        break;
    default:
        // Throw
        break;
    }
}


/****************************************************************************************
*  Name     : estimate_over_trajectories
*  Function : Takes in own-ship and obstacle trajectories, plus the associated obstacle
*             covariances, and estimates the collision probabilities using a chosen 
*             method. Can use a larger sample time than used in predicting
*			  the vessel trajectories.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void CPE_CPU::estimate_over_trajectories(
		Eigen::Matrix<double, 1, -1> &P_c_i,                // In/out: Collision probability row vector: 1 x n_samples
		const Eigen::Matrix<double, 6, -1> &xs_p,           // In: Ownship predicted trajectory
		const Eigen::Matrix<double, 4, -1> &xs_i_p,         // In: Obstacle i predicted trajectory
		const Eigen::Matrix<double, 16, -1> &P_i_p,         // In: Obstacle i associated predicted covariances
        const double d_safe_i,                              // In: Safety zone around own-ship when facing obstacle i,
        const double dt,                                    // In: Prediction time step, can be larger than that used for the trajectory generation
        const int p_step                                    // In: Step between trajectory samples, matches the input prediction time step
    )
{
    dt_seg = dt;

    n_samples_traj = xs_p.cols();
    n_seg_samples = std::round(dt_seg / dt) + 1;

    xs_os_seg.resize(6, n_seg_samples); xs_i_seg.resize(4, n_seg_samples); P_i_seg.resize(16, n_seg_samples);

    initialize(xs_p.col(0), xs_i_p.col(0), P_i_p.col(0), d_safe_i);

    P_c_i.setZero(); v_os_prev.setZero(); v_i_prev.setZero();
    k_j_ = 0; k_j = 0; sample_count = 0;
    for (int k = 0; k < n_samples_traj; k += p_step)
    {
        switch(method)
        {
            case CE :	
                // If CE is used, (typically when n_cols = 1)
                // The last column gives the current prediction time information
                P_i_2D = reshape(P_i_p.col(k), 4, 4).block<2, 2>(0, 0);

                if (k > 0)
                {   
                    v_os_prev = xs_p.block<2, 1>(3, k - p_step);
                    v_os_prev = rotate_vector_2D(v_os_prev, xs_p(2, k - p_step));
                    v_i_prev = xs_i_p.block<2, 1>(2, k - p_step);
                }

                P_c_i(0, k) = CE_estimation(xs_p.block<2, 1>(0, k), xs_i_p.block<2, 1>(0, k), P_i_2D, v_os_prev, v_i_prev, dt);

                P_c_i.block(0, k, 1, p_step) = P_c_i(0, k) * Eigen::MatrixXd::Ones(1, p_step);

                break;
            case MCSKF4D :     
                xs_os_seg.col(sample_count) = xs_p.col(k); 
                xs_i_seg.col(sample_count) = xs_i_p.col(k);
                P_i_seg.col(sample_count) = P_i_p.col(k);
        
                if (fmod(k / p_step, n_seg_samples - 1) == 0 && k > 0)
                {
                    k_j_ = k_j; k_j = k;

                    P_c_i(0, k_j_) = MCSKF4D_estimation(xs_os_seg, xs_i_seg, P_i_seg);

                    // Collision probability on this active segment are all equal
                    P_c_i.block(0, k_j_, 1, k_j - k_j_ + 1) = P_c_i(0, k_j_) * Eigen::MatrixXd::Ones(1, k_j - k_j_ + 1);
                }	
                // Shift segment samples to the left if necessary
                sample_count += 1;
                if (sample_count == n_seg_samples)
                {
                    sample_count -= 1;
                    for (int s = 1; s < n_seg_samples; s++)
                    {
                        xs_os_seg.col(s - 1) = xs_os_seg.col(s);
                        xs_i_seg.col(s - 1) = xs_i_seg.col(s);
                        P_i_seg.col(s - 1) = P_i_seg.col(s);
                    }
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
*  Name     : assign_data
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void CPE_CPU::assign_data(
    const CPE_CPU &cpe                                                   // In: CPE object whose data to assign to *this
)
{
    this->method = cpe.method;

    this->n_CE = cpe.n_CE; this->n_MCSKF = cpe.n_MCSKF;

    this->generator = xoshiro256plus64(seed());

    this->std_norm_pdf = std::normal_distribution<double>(0, 1);

    this->sigma_inject = cpe.sigma_inject;
    this->alpha_n = cpe.alpha_n;
    this->gate = cpe.gate;
    this->rho = cpe.rho;
    this->max_it = cpe.max_it;

    this->converged_last = cpe.converged_last;

    this->mu_CE_last = cpe.mu_CE_last;
    this->P_CE_last = cpe.P_CE_last;

    this->N_e = cpe.N_e; this->e_count = cpe.e_count;
    this->elite_samples = cpe.elite_samples;

    this->weights = cpe.weights; this->integrand = cpe.integrand; this->importance = cpe.importance;

    this->q = cpe.q; this->r = cpe.r; this->dt_seg = cpe.dt_seg;

    this->P_c_p = cpe.P_c_p; this->var_P_c_p = cpe.var_P_c_p;
    this->P_c_upd = cpe.P_c_upd; this->var_P_c_upd = cpe.var_P_c_upd;

    this->samples = cpe.samples; this->valid = cpe.valid;

    this->d_safe = cpe.d_safe;

    this->L = cpe.L;
}

/****************************************************************************************
*  Name     : resize_matrices
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void CPE_CPU::resize_matrices()
{
    switch (method)
    {
        case CE :
            samples.resize(2, n_CE);
            elite_samples.resize(2, n_CE);
            valid.resize(n_CE);
            weights.resize(n_CE); 
            integrand.resize(n_CE); 
            importance.resize(n_CE);
            L.resize(2, 2);
            break;
        case MCSKF4D :
            samples.resize(4, n_MCSKF);
            valid.resize(n_MCSKF);
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
inline void CPE_CPU::update_L(
    const Eigen::MatrixXd &in                                                     // In: Matrix in consideration
    )
{
    n = in.rows();
    L = Eigen::MatrixXd::Zero(n, n);

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
                L(i, j) = 0.0;
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
inline void CPE_CPU::norm_pdf_log(
    Eigen::VectorXd &result,                                                    // In/out: Resulting vector of pdf values
    const Eigen::VectorXd &mu,                                                  // In: Expectation of the MVN
    const Eigen::MatrixXd &Sigma                                                // In: Covariance of the MVN
    )
{
    n = samples.rows();
    n_samples = samples.cols();

    log_val = - (n / 2.0) * log(2 * M_PI) - log(Sigma.determinant()) / 2.0;
    Sigma_inv = Sigma.inverse();
    for (int i = 0; i < n_samples; i++)
    {
        exp_val = (samples.col(i) - mu).transpose() * Sigma_inv * (samples.col(i) - mu);

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
inline void CPE_CPU::generate_norm_dist_samples(
    const Eigen::VectorXd &mu,                                                  // In: Expectation of the MVN
    const Eigen::MatrixXd &Sigma                                                // In: Covariance of the MVN
    )
{
    n = samples.rows();
    n_samples = samples.cols();

    update_L(Sigma);
    
    for (int c = 0; c < n; c++)
    {
        for(int i = 0; i < n_samples; i++)
        {
            samples(c, i) = std_norm_pdf(generator);
        }
    }
    // Box-muller transform
    samples = (L * samples).colwise() + mu;
}

/****************************************************************************************
*  Name     : calculate_roots_2nd_order
*  Function : Simple root calculation for a 2nd order polynomial Ax² + Bx + C = 0
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void CPE_CPU::calculate_roots_2nd_order()
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
double CPE_CPU::produce_MCS_estimate(
	const Eigen::Vector4d &xs_i,                                                // In: Obstacle state vector
	const Eigen::Matrix4d &P_i,                                                 // In: Obstacle covariance
	const Eigen::Vector2d &p_os_cpa,                                            // In: Position of own-ship at cpa
	const double t_cpa                                                         // In: Time to cpa
    )
{
    y_P_c = 0.0;

    generate_norm_dist_samples(xs_i, P_i);
    
    determine_sample_validity_4D(p_os_cpa, t_cpa);

    // The estimate is taken as the ratio of samples inside the integration domain, 
    // to the total number of samples, i.e. the mean of the validity vector
    y_P_c = valid.mean();
    if (y_P_c > 1.0)    { return 1.0; }
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
void CPE_CPU::determine_sample_validity_4D(
    const Eigen::Vector2d &p_os_cpa,                                            // In: Position of own-ship at cpa
    const double t_cpa                                                          // In: Time to cpa
    )
{
    n_samples = samples.cols();

    constant = p_os_cpa.dot(p_os_cpa) - pow(d_safe, 2);
    for (int j = 0; j < n_samples; j++)
    {
        valid(j) = 0.0;

        p_i_sample = samples.block<2, 1>(0, j);
        v_i_sample = samples.block<2, 1>(2, j);

        A = v_i_sample.dot(v_i_sample);
        B = 2 * (p_i_sample - p_os_cpa).transpose() * v_i_sample;
        C = p_i_sample.dot(p_i_sample) - 2 * p_os_cpa.dot(p_i_sample) + constant;

        calculate_roots_2nd_order();

        // Distinct real positive or pos+negative roots: 2 crossings, possibly only one in
        // t >= t0, checks if t_cpa occurs inside this root interval <=> inside safety zone
        if (!complex_roots && roots(0) != roots(1) && t_cpa >= 0 && (roots(0) <= t_cpa && t_cpa <= roots(1)))
        {
            valid(j) = 1.0;
        }
        // Repetitive real positive roots: 1 crossing, this is only possible if the sampled
        // trajectory is tangent to the safety zone at t_cpa, checks if t_cpa = cross time
        // in this case
        else if(!complex_roots && roots(0) == roots(1) && t_cpa >= 0 && t_cpa == roots(0))
        {
            valid(j) = 1.0;
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
double CPE_CPU::MCSKF4D_estimation(
	const Eigen::MatrixXd &xs_os,                                               // In: Own-ship states for the active segment
    const Eigen::MatrixXd &xs_i,                                                // In: Obstacle i states for the active segment
    const Eigen::MatrixXd &P_i                                                  // In: Obstacle i covariance for the active segment
    )
{
    // Collision probability "measurement" from MCS
    y_P_c_i = 0.0;

    /*****************************************************
    * Find linear segment representative state vectors
    *****************************************************/
    n_seg_samples = xs_os.cols();

    if (n_seg_samples > 1)
    {
        // Own-ship segment
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
    P_i_sl = reshape(P_i.col(0), 4, 4);

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
        y_P_c_i = produce_MCS_estimate(xs_i_sl, P_i_sl, xs_os.block<2, 1>(0, n_seg_samples - 1), dt_seg);
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
void CPE_CPU::determine_sample_validity_2D(
	const Eigen::Vector2d &p_os                                                // In: Own-ship position vector
    )
{
    n_samples = samples.cols();
    inside_safety_zone = false;
    for (int j = 0; j < n_samples; j++)
    {
        valid(j) = 0.0;
        inside_safety_zone = (samples.col(j) - p_os).dot(samples.col(j) - p_os) <= pow(d_safe, 2);
        if (inside_safety_zone)
        {
            valid(j) = 1.0;
        }
    }
}

/****************************************************************************************
*  Name     : determine_best_performing_samples
*  Function : Determine the elite samples for the CE method for a given iteration
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void CPE_CPU::determine_best_performing_samples(
    const Eigen::Vector2d &p_os,                                                    // In: Own-ship position vector
    const Eigen::Vector2d &p_i,                                                     // In: Obstacle i position vector
    const Eigen::Matrix2d &P_i_inv                                                  // In: Obstacle i positional inverse covariance
    )
{
    N_e = 0;
    inside_safety_zone = false;
    inside_alpha_p_confidence_ellipse = false;
    for (int j = 0; j < n_CE; j++)
    {
        valid(j) = 0.0;
        inside_safety_zone = (samples.col(j) - p_os).dot(samples.col(j) - p_os) <= pow(d_safe, 2);

        inside_alpha_p_confidence_ellipse = 
            (samples.col(j) - p_i).transpose() * P_i_inv * (samples.col(j) - p_i) <= gate;

        if (inside_safety_zone && inside_alpha_p_confidence_ellipse)
        {
            valid(j) = 1.0;
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
void CPE_CPU::update_importance_density()
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
    mu_CE = alpha_n * mu_CE + (1.0 - alpha_n) * mu_CE_prev;
    P_CE =  alpha_n * P_CE  + (1.0 - alpha_n) * P_CE_prev;
}


/****************************************************************************************
*  Name     : CE_estimation
*  Function : Collision probability estimation using the Cross-Entropy method
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double CPE_CPU::CE_estimation(
	const Eigen::Vector2d &p_os,                                                // In: Own-ship position vector
    const Eigen::Vector2d &p_i,                                                 // In: Obstacle i position vector
    const Eigen::Matrix2d &P_i,                                                 // In: Obstacle i positional covariance
    const Eigen::Vector2d &v_os_prev,                                           // In: Previous time step own-ship north-east velocity (or zero if k = 0)
    const Eigen::Vector2d &v_i_prev,                                            // In: Previous time step obstacle north-east velocity (or zero if k = 0)
    const double dt                                                             // In: Time step
    )
{
    P_c_CE = 0.0;
    /******************************************************************************
    * Check if it is necessary to perform estimation
    ******************************************************************************/
    d_0i = (p_i - p_os).norm();
    var_P_i_largest = 0.0;
    
    if (P_i(0, 0) > P_i(1, 1)) { var_P_i_largest = P_i(0, 0); }
    else                       { var_P_i_largest = P_i(1, 1); }

    // This large a distance usually means no effective conflict zone, as
    // practically 100% of probability mass inside 3.5 * standard deviations
    // (assuming equal std dev in x, y (assumption))
    if (d_0i > d_safe + 3.5 * sqrt(var_P_i_largest)) 
    { 
        return P_c_CE; 
    }

    /******************************************************************************
    * Convergence depedent initialization prior to the run at time t_k
    ******************************************************************************/
    sigma_inject = d_safe / 3.0;
    if (converged_last)
    {
        mu_CE_prev = mu_CE_last; 
        mu_CE = mu_CE_last + (v_os_prev + v_i_prev) * dt;

        P_CE_prev = P_CE_last; 
        P_CE = P_CE_last + pow(sigma_inject, 2) * Eigen::Matrix2d::Identity();
    }
    else
    {
        mu_CE_prev = 0.5 * (p_i + p_os); 
        mu_CE = mu_CE_prev;

        P_CE_prev = pow(d_safe, 2) * Eigen::Matrix2d::Identity() / 3;
        P_CE = P_CE_prev;
    }

    /******************************************************************************
    * Iterative optimization to find the near-optimal importance density
    ******************************************************************************/
    P_i_inv = P_i.inverse();
    for (int it = 0; it < max_it; it++)
    {
        generate_norm_dist_samples(mu_CE, P_CE);

        determine_best_performing_samples(p_os, p_i, P_i_inv);

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

    norm_pdf_log(integrand, p_i, P_i);
    norm_pdf_log(importance, mu_CE, P_CE);
    
    // Calculate importance weights for estimating the integral \Int_S_2 {p^i(x, y, t_k) dx dy}
    // where p^i = Norm_distr(p_i, P_i; t_k) is the obstacle positional uncertainty (or combined uncertainty
    // if the own-ship uncertainty is also considered). 
    // Divide using log-values as this is more robust against underflow
    weights = (integrand - importance).array().exp();
    weights = weights.cwiseProduct(valid);

    P_c_CE = weights.mean();
    if (P_c_CE > 1.0) return 1.0;
    else return P_c_CE;
}