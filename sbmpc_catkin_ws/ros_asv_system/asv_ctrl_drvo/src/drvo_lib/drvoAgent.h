/**
 * \file   drvoAgent.h
 * \brief  Declares the DRVO Agent class.
 */

#ifndef DRVO_AGENT_H_
#define DRVO_AGENT_H_

#include <cstddef>
#include <map>
#include <set>
#include <utility>
#include <vector>
#include <ctime>


#ifndef HRVO_VECTOR2_H_
#include "Vector2.h"
#endif

#ifndef DRVO_KD_TREE_H_
#include "KdTree.h"
#endif

#include "Eigen/Dense"
#include "ship_model.h"
#include "obstacle.h"

#include <math.h>

#include "iostream"
#include "fstream"

using namespace std;

	/**
	 * \class  Agent
	 * \brief  An agent in the simulation.
	 */
	class Agent {

	public:


		/**
		 * \brief      Constructor.
		 */
		Agent();

		/**
		 * \brief      Constructor.
		 * \param[in]  simulator          The simulation.
		 * \param[in]  position           The starting position of this agent.
		 * \param[in]  goalNo             The goal number of this agent.
		 * \param[in]  neighborDist       The maximum neighbor distance of this agent.
		 * \param[in]  maxNeighbors       The maximum neighbor count of this agent.
		 * \param[in]  radius             The radius of this agent.
		 * \param[in]  goalRadius         The goal radius of this agent.
		 * \param[in]  prefSpeed          The preferred speed of this agent.
		 * \param[in]  maxSpeed           The maximum speed of this agent.
		 * \param[in]  uncertaintyOffset  The uncertainty offset of this agent.
		 * \param[in]  maxAccel           The maximum acceleration of this agent.
		 * \param[in]  velocity           The initial velocity of this agent.
		 * \param[in]  orientation        The initial orientation (in radians) of this agent.
		 */
		Agent(const Vector2 &position, const Vector2 &velocity, float prefSpeed, float maxSpeed, float maxAccel, float orientation, float radius, float neighborDist, std::size_t maxNeighbors, float uncertaintyOffset);


		// Destructor
		~Agent();


		/**
		 *  @brief computes the best colav input for this agent.
		 *
		 * @param u_os_best The reference parameter to store the best speed offset.
		 * @param psi_os_best The reference parameter to store the best heading offset.
		 * @param u_d The nominal speed
		 * @param psi_d The nominal heading
		 * @param asv_state The state of the asv: x, y, psi, u, v, r.
		 * @param obst_states The states of the obstacles : x, y, u, v, A, B, C, D. (A, B, C, D - size from AIS)
		 */
		//void getBestControlInput(double &u_best, double &psi_best, double u_d, double psi_d, const Eigen::Matrix<double,6,1>& asv_state, const Eigen::Matrix<double,-1,10>& obst_states, ofstream &log_asv, ofstream &log_obst_1, ofstream &log_obst_2, ofstream &log_obst_3);
	
		void getBestControlOffset(double &u_os_best, double &psi_os_best, double u_d, double psi_d, const Eigen::Matrix<double,6,1>& asv_state, const Eigen::Matrix<double,-1,10>& obst_states, const Eigen::Matrix<double,-1,4>& static_obst, const Eigen::Matrix<double,-1,2>& next_waypoints);


		/**
		 * @brief  Returns the simulation time (prediction horizon) [sec].
		 */
		double getT();
		/**
		 * @brief Returns the time step for the simulation [sec].
		 */
		double getDt();
		
	        /**
	         * @brief  Returns the simulation time (prediction horizon) for static obstacles [sec].
	        */
    		double getT_stat();		

		/**
		 * @brief Returns the distance where COLREGS are said to apply [m].
		 */
		double getDClose();
		/**
		 * @brief Returns the minimal distance which is considered as safe [m].
		 */
		double getDSafe();

		/**
		 * @brief Returns the minimal distance which represents a collision range [m].
		 */
		double getDInit();

		/**
		 * @brief Returns the angle within which an obstacle is said to be ahead
		 * [deg].
		 */
		double getPhiAH();
		/**
		 * @brief Returns the angle outside of which an obstacle will be said to
		 * be overtaking, if the speed of the obstacle is larger than the ship's
		 * own speed
		 */
		double getPhiOT();
		/**
		 * @brief Returns the angle whitin which an obstacle is said to be head
		 * on [deg].
		 */
		double getPhiHO();
		/**
		 * @brief Returns the angle outside of which an obstacle is said to be
		 * crossing, if it is on the starboard side, heading towards the ship
		 * and not overtaking the ship.
		 */
		double getPhiCR();


		/**
		 * @brief Returns the cost of not complying with the COLREGS.
		 */
		double getKappa();
		/**
		 * @brief Returns the cost of deviating from the nominal speed.
		 */
		double getKP();
		/**
		 * @brief Returns the cost of changing the speed offset.
		 */
		double getKdP();
		/**
		 * @brief Returns the cost of deviating from the nominal heading
		 */
		double getKChi();
		/**
		 * @brief Returns the cost of changing the heading offset to starboard.
		 */
		double getKdChiSB();
		/**
		 * @brief Returns the cost of changing the heading offset to port.
		 */
		double getKdChiP();
		/**
		 * @brief Returns the possible offsets to the nominal heading [deg].
		 */
		Eigen::VectorXd getChiCA();
		/**
		 * @brief Returns the possible offsets to the nominal course, should be
		 * in the range [-1,1].
		 */
		Eigen::VectorXd getPCA();

		// get new parameters - declaration
		double getKChiSB();
		double getKChiP();
	
	    double getG();	
	    int getObstFilterStatus();
	    double getTLostLimit();
	    double getTTrackedLimit();


		/**
		 * @brief Sets the prediction horizon [sec].
		 */
		void setT(double T);
		/**
		 * @brief Sets the simulation step time [sec].
		 */
		void setDt(double dt);    
	    /**
	     * @brief Sets the prediction horizon for static objects [sec].
	     */
	    void setT_stat(double T_stat);

		void setDClose(double d_close);
		void setDSafe(double d_safe);

		void setPhiAH(double phi_AH);
		void setPhiOT(double phi_OT);
		void setPhiHO(double phi_HO);
		void setPhiCR(double phi_CR);
		void setDInit(double D_Init);

		void setP(double p);
		void setQ(double q);
		void setKColl(double k_coll);
		void setKappa(double kappa);
		void setKP(double K_P);
		void setKdP(double K_dP);
		void setKChi(double K_Chi);
		void setKdChiSB(double K_dChi_SB);
		void setKdChiP(double K_dChi_P);
		void setChiCA(Eigen::VectorXd Chi_ca);
		void setPCA(Eigen::VectorXd P_ca);

		std::string getMethod();

		// i = 0 -> eulers, i = 1 -> linear prediction
		void setMethod(int i);

		// set new parameters - declaration
		void setKChiSB(double K_Chi_SB);
		void setKChiP(double K_Chi_P);	
		    
	    void setG(double G);
	    void setObstFilterStatus(int obst_filter_on);
	    void setTLostLimit(double tLostLimit);
	    void setTTrackedLimit(double tTrackedLimit);


	private:
		/**
		 * \class  Candidate
		 * \brief  A candidate point.
		 */
		class Candidate {
		public:
			/**
			 * \brief  Constructor.
			 */
			Candidate() : velocityObstacle1_(0), velocityObstacle2_(0) { }

			/**
			 * \brief  The position of the candidate point.
			 */
			Vector2 position_;

			/**
			 * \brief  The number of the first velocity obstacle.
			 */
			int velocityObstacle1_;

			/**
			 * \brief  The number of the second velocity obstacle.
			 */
			int velocityObstacle2_;
		};

		/**
		 * \class  VelocityObstacle
		 * \brief  A hybrid reciprocal velocity obstacle.
		 */
		class VelocityObstacle {
		public:
			/**
			 * \brief  Constructor.
			 */
			VelocityObstacle() { }
			/**
			 * \brief  The position of the apex of the hybrid reciprocal velocity obstacle.
			 */
			Vector2 apex_;

			/**
			 * \brief  The direction of the first side of the hybrid reciprocal velocity obstacle.
			 */
			Vector2 side1_;

			/**
			 * \brief  The direction of the second side of the hybrid reciprocal velocity obstacle.
			 */
			Vector2 side2_;
		};


		/**
		 * \brief  Computes the neighbors of this agent.
		 */
		void computeNeighbors();

		/**
		 * \brief  Computes the new velocity of this agent.
		 */
		//void computeNewVelocity();

		/**
		 * \brief  Computes the preferred velocity of this agent.
		 */
		//void computePreferredVelocity();

		/**
		 * \brief          Inserts a neighbor into the set of neighbors of this agent.
		 * \param[in]      agentNo  The number of the agent to be inserted.
		 * \param[in,out]  rangeSq  The squared range around this agent.
		 */
		void insertNeighbor(std::size_t agentNo, float &rangeSq);

		/**
		 * \brief  Updates the orientation, position, and velocity of this agent.
		 */
		void update(double u_d, double psi_d, const Eigen::Matrix<double,6,1>& asv_state);

		/**
		 * \brief  Computes the dynamic weighted average values for this agent's current situation.
		 */
		void computeDynamicWeights();

		/**
		 * \brief  Computes the strategic reference velocity of this agent.
		 */
		void computeStrategicVelocity(const Eigen::Matrix<double,6,1>& asv_state, double u_d, double psi_d, const Eigen::Matrix<double,1,4>& static_obst);

		/**
		 * \brief  Computes feasible dynamic inputs that track the straight-line trajectory defined by the strategic reference velocity.
		 */
		//void computeDynamicInputs();

		/**
	 	 * \brief      Computes the square of a float.
	 	 * \param[in]  scalar  The float to be squared.
	  	 * \return     The square of the float.
	 	 */
		inline float sqrd(float scalar);


		/**
	 	 * \brief      Normalizes angle between -pi and pi.
	 	 * \param[in]  angle  The double to be normalized.
	  	 * \return     The normalized angle.
	 	 */
		inline double normalize_angle(double angle);
		
		inline double normalize_angle_360(double angle);


		/**
	 	 * \brief      Apply rotation defined by yaw angle on vector res
	 	 */
		void rot2d(double yaw, Eigen::Vector2d &res);

	    // Methods dealing with geographical constraints
	    /**
	     * @brief Finds distance to line segment
	     */
	    double dist2staticObst(Eigen::Vector2d p1, Eigen::Vector2d v1, Eigen::Vector2d v2);
	    /**
	     * @brief Finds the orientation of the ordered triplet (p,q,r)
	     * Returns the test result:
	     * 0 -> p,q,r are colinear
	     * 1 -> Clockwise
	     * 2 -> Counterclockwise
	     */
	    int orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);                           // new 2018
	    /**
	     * @brief Given three colinear points p,q,r. Checks if the point q is on the segment pr.
	     */
	    bool onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);                            // new 2018
	    /**
	     * @brief Checks if the line segments defined by p1,q1 and p2,q2 intersects
	     */
	    bool doIntersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2);   // new 2018
	    /**
	     * @brief Finds the distance from the point p1 to the line defined by q1,q2.
	     */
	    double distPoint2line(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d q2);                  // new 2018
	    /**
	     * @brief Checks if the point p1 is behind the line defined by v1,v2.
	     */
	    bool isBehind(Eigen::Vector2d p1, Eigen::Vector2d v1, Eigen::Vector2d v2, double dist2Line);        // new 2018
	    



		// MPC functions
		enum Methods {EulerFirstOrder, LinearPrediction};

		Methods method; 


		// General parameters
		double T_;
		double T_HD_;
		double DT_;
    		double T_stat_;
		double MAXDCHI_;

		double D_INIT_;  // same as D_COLAV_
		double D_CLOSE_; // fixed alternative to d_col_ = t_col_*prefSpeed_
		double D_SAFE_;  // same as safety radius

		double D_CRIT_;
		double D_REACT_;
		double T_HOLD_;

		double PHI_AH_;
		double PHI_OT_;
		double PHI_HO_;
		double PHI_CR_;
		
    		double G_;
    
		int OBST_FILTER_ON_;
		double T_LOST_LIMIT_;
		double T_TRACKED_LIMIT_;
		
		int n_obst;


		// MPC Tuning Parameters
		double P_;
		double Q_;
		double K_COLL_;
		double KAPPA_;
		double K_P_;
		double K_DP_;
		double K_CHI_;
		double K_DCHI_;
		double K_DCHI_SB_;
		double K_DCHI_P_;
		double K_CHI_SB_; 
		double K_CHI_P_;

		// MPC variables
		double cost_;
		Eigen::VectorXd Chi_ca_0;
		Eigen::VectorXd P_ca_;


		// Current colav behavior parameters
		Eigen::VectorXd SB_0, AH_0;
		Eigen::VectorXd CRG_0, OTG_0, OT_0, HO_0, OBS_OUT_OF_WAY_0, PHI_0;

		Eigen::VectorXd alpha_0; // current alpha for all obstacles
		Eigen::MatrixXd alpha_l; // last alpha applied for all obstacles, saved according to id_
		//Eigen::VectorXd alpha_t; // alpha values for all obstacles for the entire t_col or t_H (used when no colision is detected)

		Eigen::VectorXd dist_0;

		// Past control commands
		double Chi_last_;
		double P_last_;

		// MPC past control offsets
		double Chi_ca_last_;
		double Chi_ca_last_0;
		double P_ca_last_;

		// timing
		//clock_t tick1, tock1;
		bool hold_last_1;

		// Colav dynamic range variables
		double distToCol_; // d_col or d_CPA
		double minColTime_; // t_col or t_CPA


		// Agent-specific and environment parameters:
		shipModel *asv;
		std::vector<obstacle*> obstacles_; // was obst_vec
		std::vector<obstacle*> oldObstacles_; // 
		KdTree *kdTree_; // moved from simulator, agent manages its own environment  

		Vector2 newVelocity_;
		Vector2 position_;
		Vector2 prefVelocity_;
		Vector2 velocity_;
		Vector2 prevVelocity_;
		Vector2 gridVelocity_;
		Vector2 cand_predicted_pos_;
		Vector2 obst_predicted_pos_;

		std::size_t maxNeighbors_;
		float neighborDist_;
		float prefSpeed_;
		float maxSpeed_;
		float maxAccel_;
		float maxTurnRate_;
		float orientation_;
		float radius_;
		float uncertaintyOffset_;

		std::multimap<float, Candidate> candidates_;
		std::set<std::pair<float, std::size_t> > neighbors_;
		std::vector<VelocityObstacle> velocityObstacles_;

		friend class KdTree;

	};

#endif /* DRVO_AGENT_H_ */
