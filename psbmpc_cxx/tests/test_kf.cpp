#include "cpu/utilities_cpu.hpp"
#include "kf.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>

#define BUFSIZE 1000000

int main()
{
	double t_0(0.0), dt(0.5);

	Eigen::Vector4d xs_i, xs_i_p, xs_i_upd, y_m;
	xs_i << 0, 0, 2, 2;
	y_m << 10, 0, 2, 1;

	Eigen::Matrix4d P_i;
	P_i << 10, 0, 0, 0,
		0, 10, 0, 0,
		0, 0, 2, 0,
		0, 0, 0, 2;

	std::unique_ptr<PSBMPC_LIB::KF> kf(new PSBMPC_LIB::KF(xs_i, P_i, t_0, true));

	double duration_lost(0.0);

	kf->predict(dt);
	kf->update(y_m, duration_lost, dt);
	xs_i_upd = kf->get_state();
	std::cout << "xs = [" << xs_i_upd(0) << ", " << xs_i_upd(1) << ", " << xs_i_upd(2) << ", "
			  << xs_i_upd(3) << "]" << std::endl;

	std::cout << "P_i = " << std::endl;
	std::cout << kf->get_covariance() << std::endl;

	kf->reset(xs_i, P_i, t_0);
	xs_i_upd = kf->get_state();
	std::cout << "xs = [" << xs_i_upd(0) << ", " << xs_i_upd(1) << ", " << xs_i_upd(2) << ", "
			  << xs_i_upd(3) << "]" << std::endl;

	std::cout << "P_i = " << std::endl;
	std::cout << kf->get_covariance() << std::endl;

	return 0;
}