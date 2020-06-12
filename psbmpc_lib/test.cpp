
#include "kf.h"
#include <iostream>
#include <variant>
#include <vector>
#include "Eigen/StdVector"

int main(){

	Eigen::Matrix<double, 6, 1> xs;
	xs << 1, 2, 3, 4, 5, 6;

	std::cout << "xs = " << xs(0) << xs(1) << xs(2) << xs(3) << xs(4) << xs(5) << std::endl;

	Eigen::Matrix<double, 6, 1> *xs_ptr;

	xs_ptr = &xs;

	std::cout << "xs_ptr dereferences = " << xs_ptr->operator()(0) << xs_ptr->coeff(1) << xs_ptr->coeff(2) << xs_ptr->coeff(3) << xs_ptr->coeff(4) << xs_ptr->coeff(5) << std::endl;

	double u_d = 0.0;

	double *u_d_ptr = &u_d;

	std::cout << "u_d = " << u_d << " and u_d_ptr " << *u_d_ptr << std::endl;

	std::vector<Eigen::VectorXd> chi_offsets, value;
	chi_offsets.resize(3);
	chi_offsets[0].resize(13); chi_offsets[1].resize(7); chi_offsets[2].resize(5);
	Eigen::VectorXd v1, v2, v3;
	v1.resize(13); v2.resize(7); v3.resize(5);
	v1 << -90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90;
	v2 << -90, -75, -45, 0, 45, 75, 90;
	v3 << -90, -45, 0, 45, 90;
	chi_offsets[0] << v1;
	chi_offsets[1] << v2;
	chi_offsets[2] << v3;

	
	for (std::vector<Eigen::VectorXd>::iterator it = chi_offsets.begin(); it != chi_offsets.end(); it++){
		std::cout << "chi_offsets(j) = " << *it << std::endl;
	}
	
	/*
	for (int j = 0; j < chi_offsets.size(); j++){
		std::cout << "chi_offsets(j) = " << chi_offsets[j] << std::endl;
	}
	*/
	
	value = chi_offsets;
	value.erase(value.begin() + 1);

	if (value.size() > 0)
	{
		for (int j = 0; j < chi_offsets.size(); j++){
			if (j < value.size()){
				if (value[j].size() > 0)
				{
					chi_offsets[j] = value[j];
				}
			}
			else{
				chi_offsets.erase(chi_offsets.begin() + j);
			}
		}
	}
	for (std::vector<Eigen::VectorXd>::iterator it = chi_offsets.begin(); it != chi_offsets.end(); it++){
		std::cout << "chi_offsets(j) = " << *it << std::endl;
	}
	return 0;
}