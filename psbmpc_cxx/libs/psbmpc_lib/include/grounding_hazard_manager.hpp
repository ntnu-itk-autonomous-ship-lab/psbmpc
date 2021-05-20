/****************************************************************************************
*
*  File name : grounding_hazard_manager.hpp
*
*  Function  : Header file for the static obstacle interface and data structure for
*			   keeping information on static obstacles/grounding hazards.
*
*  
*	           ---------------------
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

#pragma once

#include "psbmpc_parameters.hpp"
#include "shapefile_related/shapefil.h"

#include "Eigen/Dense"

#include <string>
#include <stdexcept>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/foreach.hpp>

typedef boost::geometry::model::d2::point_xy<double> point_2D;
typedef boost::geometry::model::polygon<point_2D> polygon_2D;

namespace PSBMPC_LIB
{
	class Grounding_Hazard_Manager
	{
	private:

		double d_so;

		std::vector<polygon_2D> polygons, simplified_polygons;

		Eigen::Vector2d map_origin;

		/****************************************************************************************
		*  Name     : read_shapefile
		*  Function : Reads a shapefile into the input vector of polygons, using the convert 
		*		      function to convert SHPObjects to Polygons.
		*  Author   : Tom Daniel Grande & Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <typename T>
		void read_shapefile(const std::string &filename, std::vector<T> &polygons)
		{
			try
			{
				SHPHandle handle = SHPOpen(filename.c_str(), "rb");
				if (handle <= (SHPHandle)0)
				{
					throw std::string("File " + filename + " not found");
				}

				int n_shape_type, n_entities;
				double adfMinBound[4], adfMaxBound[4];
				SHPGetInfo(handle, &n_entities, &n_shape_type, adfMinBound, adfMaxBound);

				for (int i = 0; i < n_entities; i++)
				{
					SHPObject* ps_shape = SHPReadObject(handle, i);

					// Read only polygons
					if (ps_shape->nSHPType == SHPT_POLYGON)
					{
						T polygon;
						convert(ps_shape, polygon);
						polygons.push_back(polygon);
					}
					SHPDestroyObject( ps_shape );
				}
				SHPClose(handle);
			}
			catch(const std::string &s)
			{
				throw s;
			}
			catch(...)
			{
				throw std::string("Other exception");
			}
		}
		
		/****************************************************************************************
		*  Name     : convert
		*  Function : Converts a SHPObject to a polygon. 
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		void convert(SHPObject *ps_shape, polygon_2D &polygon)
		{
			double* x = ps_shape->padfX; // easting
			double* y = ps_shape->padfY; // northing

			// Only consider the part for the outer ring of the polygon
			int outer_part_start = ps_shape->panPartStart[0];
			int outer_part_end(0);
			if (ps_shape->nParts < 2)
			{
				outer_part_end = ps_shape->nVertices;
			}
			else
			{
				outer_part_end = ps_shape->panPartStart[1];
			}
			//printf("n_parts = %d | n_vertices = %d | Outer part start, end = (%d, %d)\n", ps_shape->nParts, ps_shape->nVertices, outer_part_start, outer_part_end);

			for (int v = outer_part_start; v < outer_part_end; v++)
			{
				// want the points on format (northing, easting), and relative to the map origin
				typename boost::geometry::point_type<polygon_2D>::type point;
				boost::geometry::assign_values(point, y[v] - map_origin(0), x[v] - map_origin(1)); 
				boost::geometry::append(polygon, point);
			}
		}

		/****************************************************************************************
		*  Name     : create_simplified_polygons
		*  Function : Uses the 
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		void create_simplified_polygons()
		{
			std::vector<point_2D> vertices_in, vertices_out;

			double epsilon = 0.5;
			int n_polygons = polygons.size();
			for (int j = 0; j < n_polygons; j++)
			{
				for(auto it = boost::begin(boost::geometry::exterior_ring(polygons[j])); it != boost::end(boost::geometry::exterior_ring(polygons[j])); it++)
				{
					vertices_in.emplace_back(*it);
				}

				ramer_douglas_peucker(vertices_out, vertices_in, epsilon);

				polygon_2D poly;
				for (size_t v = 0; v < vertices_out.size(); v++)
				{
					boost::geometry::append(poly, vertices_out[v]);
				}
				simplified_polygons.emplace_back(poly);
			}
		}

		/****************************************************************************************
		*  Name     : ramer_douglas_peucker
		*  Function : Uses the Ramer Douglas Peucker algorithm to reduce vertices in a polygon,
		*			  conserving shape as best as possible.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		void ramer_douglas_peucker(
			std::vector<point_2D> &vertices_out, 						// In/Out: Vertices of simplified polygon
			const std::vector<point_2D> &vertices_in, 					// In: Vertices of polygon to simplify								
			double epsilon 												// In: Tolerance for distance
			)
		{
			int n_vertices_in = vertices_in.size();
			if(n_vertices_in < 2)
			{
				throw std::invalid_argument("Not enough points to simplify");
			}

			// Find the point with the maximum distance from line between start and end
			double d_max(0.0), d2line(0.0);
			size_t index = 0;
			size_t end = vertices_in.size() -1;
			for(size_t i = 1; i < end; i++)
			{
				d2line = distance_to_line_segment(vertices_in[i], vertices_in[0], vertices_in[end]);
				if (d2line > d_max)
				{
					index = i;
					d_max = d2line;
				}
			}

			if(d_max > epsilon)
			{
				std::vector<point_2D> recursive_results_1, recursive_results_2;
				std::vector<point_2D> first_line(vertices_in.begin(), vertices_in.begin() + index + 1);
				std::vector<point_2D> last_line(vertices_in.begin() + index, vertices_in.end());

				ramer_douglas_peucker(recursive_results_1, first_line, epsilon);
				ramer_douglas_peucker(recursive_results_2, last_line, epsilon);
		
				// Build the result list
				vertices_out.assign(recursive_results_1.begin(), recursive_results_1.end() - 1);
				vertices_out.insert(vertices_out.end(), recursive_results_2.begin(), recursive_results_2.end());
				
				if(vertices_out.size() < 2)
				{
					throw std::runtime_error("Problem assembling output");
				}	
			} 
			else 
			{
				vertices_out.clear();
				vertices_out.push_back(vertices_in[0]);
				vertices_out.push_back(vertices_in[end]);
			}
		}

		/****************************************************************************************
		*  Name     : distance_to_line_segment
		*  Function : Calculate distance from p to line segment {v_1, v_2}
		*  Author   : Trym Tengesdal
		*  Modified : 
		*****************************************************************************************/
		double distance_to_line_segment(
			const point_2D &p, 
			const point_2D &q_1, 
			const point_2D &q_2
			) const
		{
			double epsilon = 0.00001, l_sqrt(0.0), t_line(0.0);
			Eigen::Vector3d a, b;
			Eigen::Vector2d q_1_e, q_2_e, p_e, projection;
			q_1_e << boost::geometry::get<0>(q_1), boost::geometry::get<1>(q_1);
			q_2_e << boost::geometry::get<0>(q_2), boost::geometry::get<1>(q_2);
			p_e << boost::geometry::get<0>(p), boost::geometry::get<1>(p);
			a << (q_2_e - q_1_e), 0.0;
			b << (p_e - q_1_e), 0.0;

			l_sqrt = a(0) * a(0) + a(1) * a(1);
			if (l_sqrt <= epsilon)	{ return (q_1_e - p_e).norm(); }

			t_line = std::max(0.0, std::min(1.0, a.dot(b) / l_sqrt));
			projection = q_1_e + t_line * (q_2_e - q_1_e);

			return (projection - p_e).norm();
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		template <class MPC_Type>
		Grounding_Hazard_Manager(const std::string &filename, const MPC_Type &mpc) 
			: 
			d_so(mpc.pars.d_init), 
			map_origin(7042250, 270250) // Trondheim, just north of ravnkloa, brattora crossing
		{
			read_shapefile(filename, polygons);

			create_simplified_polygons();
		}

		template <class MPC_Type>
		Grounding_Hazard_Manager(const std::string &filename, Eigen::Vector2d &map_origin, const MPC_Type &mpc) 
			: 
			d_so(mpc.pars.d_init), 
			map_origin(map_origin)
		{
			read_shapefile(filename, polygons);

			create_simplified_polygons();
		}

		Eigen::Vector2d get_map_origin() const { return map_origin; }

		std::vector<polygon_2D> get_polygons() const { return polygons; }
		std::vector<polygon_2D> get_simplified_polygons() const { return simplified_polygons; }

		/****************************************************************************************
		*  Name     : operator()
		*  Function : Returns a vector of relevant static obstacles for use by the PSB/SB-MPC,
		*		 	  given the current own-ship position.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		std::vector<polygon_2D> operator()(
			const Eigen::VectorXd &ownship_state							// State of the own-ship, either [x, y, psi, u, v, r]^T or [x, y, chi, U]^T
			)
		{
			std::vector<polygon_2D> ret;

			double d_0j = 0.0; // distance to static obstacle
			point_2D p_os(ownship_state(0), ownship_state(1));
			BOOST_FOREACH(polygon_2D const& poly, polygons)
			{
				d_0j = boost::geometry::distance(p_os, poly);
				if (d_0j < d_so)
				{
					ret.emplace_back(poly);
				}
			
			}
			return ret;
		}		
	};
}