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
#include "cpu/utilities_cpu.hpp"
#include "utm_projection.hpp"
#include "shapefile_related/shapefil.h"

#include "Eigen/Dense"

#include <iostream>
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
	using Static_Obstacles = std::vector<polygon_2D>;
	class Grounding_Hazard_Manager
	{
	private:

		// Distance threshold for a polygon/static obstacle to be relevant for COLAV, and the
		// ramer-douglas-peucker algorithm distance tolerance threshold
		double d_so_relevant, epsilon;

		Static_Obstacles polygons_lla, polygons_ned, simplified_polygons_ned, simplified_polygons_lla;

		// Name of the NED frame polygons are specified relative to
		std::string frame_name;

		// Northing, easting origin of NED frame polygons are specified relative to
		Eigen::Vector2d map_origin_ned;

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
				point_2D point;
				boost::geometry::assign_values(point, y[v] - map_origin_ned(0), x[v] - map_origin_ned(1)); 
				boost::geometry::append(polygon, point);
			}
		}

		/****************************************************************************************
		*  Name     : create_lla_referenced_polygons
		*  Function : Transforms polygons referenced to map origin in NED, to latitude longitude
		*			  coordinates
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		void create_lla_referenced_polygons()
		{
			polygons_lla.clear();
			int n_polygons = polygons_ned.size();
			point_2D point_lla;
			Eigen::Vector2d p_ne, p_lla;
			for (int j = 0; j < n_polygons; j++)
			{
				polygon_2D poly_lla;
				for(auto it = boost::begin(boost::geometry::exterior_ring(polygons_ned[j])); it != boost::end(boost::geometry::exterior_ring(polygons_ned[j])); it++)
				{
					p_ne(0) = boost::geometry::get<0>(*it) + map_origin_ned(0);
					p_ne(1) = boost::geometry::get<1>(*it) + map_origin_ned(1);

					utm_p.reverse(p_lla(0), p_lla(1), p_ne(1), p_ne(0)); 

					boost::geometry::assign_values(point_lla, p_lla(0), p_lla(1)); 
					boost::geometry::append(poly_lla, point_lla);
				}

				polygons_lla.push_back(poly_lla);
			}
		}

		/****************************************************************************************
		*  Name     : create_ned_referenced_polygons
		*  Function : Transforms polygons referenced in lla, to NED
		*			  coordinates.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		void create_ned_referenced_polygons()
		{
			polygons_ned.clear();
			int n_polygons = polygons_lla.size();
			point_2D point_ne;
			Eigen::Vector2d p_ne, p_lla;
			for (int j = 0; j < n_polygons; j++)
			{
				polygon_2D poly_ned;
				for(auto it = boost::begin(boost::geometry::exterior_ring(polygons_lla[j])); it != boost::end(boost::geometry::exterior_ring(polygons_lla[j])); it++)
				{
					p_lla(0) = boost::geometry::get<0>(*it);
					p_lla(1) = boost::geometry::get<1>(*it);

					utm_p.forward(p_ne(1), p_ne(0), p_lla(0), p_lla(1)); 

					boost::geometry::assign_values(point_ne, p_ne(0) - map_origin_ned(0), p_ne(1) - map_origin_ned(1)); 
					boost::geometry::append(poly_ned, point_ne);
				}

				polygons_ned.push_back(poly_ned);
			}
		}

		/****************************************************************************************
		*  Name     : create_simplified_polygons
		*  Function : Uses the RDP algorithm to simplify the input polygons.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		Static_Obstacles create_simplified_polygons(const Static_Obstacles &polygons_in)
		{
			Static_Obstacles polygons_out;
			std::vector<point_2D> vertices_in, vertices_out;

			int n_polygons = polygons_in.size();
			for (int j = 0; j < n_polygons; j++)
			{
				vertices_in.clear();
				for(auto it = boost::begin(boost::geometry::exterior_ring(polygons_in[j])); it != boost::end(boost::geometry::exterior_ring(polygons_in[j])) - 1; it++)
				{
					vertices_in.emplace_back(*it);
				}

				ramer_douglas_peucker(vertices_out, vertices_in);

				polygon_2D poly;
				for (size_t v = 0; v < vertices_out.size(); v++)
				{
					boost::geometry::append(poly, vertices_out[v]);
				}
				boost::geometry::append(poly, vertices_out[0]);
				polygons_out.push_back(poly);
				//printf("Polygon: %d | Vertices before: %ld | Vertices after: %ld\n", j + 1, vertices_in.size(), vertices_out.size());
			}
			return polygons_out;
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
			const std::vector<point_2D> &vertices_in 					// In: Vertices of polygon to simplify								
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

				ramer_douglas_peucker(recursive_results_1, first_line);
				ramer_douglas_peucker(recursive_results_2, last_line);
		
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

		UTM_Projection utm_p;

		/****************************************************************************************
		*  Name     : Grounding_Hazard_Manager
		*  Function : Class constructor, initializes variables, objects, parameters
		*  Author   : 
		*  Modified : 
		*****************************************************************************************/
		Grounding_Hazard_Manager() {}

		Grounding_Hazard_Manager(const std::string &filename) 
			: 
			d_so_relevant(1000), epsilon(2.0), frame_name("local_NED"),
			map_origin_ned(7042250, 270250) // Trondheim, just north of ravnkloa, brattora crossing
		{
			read_shapefile(filename, polygons_ned);
			create_lla_referenced_polygons();
			simplified_polygons_ned = create_simplified_polygons(polygons_ned);
			simplified_polygons_lla = create_simplified_polygons(polygons_lla);
		}

		template <class MPC_Parameters>
		Grounding_Hazard_Manager(const std::string &filename, const Eigen::Vector2d &map_origin_ned, const MPC_Parameters &mpc_pars) 
			: 
			d_so_relevant(mpc_pars.d_so_relevant), epsilon(mpc_pars.epsilon_rdp), frame_name("local_NED"),
			map_origin_ned(map_origin_ned)
		{
			read_shapefile(filename, polygons_ned);
			create_lla_referenced_polygons();
			simplified_polygons_ned = create_simplified_polygons(polygons_ned);
			simplified_polygons_lla = create_simplified_polygons(polygons_lla);
		}

		template <class MPC_Parameters>
		Grounding_Hazard_Manager(const std::string &filename, const std::vector<double> &map_origin_ned, const MPC_Parameters &mpc_pars) 
			: d_so_relevant(mpc_pars.d_so_relevant), epsilon(mpc_pars.epsilon_rdp), frame_name("local_NED")
		{
			assert(map_origin_ned.size() == 2);
			this->map_origin_ned(0) = map_origin_ned[0]; this->map_origin_ned(1) = map_origin_ned[1];

			read_shapefile(filename, polygons_ned);
			create_lla_referenced_polygons();
			simplified_polygons_ned = create_simplified_polygons(polygons_ned);
			simplified_polygons_lla = create_simplified_polygons(polygons_lla);
		}

		template <class MPC_Parameters>
		Grounding_Hazard_Manager(
			const std::string &filename, 			// In: Filename of shapefile to process
			const double equatorial_radius,			// In: Radius  of ellipsoid parametrization for lla <-> UTM conversions
			const double flattening_factor,			// In: Flattening factor of ellipsoid parametrization for lla <-> UTM conversions
			const int utm_zone,						// In: UTM Zone to consider (NOTE: Must match the one used when generating the shapefiles)
			const bool northp,						// In: Boolean determining if it is the northern or southern hemisphere
			const Eigen::Vector2d &map_origin_lla,  // In: Origin of map (from shapefile) in latitude, longitude format
			const std::string frame_name,  			// In: Name of resulting local NED frame
			const MPC_Parameters &mpc_pars,			// In: Parameters of calling MPC (Either SB-MPC or PSB-MPC)
			const bool read_file					// In: Read input file <filename> or not
			) 
			: 
			d_so_relevant(mpc_pars.d_so_relevant), 
			epsilon(mpc_pars.epsilon_rdp), 
			frame_name(frame_name), 
			utm_p(equatorial_radius, flattening_factor, utm_zone, northp)
		{
			assert(map_origin_ned.size() == 2);
			utm_p.forward(this->map_origin_ned(1), this->map_origin_ned(0), map_origin_lla(0), map_origin_lla(1));

			if (read_file)
			{
				read_shapefile(filename, polygons_ned);
				create_lla_referenced_polygons();
				simplified_polygons_ned = create_simplified_polygons(polygons_ned);
				simplified_polygons_lla = create_simplified_polygons(polygons_lla);
			}
		}

		template <class MPC_Parameters>
		Grounding_Hazard_Manager(
			const std::string &filename, 			// In: Filename of shapefile to process
			const double equatorial_radius,			// In: Radius  of ellipsoid parametrization for lla <-> UTM conversions
			const double flattening_factor,			// In: Flattening factor of ellipsoid parametrization for lla <-> UTM conversions
			const int utm_zone,						// In: UTM Zone to consider (NOTE: Must match the one used when generating the shapefiles)
			const bool northp,						// In: Boolean determining if it is the northern or southern hemisphere
			const Eigen::Vector2d &map_origin_lla,  // In: Origin of map (from shapefile) in latitude, longitude format
			const std::string frame_name,  			// In: Name of resulting local NED frame
			const MPC_Parameters &mpc_pars			// In: Parameters of calling MPC (Either SB-MPC or PSB-MPC)
			) 
			: 
			d_so_relevant(mpc_pars.d_so_relevant), 
			epsilon(mpc_pars.epsilon_rdp), 
			frame_name(frame_name), 
			utm_p(equatorial_radius, flattening_factor, utm_zone, northp)
		{
			assert(map_origin_ned.size() == 2);
			utm_p.forward(this->map_origin_ned(1), this->map_origin_ned(0), map_origin_lla(0), map_origin_lla(1));

			read_shapefile(filename, polygons_ned);
			create_lla_referenced_polygons();
			simplified_polygons_ned = create_simplified_polygons(polygons_ned);
			simplified_polygons_lla = create_simplified_polygons(polygons_lla);
		}

		void clear_all_polygons() { polygons_lla.clear(); polygons_ned.clear(); simplified_polygons_ned.clear(); simplified_polygons_lla.clear(); }

		std::string get_frame_name() const { return frame_name; }
		Eigen::Vector2d get_map_origin_ned() const { return map_origin_ned; }
		Static_Obstacles get_polygons_ned() const { return polygons_ned; }
		Static_Obstacles get_polygons_lla() const { return polygons_lla; }
		Static_Obstacles get_simplified_polygons_ned() const { return simplified_polygons_ned; }
		Static_Obstacles get_simplified_polygons_lla() const { return simplified_polygons_lla; }

		/****************************************************************************************
		*  Name     : switch_local_ned_frame
		*  Function : Changes the local ned frame to another UTM zone and map origin
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		void switch_local_ned_frame(
			const double equatorial_radius,			// In: Radius  of ellipsoid parametrization for lla <-> UTM conversions
			const double flattening_factor,			// In: Flattening factor of ellipsoid parametrization for lla <-> UTM conversions
			const int utm_zone,						// In: New UTM Zone to consider
			const bool northp,						// In: Boolean determining if the origin is in the northern or southern hemisphere
			const Eigen::Vector2d &map_origin_lla,  // In: Origin of frame in latitude, longitude format
			const std::string frame_name  			// In: Name of resulting local NED frame
			)
		{
			utm_p = UTM_Projection(equatorial_radius, flattening_factor, utm_zone, northp);
			assert(map_origin_ned.size() == 2);
			utm_p.forward(this->map_origin_ned(1), this->map_origin_ned(0), map_origin_lla(0), map_origin_lla(1));

			create_ned_referenced_polygons();
			simplified_polygons_ned = create_simplified_polygons(polygons_ned);

			this->frame_name = frame_name;
		}

		/****************************************************************************************
		*  Name     : read_other_polygons
		*  Function : Reads other polygons from a file consisting of a matrix (2 x N) of polygon 
		*			  vertices, separated by columns of -1 (tested for .csv type).
		*		      Either in (easting, northing) or (longitude, latitude format).
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		void read_other_polygons(
			const std::string &filename, 							// In: Name of .csv file to process
			const bool is_ned_frame, 								// In: Boolean determining type of coordinates in file
			const bool clear_old_polygons 							// In: Boolean determining if the old polygons should be cleared or not
			)
		{
			try
			{
				if (clear_old_polygons)
				{
					clear_all_polygons();
				}
				
				Eigen::MatrixXd M = PSBMPC_LIB::CPU::read_matrix_from_file<Eigen::MatrixXd>(filename);
				assert(M.rows() == 2);
				int n_cols = M.cols();

				typename boost::geometry::point_type<polygon_2D>::type v_first, v;
				polygon_2D polygon, polygon_copy;
				int vcount(0);
				for (int i = 0; i < n_cols; i++)
				{
					if (M(0, i) <  0 || i == n_cols - 1)
					{
						// Add first vertex again to make the circle complete
						boost::geometry::append(polygon_copy, v_first);
						if (is_ned_frame)
						{
							polygons_ned.push_back(polygon_copy);
						}
						else
						{
							polygons_lla.push_back(polygon_copy);
						}
						
						polygon_copy = polygon;
						vcount = 0;
					}
					else
					{
						if (is_ned_frame)
						{
							// want the points on format (northing, easting), and relative to the map origin
							boost::geometry::assign_values(v, M(1, i) - map_origin_ned(0), M(0, i) - map_origin_ned(1)); 
						}
						else
						{
							boost::geometry::assign_values(v, M(1, i), M(0, i)); 
						}
						
						boost::geometry::append(polygon_copy, v);
						if (vcount == 0)
						{
							v_first = v;
						}
						vcount++;
					}
				}
				// Re-run RDP algorithm on the new set of polygons
				if (is_ned_frame)
				{
					create_lla_referenced_polygons();
					simplified_polygons_ned = create_simplified_polygons(polygons_ned);
				}
				else
				{
					create_ned_referenced_polygons();
					simplified_polygons_lla = create_simplified_polygons(polygons_lla);
				}
				
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
		*  Name     : operator()
		*  Function : Returns a vector of relevant static obstacles for use by the PSB/SB-MPC,
		*		 	  given the current own-ship position. Two overloads, latter one with
		*			  input parameter determining radius for relevant obstacles.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		Static_Obstacles operator()(
			const Eigen::VectorXd &ownship_state							// State of the own-ship, either [x, y, psi, u, v, r]^T or [x, y, chi, U]^T
			)
		{
			Static_Obstacles ret;

			double d_0j = 0.0; // distance to static obstacle
			point_2D p_os(ownship_state(0), ownship_state(1));
			BOOST_FOREACH(polygon_2D const& poly, simplified_polygons_ned)
			{
				d_0j = boost::geometry::distance(p_os, poly);
				if (d_0j < d_so_relevant)
				{
					ret.emplace_back(poly);
				}
			
			}
			return ret;
		}		

		Static_Obstacles operator()(
			const Eigen::VectorXd &ownship_state,							// State of the own-ship, either [x, y, psi, u, v, r]^T or [x, y, chi, U]^T
			const double d_so_relevant										// Radius of relevance for static obstacles 		
			)
		{
			Static_Obstacles ret;

			double d_0j = 0.0; // distance to static obstacle
			point_2D p_os(ownship_state(0), ownship_state(1));
			BOOST_FOREACH(polygon_2D const& poly, simplified_polygons_ned)
			{
				d_0j = boost::geometry::distance(p_os, poly);
				if (d_0j < d_so_relevant)
				{
					ret.emplace_back(poly);
				}
			}
			return ret;
		}		
	};
}