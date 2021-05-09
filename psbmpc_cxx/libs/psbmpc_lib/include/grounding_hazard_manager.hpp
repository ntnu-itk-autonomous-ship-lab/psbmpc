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

		std::vector<polygon_2D> polygons;

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
				printf("n_parts = %d | n_vertices = %d | Outer part start, end = (%d, %d)\n", ps_shape->nParts, ps_shape->nVertices, outer_part_start, outer_part_end);

			}
			for (int v = outer_part_start; v < outer_part_end; v++)
			{
				// want the points on format (northing, easting), and relative to the map origin
				typename boost::geometry::point_type<polygon_2D>::type point;
				boost::geometry::assign_values(point, y[v] - map_origin(0), x[v] - map_origin(1)); 
				boost::geometry::append(polygon, point);
			}
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
		}

		template <class MPC_Type>
		Grounding_Hazard_Manager(const std::string &filename, Eigen::Vector2d &map_origin, const MPC_Type &mpc) 
			: 
			d_so(mpc.pars.d_init), 
			map_origin(map_origin)
		{
			read_shapefile(filename, polygons);
		}

		Eigen::Vector2d get_map_origin() const { return map_origin; }

		std::vector<polygon_2D> get_polygons() const { return polygons; }

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