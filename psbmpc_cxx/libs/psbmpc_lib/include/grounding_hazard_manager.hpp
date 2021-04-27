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

		std::vector<polygon_2D> polygons;

		// Matrix of 2 x (-1) polygons inside a grid (determined by input parameters)
		// Each polygon is separated by (-1, -1)
		Eigen::MatrixXd filtered_polygons;

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

					// Read only polygons, and only those without holes
					if (ps_shape->nSHPType == SHPT_POLYGON && ps_shape->nParts == 1)
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
		*  Author   : Tom Daniel Grande & Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		void convert(SHPObject *ps_shape, polygon_2D &polygon)
		{
			double* x = ps_shape->padfX;
			double* y = ps_shape->padfY;
			for (int v = 0; v < ps_shape->nVertices; v++)
			{
				typename boost::geometry::point_type<polygon_2D>::type point;
				boost::geometry::assign_values(point, x[v], y[v]);
				boost::geometry::append(polygon, point);
			}
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Grounding_Hazard_Manager(const std::string &filename)
		{
			read_shapefile(filename, polygons);
		}

		template <class MPC_Type>
		std::vector<polygon_2D> operator()(
			Eigen::Matrix<double, 2, 4> &box, 						// Bounding box matrix to used to filter out irrelevant polygons
			MPC_Type &mpc 											// Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			std::vector<polygon_2D> ret = polygons;

			return ret;
		}

		
	};
}