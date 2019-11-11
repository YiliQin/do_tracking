#include "PointSet.h"

namespace gen_point_set
{

  PointSet::PointSet(int _p_num_row, int _p_num_col)
  {
    pointSet.resize(_p_num_row * _p_num_col, 3);
    p_num_row_ = _p_num_row;
    p_num_col_ = _p_num_col;
    rows_ = _p_num_col;
    cols_ = _p_num_row;
    p_num_ = _p_num_row * _p_num_col;
    center_ << 0.0, 0.0, 0.0;

    //gen_point_set();
  }

  void PointSet::gen_point_set()
  {
    //// Genreate a line
		//for (size_t i = 0; i < gen_pointset.rows(); i++)
		//{
			//double u = (double)(i);
			//gen_pointset(i, 0) = (u / 10);
			//gen_pointset(i, 1) = (u / 10);
			//gen_pointset(i, 2) = (u / 10);
		//}

		// Genreate a plane

    int index = 0;
		for (size_t i = 0; i < pointSet.rows()/6; i++)
		{
			pointSet(index + i, 1) = ((double)(i))/20;
			//pointSet(index + i, 1) = ((double)(i))/3;
			//pointSet(index + i, 2) = ((double)(i))/3;
			pointSet(index + i, 0) = 0.05;
			pointSet(index + i, 2) = 0.0;
		}
		index = pointSet.rows() / 6;
		for (size_t i = 0; i < pointSet.rows()/6; i++)
		{
			//pointSet(index + i, 0) = ((double)(i))/3 + 0.5;
			pointSet(index + i, 1) = ((double)(i))/20;
			//pointSet(index + i, 1) = ((double)(i))/3;
			//pointSet(index + i, 2) = ((double)(i))/3;
			pointSet(index + i, 0) = 0.0;
			pointSet(index + i, 2) = 0.0;
		}
		index = 2 * pointSet.rows() / 6;
		for (size_t i = 0; i < pointSet.rows()/6; i++)
		{
			//pointSet(index + i, 0) = ((double)(i))/3 - 0.5;
			pointSet(index + i, 1) = ((double)(i))/20;
			//pointSet(index + i, 1) = ((double)(i))/3;
			//pointSet(index + i, 2) = ((double)(i))/3;
			pointSet(index + i, 0) = -0.05;
			pointSet(index + i, 2) = 0.0;
		}
    index = 3 * pointSet.rows() / 6;
		for (size_t i = 0; i < pointSet.rows()/6; i++)
		{
			//pointSet(index + i, 0) = ((double)(i))/3 - 0.5;
			pointSet(index + i, 1) = ((double)(i))/20;
			//pointSet(index + i, 1) = ((double)(i))/3;
			//pointSet(index + i, 2) = ((double)(i))/3;
			pointSet(index + i, 0) = -0.10;
			pointSet(index + i, 2) = 0.0;
		}
    index = 4 * pointSet.rows() / 6;
		for (size_t i = 0; i < pointSet.rows()/6; i++)
		{
			//pointSet(index + i, 0) = ((double)(i))/3 - 0.5;
			pointSet(index + i, 1) = ((double)(i))/20;
			//pointSet(index + i, 1) = ((double)(i))/3;
			//pointSet(index + i, 2) = ((double)(i))/3;
			pointSet(index + i, 0) = -0.15;
			pointSet(index + i, 2) = 0.0;
		}
    index = 5 * pointSet.rows() / 6;
		for (size_t i = 0; i < pointSet.rows()/6; i++)
		{
			//pointSet(index + i, 0) = ((double)(i))/3 - 0.5;
			pointSet(index + i, 1) = ((double)(i))/20;
			//pointSet(index + i, 1) = ((double)(i))/3;
			//pointSet(index + i, 2) = ((double)(i))/3;
			pointSet(index + i, 0) = -0.20;
			pointSet(index + i, 2) = 0.0;
		}
  } 

  Eigen::Vector3d PointSet::get_point()
  {
    Eigen::Vector3d p;

    return p;  
  }

  int PointSet::rows()
  {
    return rows_; 
  }

  int PointSet::cols()
  {
    return cols_; 
  }

  int PointSet::p_num()
  {
    return p_num_; 
  }

}
