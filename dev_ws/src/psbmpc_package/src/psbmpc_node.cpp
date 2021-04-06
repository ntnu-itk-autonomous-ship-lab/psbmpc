/****************************************************************************************
*
*  File name : psbmpc_node.cpp
*
*  Function  : Class functions for the PSBMPC ROS2 node, and the main function for
*              running it.
*  
*	           ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2021 Trym Tengesdal, NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengesdal
*
*  Modified  : 
*
*****************************************************************************************/

#include <cstdio>
#include "psbmpc_node.h"

int main(int argc, char ** argv)
{
  PSBMPC_Node node;

  node.run();

  
}
