/************************************************************************
*
*  lap.h
   version 1.0 - 21 june 1996
   author  Roy Jonker, MagicLogic Optimization Inc.

   header file for LAP
*
      pyLAPJV by Harold Cooper (hbc@mit.edu)
      2004-08-13:
          -- fixed Jonker's function declarations to actually use row, col,
             and cost types
	  -- row, col, and cost now based on basic types

      15/03/2022: David Eugenio <david.eugenioq@alumnos.upm.es>
*
**************************************************************************/


/*************** CONSTANTS  *******************/

  #define BIG 100000

/*************** TYPES      *******************/

  typedef int row;
  #define ROW_TYPE INT
  typedef int col;
  #define COL_TYPE INT
  typedef double cost;
  #define COST_TYPE DOUBLE

/*************** FUNCTIONS  *******************/

extern cost lap(int dim, const cost * const *assigncost,
                col *rowsol, row *colsol, cost *u, cost *v);



