/*

  dcubebcp.c

  interface to the BCP solver

  Copyright (C) 2001 Oliver Kraus (olikraus@yahoo.com)

  This file is part of DGC.

  DGC is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  DGC is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with DGC; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA  
  
  
  
  well ok, in general it is stupid to use a binate cover solver
  for unate problems. The interface should only exist for test purpose.
  I do expect that it is much slower than matrix.c
  
*/

#include <assert.h>
#include "dcube.h"
#include "b_bcp.h"

int dclBCP(pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc)
{
  pinfo local_pi;
  dclist cl_array, cl_out;
  b_bcp_type bc;
  int i, j;
  int problem_width;
  
  if ( dclCnt(cl_pr) <= 1 )
    return 1; /* all done */
    
  if ( pinfoInit(&local_pi) == 0 )
    return 0;
  
  if ( dclInitVA(2, &cl_out, &cl_array) == 0 )
    return pinfoDestroy(&local_pi), 0;


  /* create prime dependency */
    
  if (dclIrredundantDCubeTMatrixRC(&local_pi, cl_array, 
      pi, cl_es, cl_pr, cl_dc, NULL) == 0 )
    return dclDestroyVA(2, cl_out, cl_array), pinfoDestroy(&local_pi), 0;
  
  assert(dclCnt(cl_array) == dclCnt(cl_pr));
  problem_width = dclCnt(cl_array);
  
  bc = b_bcp_Open();
  if ( bc == NULL )
    return dclDestroyVA(2, cl_out, cl_array), pinfoDestroy(&local_pi), 0;

  if (dclClearFlags(cl_pr) == 0 || dclClearFlags(cl_array) == 0 )
    return dclDestroyVA(2, cl_out, cl_array), pinfoDestroy(&local_pi), 0;


  /* delete empty columns */
  /*
  for (i = 0; i < dclCnt(cl_array); i++)
    if (dcIsOutIllegal(&local_pi, dclGet(cl_array, i)) != 0)
    {
      dclSetFlag(cl_pr, i);
      dclSetFlag(cl_array, i);
    }
  dclDeleteCubesWithFlag(pi, cl_pr);
  dclDeleteCubesWithFlag(&local_pi, cl_array);
  */

  /* build the matrix */
    
  for (i = 0; i < problem_width; i++)
    for (j = 0; j < local_pi.out_cnt; j++)
      if (dcGetOut(dclGet(cl_array, i), j))
        if (b_bcp_Set(bc, i, j, BCN_VAL_ONE) == 0)
          return b_bcp_Close(bc), 
                 dclDestroyVA(2, cl_out, cl_array), pinfoDestroy(&local_pi), 0;
  
  dclDestroyVA(2, cl_out, cl_array);
  pinfoDestroy(&local_pi);

  
  /* solve the problem */
  
  if ( b_bcp_Do(bc) == 0 )
    return b_bcp_Close(bc), 0;
  assert(bc->is_no_solution == 0);


  /* store the result: reduce cl_pr to the number of required cubes */

  if ( dclClearFlags(cl_pr) == 0 )
    return b_bcp_Close(bc), 0;
  for( i = 0; i < problem_width; i++ )
    if ( b_bcp_Get(bc, i) != BCN_VAL_ONE )
      dclSetFlag(cl_pr, i);
  dclDeleteCubesWithFlag(pi, cl_pr);
  
  b_bcp_Close(bc);
      
  return 1;
}
/* this is a modified copy of dclMinimizeDC */
int dclMinimizeDCWithBCP(pinfo *pi, dclist cl, dclist cl_dc)
{
  dclist cl_es, cl_fr, cl_pr, cl_on;
  dclInitVA(4, &cl_es, &cl_fr, &cl_pr, &cl_on);
  
  if ( dclCopy(pi, cl_on, cl) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   
  
  if ( dclPrimesDC(pi, cl, cl_dc) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
  
  if ( dclSplitRelativeEssential(pi, cl_es, cl_fr, cl_pr, cl, cl_dc, NULL) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
  
  if ( dclBCP(pi, cl_es, cl_pr, cl_dc) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;

  if ( dclJoin(pi, cl_pr, cl_es) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
    
  dclRestrictOutput(pi, cl_pr);
  
  if( dclIsEquivalentDC(pi, cl_pr, cl_on, cl_dc) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
    
  if ( dclCopy(pi, cl, cl_pr) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   
    
  dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on);
  return 1;

}
