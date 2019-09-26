/*

  xbmmis.c

  minimization of finite state machines

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

*/

#include <assert.h>
#include "mis.h"

int xbm_NoMinimizeStates(xbm_type x)
{
  int st_pos = -1;

  xbm_ClearGr(x);

  while( xbm_LoopSt(x, &st_pos) != 0 )
    if ( xbm_SetStGr(x, st_pos, st_pos) == 0 )
      return 0;
  return 1;
}

int xbm_MinimizeStates(xbm_type x)
{
  int st_pos;
  mis_type m;
  m = mis_Open(NULL, x);
  if ( m == NULL )
    return 0;

  xbm_ClearGr(x);
    
  mis_MarkAlwaysCompatible(m);
  mis_MarkNotCompatible(m);
  if ( mis_MarkConditionalCompatible(m) == 0 )
    return mis_Close(m), 0;
  mis_UnmarkCondictionalCompatible(m);  

  /* mis_ShowStates(m); */

  if ( mis_CalculateMCL(m) == 0 )
    return mis_Close(m), 0;
  
  if ( mis_CopyMCLtoCList(m) == 0 )
    return mis_Close(m), 0;
  
  if ( mis_CalculateClassSets(m) == 0 )
    return mis_Close(m), 0;
  
  if ( mis_BuildPrimeCompatibleList(m) == 0 )
    return mis_Close(m), 0;
    
  /* mis_ShowClassSets(m); */

  if ( mis_CalculateBCP(m) == 0 )
    return mis_Close(m), 0;

  if ( mis_CalculateGroups(m) == 0 )
  {
    xbm_Log(x, 5, "XBM mis: Group merge failed, continue without state minimization.");
    mis_Close(m);
    return xbm_NoMinimizeStates(x);
  }

  st_pos = -1;
  while( xbm_LoopSt(x, &st_pos) != 0 )
  {
    assert(xbm_GetStGrPos(x, st_pos) >= 0);
  }
  
  xbm_Log(x, 4, "XBM: State minimization %d -> %d.", xbm_GetStCnt(x), xbm_GetGrCnt(x));

  return mis_Close(m), 1;
} 


