/*

  fsmmis.c

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

  A new attempt to find a minimal number of states.
  Reference: Chris J. Myers "Asynchronous Circuit Design", Wiley 2001
  pages 140 - 154    

*/

#include <assert.h>
#include "mis.h"


/*-- interface --------------------------------------------------------------*/

int fsm_NewMinimizeStates(fsm_type fsm, int is_fbo)
{
  mis_type m;
  m = mis_Open(fsm, NULL);
  if ( m == NULL )
    return 0;
    
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
    fsm_Log(m->fsm, "FSM mis: Group merge failed.");
    return mis_Close(m), 0;
  }

  return mis_Close(m), 1;
}

int fsm_MinimizeStates(fsm_type fsm, int is_fbo, int is_old)
{
  if ( is_old != 0 )
    return fsm_OldMinimizeStates(fsm, is_fbo);
  if ( fsm_NewMinimizeStates(fsm, is_fbo) == 0 )
  {
    int n = -1;
    while( fsm_LoopNodes(fsm, &n) != 0 )
      fsm_DeleteNodeGroup(fsm, n);
    fsm_Log(fsm, "FSM mis: Minimization failed, using old method.");
    return fsm_OldMinimizeStates(fsm, is_fbo);
  }
  return 1;    
}
