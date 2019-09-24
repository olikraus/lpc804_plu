/*

  fsmenc.c

  finite state machine, based on b_set, b_il

  Partly Copyright (C) 2001 Oliver Kraus (olikraus@yahoo.com)

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
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#include "fsm.h"
#include "fsmenc.h"
#include "mwc.h"
#include "encode_func.h"

/*---------------------------------------------------------------------------*/

int fsm_MinimizeClockedMachine(fsm_type fsm)
{
  return dclMinimize(fsm->pi_machine, fsm->cl_machine);
}

/*---------------------------------------------------------------------------*/

int fsm_BuildClockedMachine(fsm_type fsm, int coding_type)
{
  fsm_Log(fsm, "FSM: State encoding.");
  
  switch(coding_type)
  {
    case FSM_ENCODE_FAN_IN:
      if ( encode_Fan_In(fsm) == 0 )
        return 0;
      fsm_Log(fsm, "FSM: Encoded Fan In.");
      fsm_Log(fsm, "FSM: Building control function for clocked machines.");
      if ( fsm_BuildMachine(fsm) == 0 )
        return 0;
      fsm_Log(fsm, "FSM: Minimizing control function for clocked machines.");
      if ( fsm_MinimizeClockedMachine(fsm) == 0 )
        return 0;
      break;
      
    case FSM_ENCODE_IC_ALL:
      if( encode_IC_All( fsm )==0)
      {
        if ( fsm_EncodeSimple(fsm) == 0 )		/* fsm.c */
          return 0;
        fsm_Log(fsm, "FSM: Encoded Simple - IC_ALL impossible.");
        fsm_Log(fsm, "FSM: Building control function for clocked machines.");
        if ( fsm_BuildMachine(fsm) == 0 )
          return 0;
        fsm_Log(fsm, "FSM: Minimizing control function for clocked machines.");
        if ( fsm_MinimizeClockedMachine(fsm) == 0 )
          return 0;
      }
   
      break;
      
    case FSM_ENCODE_IC_PART:
      if( encode_IC_Relaxe( fsm )==0)
      {
        if ( fsm_EncodeSimple(fsm) == 0 )
          return 0;
        fsm_Log(fsm, "FSM: Encoded Simple - IC_PART impossible.");
        fsm_Log(fsm, "FSM: Building control function for clocked machines.");
        if ( fsm_BuildMachine(fsm) == 0 )
          return 0;
        fsm_Log(fsm, "FSM: Minimizing control function for clocked machines.");
        if ( fsm_MinimizeClockedMachine(fsm) == 0 )
          return 0;
      }
      break;
      
    case FSM_ENCODE_SIMPLE:
      if ( fsm_EncodeSimple(fsm) == 0 )			/* fsm.c */
        return 0;
      fsm_Log(fsm, "FSM: Encoded Simple.");
      fsm_Log(fsm, "FSM: Building control function for clocked machines.");
      if ( fsm_BuildTransferfunction(fsm) == 0 )
        return 0;
      fsm_Log(fsm, "FSM: Minimizing control function for clocked machines.");
      /*
      if ( fsm_MinimizeClockedMachine(fsm) == 0 )
        return 0;
      */
      if ( dclMinimizeDC(fsm->pi_machine, fsm->cl_machine, fsm->cl_machine_dc, 0, 1) == 0 )
      {
	fsm_Log(fsm, "FSM: Minimizing failed.");
        return 0;
      }
      break;
  }
  return 1;
}
