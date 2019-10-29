/*

  encode_func.h
  
  Copyright (C) 2019 Oliver Kraus (olikraus@gmail.com)

  This file is part of "pluc".

  "pluc" is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  "pluc" is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with DGC; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA  

*/  
  
#include "fsm.h"
#include "code_list.h"

int encode_Fan_In(fsm_type fsm );
int encode_IC_Relaxe(fsm_type fsm);
int encode_IC_All( fsm_type fsm );


int encode_BuildMachineWithFF(fsm_type fsm, char * ffType, MIN_SYMB_LIST constrList,
  CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode);
int encode_BuildMachine(fsm_type fsm, MIN_SYMB_LIST constrList,
  CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode);
int encode_BuildMachineToggleFF(fsm_type fsm, MIN_SYMB_LIST constrList,
  CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode);
