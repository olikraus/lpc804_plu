/*

  extra.h

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


#include "pinfo.h"
#include "dcube.h"
#include "fsm.h"
#include "cg_graph.h"


int extra_dcOutOneCnt(dcube *dc, pinfo *pi);
int extra_dcInGetDCCnt(dcube *dc, pinfo *pi);
int extra_dcOutGetOnePos(dcube *dc, pinfo *pi);
void extra_fsm_WriteEncoding(fsm_type fsm, char * filename);
int extra_fsm_GetMinNrBits(fsm_type fsm);
int extra_fsm_GetNodeFanIn(fsm_type fsm, int node_id, int *cntDC);
void extra_fsm_WriteFSM(fsm_type fsm);
int extra_fsm_ReadCodes(fsm_type fsm, char* filename);
int extra_dcNrInpCost(dcube *dc, pinfo *pi);
int extra_dclNrInpCost(dclist dcl, pinfo *pi, int * tab);
int extra_fsm_NrInpCost(fsm_type fsm);
int extra_CostComb(fsm_type fsm);
int extra_CostSeq(fsm_type fsm, char *ffType);
int extra_CostTot(fsm_type fsm, char *ffType);
int extra_GetMinNrBits(fsm_type fsm, CONSTR_GRAPH cGraph, pinfo *pConstr);
int extra_fsm_BuildMachineWithFF(fsm_type fsm, char *ffType);
fsm_type extra_GetFSM(char *filename, char *outp);
