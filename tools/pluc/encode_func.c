/*

  encode_func.c

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

#include <stdlib.h>

#include "fsm.h"
#include "fsmenc.h"
#include "node_list.h"
#include "code_list.h"
#include "cg_graph.h"
#include "cd_codes.h"
#include "min_symb_list.h"
#include "ic_list.h"
#include "index_table.h"
#include "encode_func.h"
#include "extra.h"


#define MAX_DIM_CODE_LIST_VECTOR 10

int encode_IC_Relaxe_aux(fsm_type fsm, int nrBits, char *method);


/* encoding based on the fan in of the FSM nodes */

/* --------- encode_Fan_In ---------------------------------*/

int encode_Fan_In(fsm_type fsm )
{
  NODE_LIST nodes, current;
  int fan_in, node_id, cntDC, nrBits;
  OUT_CODE_INFO codeInfo;
  dcube code;
  pinfo *pi = fsm_GetCodePINFO(fsm);
  
  nrBits  = extra_fsm_GetMinNrBits(fsm);

  if ( fsm_SetCodeWidth(fsm, nrBits, FSM_CODE_DFF_EXTRA_OUTPUT) == 0 )
    return 0;
   
  nodes = node_OpenNodeList();
  
  node_id = -1;
  while(fsm_LoopNodes(fsm, &node_id) != 0)
  {
    cntDC = 0;
    fan_in = extra_fsm_GetNodeFanIn(fsm, node_id, &cntDC);
    if( (nodes = (NODE_LIST) node_AddNodeByFanIn(nodes, node_id, fan_in, cntDC)) == NULL)
      return 0;
  }
  
  codeInfo = cd_OpenOutCodeInfo(nrBits);
  dcInit(pi, &code);
  
  for(current = nodes; current != NULL; current = current->next)
  {
    cd_GetCodeFromOutCodeInfo(codeInfo, pi, &code);
    dcCopy(pi, fsm_GetNodeCode(fsm, current->node_id), &code);
    cd_GetNextOutCodeInfo(codeInfo);
  }
  
  node_CloseNodeList(nodes);
  cd_CloseOutCodeInfo(codeInfo);
  
  dcDestroy(&code);
  return 1;
}
/* encode based on building the input constraint and trying to satisfy them partially */


/* --------- encode_IC_Relaxe ---------------------------------*/

int encode_IC_Relaxe(fsm_type fsm)
{
  int nrBits, maxBits, minBits;
  int OK;
  
  minBits = extra_fsm_GetMinNrBits(fsm);
  maxBits = b_set_Cnt(fsm->nodes); 
  
  OK = 0;
  for(nrBits = minBits; nrBits <= maxBits && !OK; nrBits++)
  {
    if(encode_IC_Relaxe_aux(fsm, nrBits, "IC_INCLUDE") != 0)
    {
      fsm_Log(fsm, "FSM: Encoded constraints partially.");
      fsm_Log(fsm, "FSM: Building control function for clocked machines.");
      fsm_Log(fsm, "FSM: Minimizing control function for clocked machines.");
      OK = 1;
    }
  }
  return OK;
}



/* encoding that builds the input constraints and tries to satisfy them all */


/* --------- encode_IC_All ---------------------------------*/

int encode_IC_All( fsm_type fsm )
{
  MIN_SYMB_LIST constrList;
  
  IC_LIST icList;
  
  CONSTR_GRAPH cGraph;
  
  INDEX_TABLE tab;
  
  int nrPrimConstr, OK, nrBits, rotate, minBits;
  //int nrConstr;
  CODE_LIST_VECTOR codeList;
  pinfo pCode, pConstr;
  //pinfo *pCond = fsm_GetConditionPINFO(fsm);
  
  KID_LIST bad;
  

  
  tab =  index_CreateTable( fsm );
  
  pinfoInit(&pConstr);
  constrList = (MIN_SYMB_LIST) min_BuildGroups(fsm,tab, &pConstr, "IC_INCLUDE");
  //nrConstr = min_GetCnt(constrList);
 
  icList = (IC_LIST) ic_CreateICList(constrList,  &pConstr);
  icList = (IC_LIST) ic_CreateICClosure(icList, &pConstr);

  cGraph = (CONSTR_GRAPH) cg_CreateGraph(icList, &pConstr);

  minBits = extra_GetMinNrBits(fsm, cGraph, &pConstr);
  nrPrimConstr = cg_GetNrPrimConstr(cGraph);
 
  OK = 0;
  for(nrBits = minBits; nrBits < minBits+3 && !OK; nrBits++)
  {
    for(rotate=0; rotate <= nrPrimConstr && !OK; rotate++)
    {
      if((codeList = code_OpenCodeListVector(MAX_DIM_CODE_LIST_VECTOR))==NULL)
        return 0;
        
      if(!pinfoInit(&pCode))
        return 0;
      pinfoSetInCnt(&pCode, nrBits);
      pinfoSetOutCnt(&pCode,0);
      
      bad = NULL;

      if( (code_AssignCodes(cGraph, codeList, &pConstr, &pCode, &bad)) == 1)  
      {
        OK = 1;

        code_CopyCodes(codeList, fsm, tab,  &pConstr, &pCode);
        
        if (encode_BuildMachine(fsm, constrList, codeList, &pConstr, &pCode)==0)
        {
          return 0;
        }

        if ( fsm_MinimizeClockedMachine(fsm) == 0 )
        {
          return 0;
        }
        fsm_Log(fsm, "FSM: Encoded all constraints.");
        fsm_Log(fsm, "FSM: Building control function for clocked machines.");
        fsm_Log(fsm, "FSM: Minimizing control function for clocked machines.");
        
        /* extra !!! */
        //extra_fsm_WriteEncoding(fsm, "example.codes");

      }
      else
      {
        cGraph = (CONSTR_GRAPH) cg_ChangeOrderOfPrimConstr(cGraph, &pConstr);
        code_CloseCodeListVector(codeList);
        codeList = NULL;
        pinfoDestroy(&pCode);
      }
    }
  }
   
  if(codeList)
    code_CloseCodeListVector(codeList);
  pinfoDestroy(&pCode);
  pinfoDestroy(&pConstr);
  min_CloseList(constrList);
  cg_CloseGraph(cGraph);
  ic_CloseList(icList);
  index_Close(tab);
  return OK;
}


/* --------- encode_IC_Relaxe_aux ---------------------------------*/

int encode_IC_Relaxe_aux(fsm_type fsm, int nrBits, char *method)
{
  MIN_SYMB_LIST constrList;
  
  IC_LIST icList;
  pinfo pConstr, pCode;
  
  CONSTR_GRAPH cGraph;

  CODE_LIST_VECTOR codeList;
  INDEX_TABLE tab;
  KID_LIST bad;
  
  int OK, wasPossible;
  
  tab =  index_CreateTable( fsm );
  
  pinfoInit(&pConstr);
  constrList = (MIN_SYMB_LIST) min_BuildGroups(fsm,tab, &pConstr, method);
 
  icList = (IC_LIST) ic_CreateICList(constrList,  &pConstr);
  icList = (IC_LIST) ic_CreateICClosure(icList, &pConstr);

  cGraph = (CONSTR_GRAPH) cg_CreateGraph(icList, &pConstr);
  
  if(pinfoInit(&pCode))
  pinfoSetInCnt(&pCode, nrBits);
  pinfoSetOutCnt(&pCode, 0);
  
  OK = 0;
  wasPossible = 0;
  while(!OK)
  {
    if((codeList = code_OpenCodeListVector(MAX_DIM_CODE_LIST_VECTOR))==NULL)
    {
      pinfoDestroy(&pCode);
      pinfoDestroy(&pConstr);
      min_CloseList(constrList);
      cg_CloseGraph(cGraph);
      ic_CloseList(icList);
      index_Close(tab);

      return 0;
    }
    
    bad = NULL;
    if( (code_AssignCodes(cGraph, codeList, &pConstr, &pCode, &bad)) == 1)  
    {
      wasPossible = 1;
      OK = 1;
    }
    else
    {
      if(extra_dcOutOneCnt(&(cGraph->nodes[bad->id]->constr), &pConstr) == 1)
      {
        wasPossible = 0;
        OK = 1;
      }
      else
      {
        cGraph = (CONSTR_GRAPH) cg_ReadaptGraph(cGraph, bad, &pConstr);
        code_CloseCodeListVector(codeList);
      }
    }

  }  
  
  if(wasPossible)
  {
    code_CopyCodes(codeList, fsm, tab, &pConstr, &pCode);
    
    if (encode_BuildMachine(fsm, constrList, codeList, &pConstr, &pCode)==0)
    {
      return 0;
    }

    if ( fsm_MinimizeClockedMachine(fsm) == 0 )
    {
      return 0;
    }


    OK = 1;
    
  }
  else
  {
    OK = 0;
  }

  if(codeList)
    code_CloseCodeListVector(codeList);
  pinfoDestroy(&pCode);
  pinfoDestroy(&pConstr);
  min_CloseList(constrList);
  cg_CloseGraph(cGraph);
  ic_CloseList(icList);
  index_Close(tab);
  return OK;
 
}



/* --------- encode_BuildMachineWithFF --------------------------------------*/

/*
 * int encode_BuildMachineWithFF(fsm_type fsm, char * ffType, MIN_SYMB_LIST constrList,
 *   CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode)
 * {
 * 
 *   if (strcmp(ffType, "DELAY")==0)
 *   {
 *     if ( encode_BuildMachine(fsm,  constrList, codeList, pConstr, pCode) == 0 )
 *       return 0;
 *   }
 *   else  
 *   {
 *     if (strcmp(ffType, "TOGGLE") == 0)    
 *     {   
 *       if ( encode_BuildMachineToggleFF(fsm, constrList, codeList, pConstr, pCode) == 0 )
 *         return 0;
 *     }
 *     else
 *     {
 *       if ( encode_BuildMachine(fsm, constrList, codeList, pConstr, pCode) == 0 )
 *         return 0;
 *     }
 *   }
 *   return 1;
 * }
 */

/* --------- encode_BuildMachine --------------------------------------*/

int encode_BuildMachine(fsm_type fsm, MIN_SYMB_LIST constrList,
  CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode)
{
  dcube group_code, dest_code;
  MIN_SYMB_LIST tmp;
  dcube c;
  
  dcInit(pCode, &group_code);
  dcInit(pCode, &dest_code);
  
  dcInit(fsm->pi_machine, &c);
  
  
 
  dclRealClear(fsm->cl_machine);
  fsm_BuildMachine(fsm);
  
  for(tmp = constrList; tmp != NULL; tmp = tmp->next)
  {
    if(tmp->is_valid)
    {
      if( (code_FindConstrInListVector(codeList, tmp->group, pConstr, &group_code, pCode)) == 0)
        return 0;
      
      if( (code_FindConstrInListVector(codeList, tmp->dest, pConstr, &dest_code, pCode)) == 0)
        return 0;
      
      dcInSetAll(fsm->pi_machine, &c, CUBE_IN_MASK_DC);
      dcOutSetAll(fsm->pi_machine, &c, 0);
   
      dcCopyInToIn(  fsm->pi_machine, &c, 0, fsm->pi_cond, tmp->condition);
      dcCopyInToIn( fsm->pi_machine, &c, fsm->pi_cond->in_cnt, pCode, &group_code);
      dcCopyInToOut(fsm->pi_machine, &c, 0, pCode, &dest_code);
    
      if ( dcIsIllegal(fsm->pi_machine, &c) == 0 )
        if ( dclAdd(fsm->pi_machine, fsm->cl_machine, &c) < 0 )
          return 0;
    }
  }
  dcDestroy(&group_code);
  dcDestroy(&dest_code);
  
  dcDestroy(&c);

  return 1;  
}


/* --------- encode_BuildMachineToggleFF --------------------------------------*/

/*
 * int encode_BuildMachineToggleFF(fsm_type fsm, MIN_SYMB_LIST constrList,
 *   CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode)
 * {
 *   dcube group_code, dest_code, dest_code_out, state_code_out;
 *   dcube code_tmp;
 *   dclist cl_codes;
 *   int cnt, i;
 *   MIN_SYMB_LIST tmp;
 *   dcube c;
 *   pinfo pCodeOut;
 *   
 *   dcInit(pCode, &group_code);
 *   dcInit(pCode, &dest_code);
 *   dclInit(&cl_codes);
 *   
 *   dcInit(fsm->pi_machine, &c);
 *   
 *   pinfoInit(&pCodeOut);
 *   pinfoSetInCnt(&pCodeOut, 0);
 *   pinfoSetOutCnt(&pCodeOut, pinfoGetInCnt(pCode));
 *   
 *   dcInit(&pCodeOut, &state_code_out);
 *   dcInit(&pCodeOut, &dest_code_out);
 *   dcInit(&pCodeOut, &code_tmp);
 *  
 *  
 *   dclRealClear(fsm->cl_machine);
 *   fsm_BuildMachineToggleFF(fsm);
 *   
 *   for(tmp = constrList; tmp != NULL; tmp = tmp->next)
 *   {
 *     if(tmp->is_valid)
 *     {
 *       if( (code_FindConstrInListVector(codeList, tmp->group, pConstr, &group_code, pCode)) == 0)
 *         return 0;
 *       
 *       if( (code_FindConstrInListVector(codeList, tmp->dest, pConstr, &dest_code, pCode)) == 0)
 *         return 0;
 *       
 *       if(dclAdd(pCode, cl_codes, &group_code) == -1)
 *         return 0;
 *       
 *       if (dclDontCareExpand(pCode, cl_codes) ==0)
 *         return 0;
 *         
 *       cnt = dclCnt(cl_codes);
 *       for(i = 0; i < cnt; i++)
 *       {
 *         dcInSetAll(fsm->pi_machine, &c, CUBE_IN_MASK_DC);
 *         dcOutSetAll(fsm->pi_machine, &c, 0);
 *         
 *         dcCopyInToIn(  fsm->pi_machine, &c, 0, fsm->pi_cond, tmp->condition);
 *         dcCopyInToIn( fsm->pi_machine, &c, fsm->pi_cond->in_cnt, pCode, dclGet(cl_codes, i));
 *         
 *         
 *         dcCopyInToOut(&pCodeOut, &dest_code_out, 0, pCode, &dest_code);
 *         dcCopyInToOut(&pCodeOut, &state_code_out, 0, pCode,dclGet(cl_codes, i) );
 *         
 *         dcToStr(&pCodeOut, &dest_code_out, "", "");
 *         dcToStr(&pCodeOut, &state_code_out, "", "");
 *         
 *         dcXorOut(&pCodeOut, &code_tmp, &state_code_out, &dest_code_out);
 *         dcCopyOutToOut(fsm->pi_machine, &c, 0, &pCodeOut, &code_tmp);
 * 
 *         if(!dclIsSubSet(pCode, fsm->cl_machine, &c))        
 *           if ( dcIsIllegal(fsm->pi_machine, &c) == 0 )
 *             if ( dclAdd(fsm->pi_machine, fsm->cl_machine, &c) < 0 )
 *               return 0;
 *         
 *       }
 *       
 *     }
 *   }
 *   dcDestroy(&group_code);
 *   dcDestroy(&dest_code);
 *   dclDestroy(cl_codes);
 *   
 *   dcDestroy(&c);
 *   
 *   dcDestroy(&state_code_out);
 *   dcDestroy(&dest_code_out);
 *   dcDestroy(&code_tmp);
 *   pinfoDestroy(&pCodeOut);
 * 
 *   return 1;  
 * }
 */
