#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 17 nov 2001  ok  removed: does not exist on FreeBSD */
/* #include <sys/ddi.h> */

#include "pinfo.h"
#include "dcube.h"
#include "fsm.h"
#include "cg_graph.h"


/*-- extra_dcOutOneCnt -----------------------------------------------------------*/

int extra_dcOutOneCnt(dcube *dc, pinfo *pi)
{
  /* returns number of bits equal to 1 in the output part of the cube dc == 
   */
   
  int nrBits;
  int cnt, i;
  
  cnt = 0;
  
  nrBits = pinfoGetOutCnt(pi);
  
  for(i=0; i < nrBits; i++)
    if(dcGetOut(dc, i) == 1)
      cnt++;
      
  return cnt;
}

/*-- extra_dcInGetDCCnt -----------------------------------------------------------*/

int extra_dcInGetDCCnt(dcube *dc, pinfo *pi)
{

  int nrBits;
  int cnt, i;

  cnt = 0;

  nrBits = pinfoGetInCnt(pi);

  for(i=0; i < nrBits; i++)
    if(dcGetIn(dc, i) == 3)
      cnt++;

  return cnt;
}

/*-- extra_dcOutGetOnePos -----------------------------------------------------------*/

int extra_dcOutGetOnePos(dcube *dc, pinfo *pi)
{
  /* returns the position of the 1 bit for cubes with dcOutOneCnt equal to 1
   * or -1 if dcOutOneCnt is > 1
   */
  int cnt, i;
  
  if(extra_dcOutOneCnt(dc, pi) != 1)
    return -1;
    
  cnt = pinfoGetOutCnt(pi);
  
  for(i=0; i< cnt; i++)
    if(dcGetOut(dc, i) == 1)
      return i;
      
  return -1;
}

/* added by Claudia Spircu; April 2001 */

/* --------- fsm_WriteEncoding ---------------------------------*/

void extra_fsm_WriteEncoding(fsm_type fsm, char * filename)
{
  /* writes the file named filename with the name of the states and the codes in the
   * format accepted by NOVA */
  FILE *fp;
  int node;
  
  fp = fopen(filename, "w");
  if ( fp == NULL )
    return;
    
  node = -1;
  while(fsm_LoopNodes(fsm, &node))
  {
    fprintf(fp, "scode %s %s\n", fsm_GetNodeName(fsm, node), dcToStr(fsm_GetCodePINFO(fsm),fsm_GetNodeCode(fsm, node) , "", ""));
  }
  
  fclose(fp);
 
}

/* --------- fsm_GetMinNrBits ---------------------------------*/

int extra_fsm_GetMinNrBits(fsm_type fsm)
{
  int state_cnt = b_set_Cnt(fsm->nodes);
  int log_2_state_cnt;
 
  if ( state_cnt <= 0 )
    return 0;
  
  state_cnt--;
  log_2_state_cnt = 0;
  while( state_cnt != 0 )
  {
    state_cnt /= 2;
    log_2_state_cnt++;
  }
  
  return log_2_state_cnt;

}

/* --------- fsm_GetNodeFanIn ---------------------------------*/

int extra_fsm_GetNodeFanIn(fsm_type fsm, int node_id, int *cntDC)
{
  /* returns the fan in of the node and in cntDC the total number of DC for all the input
   * conditions 
   */
  int cntFanIn, loop, edge_id, i;
  int edge_cond_dim;
  dclist cond;
  pinfo * pi = fsm_GetConditionPINFO(fsm);
  
  cntFanIn = 0;
  loop = -1;
  
  while(fsm_LoopNodeInEdges(fsm, node_id, &loop, &edge_id) != 0)
  {
    cond = fsm_GetEdgeCondition(fsm, edge_id);
    edge_cond_dim = dclCnt(cond);
    cntFanIn += edge_cond_dim;
    for(i = 0; i< edge_cond_dim; i++)
    {
      cntDC += extra_dcInGetDCCnt(dclGet(cond, i),pi );
    }
  }
    
  return cntFanIn;
}


/* --------- extra_fsm_WriteFSM --------------------------------------*/

void extra_fsm_WriteFSM(fsm_type fsm)
{
  int edge;

  edge = -1;
  while(fsm_LoopEdges(fsm, &edge))
  {
    printf("Edge nr %d\n", edge);
    printf("From node %s to node %s\n", fsm_GetNodeName(fsm, fsm_GetEdgeSrcNode(fsm, edge)),
        fsm_GetNodeName(fsm, fsm_GetEdgeDestNode(fsm, edge)));
    printf("Condition\n");
    dclShow(fsm_GetConditionPINFO(fsm), fsm_GetEdgeCondition(fsm, edge));
    printf("Output\n");
    dclShow(fsm_GetOutputPINFO(fsm), fsm_GetEdgeOutput(fsm, edge));
    printf("\n");
  }
}


#define MAX_BITS 50
#define MAX_STATE_CHAR_NO 100
#define MAX_LINE_CHAR_NO 1000

/* --------- extra_fsm_ReadCodes --------------------------------------*/

int extra_fsm_ReadCodes(fsm_type fsm, char* filename)
{
  FILE *f;

  char prefix[10];
  char stateCodeStr[MAX_BITS];
  char stateStr[MAX_STATE_CHAR_NO];
  char line[MAX_LINE_CHAR_NO];
  int nrBits;
  int node_id;
  dcube code;
  
  pinfo *pi = fsm_GetCodePINFO(fsm);

  if ( (f = fopen(filename, "rt")) == NULL)
    return 0;

  fgets(line, MAX_LINE_CHAR_NO, f);

  if( feof(f) != 0)
  {
    fclose(f);
    return 1;
  }
    
  sscanf(line, "%s %s %s", prefix, stateStr, stateCodeStr);
  strcpy(line, "\n");
  
  if(strcmp(prefix, "scode") != 0)
    return 0;
    
  nrBits = strlen(stateCodeStr);
  if(fsm_SetCodeWidth(fsm, nrBits, FSM_CODE_DFF_EXTRA_OUTPUT) == 0)
    return 0;
  dcInit(pi, &code);
  
  if((node_id = fsm_GetNodeIdByName(fsm, stateStr)) == -1)
    return 0;
  
  dcSetByStr(pi, &code, stateCodeStr);
  dcCopy(pi, fsm_GetNodeCode(fsm, node_id), &code);  
  
  while(feof(f) == 0 )
  {
    fgets(line, MAX_LINE_CHAR_NO, f);

    if(feof(f) != 0)
    {
      fclose(f);
      return 1;
    }

    if(strcmp(line, "\n") != 0)
    {
      sscanf(line, "%s %s %s",  prefix, stateStr, stateCodeStr);
      strcpy(line, "\n");

      if(strcmp(prefix, "scode") != 0)
        return 0;

      if((node_id = fsm_GetNodeIdByName(fsm, stateStr)) == -1)
        return 0;
  
      dcSetByStr(pi, &code, stateCodeStr);
      dcCopy(pi, fsm_GetNodeCode(fsm, node_id), &code);  

    }
  }
  fclose(f);
  dcDestroy(&code);
  return 1;
}

  
  
/* --------- extra_fsm_MakeCondPrimes --------------------------------------*/

int extra_fsm_MakeCondPrimes (fsm_type fsm)
{
  pinfo *pCond = fsm_GetConditionPINFO(fsm);
  int edge;
  
  edge = -1;
  while(fsm_LoopEdges(fsm, &edge))
  {
    if (dclPrimes(pCond, fsm_GetEdgeCondition(fsm, edge)) == 0)
      return 0;
  }
  
  return 1;
}
/* --------- extra_fsm_MakeCondMinimize --------------------------------------*/

int extra_fsm_MakeCondMinimize (fsm_type fsm)
{
  pinfo *pCond = fsm_GetConditionPINFO(fsm);
  int edge;
  
  edge = -1;
  while(fsm_LoopEdges(fsm, &edge))
  {
    if (dclMinimize(pCond, fsm_GetEdgeCondition(fsm, edge)) == 0)
      return 0;
  }
  
  return 1;
}


/* for the cost function */


/* --------- extra_dcNrInpCost --------------------------------------*/

int extra_dcNrInpCost(dcube *dc, pinfo *pi)
{
  /* returns number of bits equal to 1 or to 0 in the input part of the cube dc = the
   * number of literals in the product = the number of inputs for the AND gate 
   */
   
  int nrInp;
  int cnt, i;
  
  cnt = 0;
  
  nrInp = pinfoGetInCnt(pi);
  
  for(i=0; i < nrInp; i++)
    if(dcGetIn(dc, i) == 1 || dcGetIn(dc, i) == 2)
      cnt++;
      
  return cnt;
}

/* --------- extra_dclNrInpCost --------------------------------------*/

int extra_dclNrInpCost(dclist dcl, pinfo *pi, int * tab)
{
  /* returns the cost of the circuit realized by dcl = the total number of inputs
   * for all the gates
   */
   int nrInp, nrOutp, nrProd;
   int cost;
   int pos, p, found, cnt;
   
   nrInp = pinfoGetInCnt(pi);
   nrOutp = pinfoGetOutCnt(pi);
   
   nrProd = dclCnt(dcl);
   
   cost = 0;
   
   for(pos = 0; pos < nrOutp; pos++)
   {
    cnt = 0;
    for(p = 0; p < nrProd; p++)
    {
      if(dcGetOut(dclGet(dcl, p), pos) == 1)
      {
        if(tab[p] == 0)
        {
          /* the product wasn't considered before */
          tab[p] = 1;
          cost += extra_dcNrInpCost(dclGet(dcl, p), pi);
        }
        if(cnt == 0)
          cnt++;
        else
        {
          cost++;
          cnt++;
        }
      }
      if(cnt> 1)
        cost++;
    }
   }
   
   /* count the inverters */
   for(pos = 0; pos < nrInp; pos++)
   {
    found = 0;
    for(p = 0; p < nrProd && !found; p++)
    {
      if(dcGetIn(dclGet(dcl, p), pos) == 1)
      {
        cost ++;
        found = 1;
      }
    }
   }
   
   return cost;
}

/* --------- extra_fsm_NrInpCost --------------------------------------*/

int extra_fsm_NrInpCost(fsm_type fsm)
{
  int * tab;
  
  int nrProd, i, cost;
  
  nrProd = dclCnt(fsm->cl_machine);
  tab = (int*) malloc(nrProd * sizeof(int));
  for(i=0; i< nrProd; i++)
    tab[i] = 0;
    
  cost =  extra_dclNrInpCost(fsm->cl_machine, fsm->pi_machine,  tab);
    
  free(tab);
  
  return cost;
}

/* --------- extra_CostComb --------------------------------------*/

int extra_CostComb(fsm_type fsm)
{
  return 2 * extra_fsm_NrInpCost(fsm);
}

#define NR_TRANZ_DELAY_FF 20
#define NR_TRANZ_TOGGLE_FF 24

/* --------- extra_CostSeq --------------------------------------*/

int extra_CostSeq(fsm_type fsm, char *ffType)
{
  int nrBits = pinfoGetOutCnt(fsm_GetCodePINFO(fsm));
  if(strcmp(ffType, "TOGGLE")==0)
    return nrBits * NR_TRANZ_TOGGLE_FF;
  else
    return nrBits * NR_TRANZ_DELAY_FF;
}

/* --------- extra_CostTot --------------------------------------*/

int extra_CostTot(fsm_type fsm, char *ffType)
{
  return extra_CostSeq(fsm, ffType) + extra_CostComb(fsm);
}

 
/* --------- extra_GetMinNrBits --------------------------------------*/

int extra_GetMinNrBits(fsm_type fsm, CONSTR_GRAPH cGraph, pinfo *pConstr)
{
  /* 17 nov 2001  ok  removed function 'max' to be more portable... */

  int minBits, v1, v2;

  v1 = cg_GetMinNrBits1(cGraph, pConstr);
  v2 = cg_GetMinNrBits2(cGraph, pConstr);
  
  minBits = extra_fsm_GetMinNrBits(fsm);
  if ( minBits < v1 )
    minBits = v1;
  if ( minBits < v2 )
    minBits = v2;

  return minBits;
}

/* --------- extra_fsm_BuildMachineWithFF --------------------------------------*/

int extra_fsm_BuildMachineWithFF(fsm_type fsm, char *ffType)
{
  if (strcmp(ffType, "DELAY")==0)
  {
    if ( fsm_BuildMachine(fsm) == 0 )
      return 0;
  }
  else
  {  
    if (strcmp(ffType, "TOGGLE") == 0)    
    {
      if ( fsm_BuildMachineToggleFF(fsm) == 0 )
        return 0;
    }
    else
    {
      if ( fsm_BuildMachine(fsm) == 0 )
        return 0;
    
    }
  }
  return 1;
}

/*------ extra_GetFSM ---------------------------------------------------------------------*/

fsm_type extra_GetFSM(char *filename, char *outp)
{
  fsm_type fsm;
  char str[100];
  
  fsm = fsm_Open();
  sprintf(str,"./%s.kiss",  filename);
  if(fsm_ReadKISS(fsm, str) == 0)
    return NULL;
  /* with or without output */
  
  if(strcmp(outp, "NO_OUTPUT") == 0)
    fsm->is_with_output = 0;
  else
      fsm->is_with_output = 1;
      
  return fsm;
}
