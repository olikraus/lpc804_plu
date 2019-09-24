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
