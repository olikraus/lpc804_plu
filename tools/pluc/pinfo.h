/*

  pinfo.h
  
  problem information

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

#ifndef _PINFO_H
#define _PINFO_H

#include "cubedefs.h"
#include <stdio.h>
#include "b_sl.h"

/*
#define PINFO_MAX_IN 1024
#define PINFO_MAX_OUT 1024
*/
/* stack maximum: no of input vars + no of output vars */
#define PINFO_STACK_CUBES 1024
#define PINFO_USER_CUBES 20
#define PINFO_TMP_CUBES (PINFO_STACK_CUBES*2+PINFO_USER_CUBES)
/* tmp 0: tautologie cube */
/* tmp 1: dclSharp, dclD1Sharp, dclConsensus, clComplement, dclCofactor, mcovFindMaxAdditionalCol */
/*        dclIntersectionList, async_StatePrimes, async_PartitionCoverElement */
/*        dclSCCIntersectionCube, async_FindMax, dclPrimesUSTT, dclMinimizeUSTT */
/* tmp 2: dclTautology, dclPrimes, dclComplement, dclIrredundantMarkTaut */
/*        mcovGreedy, async_Greedy, xbm_calc_st_self_condition, */
/*        xbm_GetTrInSuper */
/* tmp 3: pinfo_out_best_solution (splitting algorithm), dclReadAllFP, fsm_SeparateState */
/*        fsm_bm_fn_output fsm_bmm_add_edge_output_transitions */
/*        xbm_BuildStateStateTransfers (xbmfn.c) */
/*        xbm_CheckStateStateTransfers (xbmchk.c) */
/*        xbm_build_sync_on_off_set (xbmfn.c) */
/*        xbm_CheckStaticTransferValues (xbmchk.c) */
/*        xbm_exec_st_st (xbmwalk.c) */
/* tmp 4: pinfo_out_best_solution (splitting algorithm), dclReadAllFP, fsm_SeparateState */
/*        fsm_bm_fn_output fsm_bmm_add_edge_output_transitions */
/*        xbm_BuildStateStateTransfers (xbmfn.c) */
/*        xbm_CheckStateStateTransfers (xbmchk.c) */
/*        xbm_exec_st_st (xbmwalk.c) */
/* tmp 5: dcSharpIn, dcD1SharpIn, dclIsBinateInVar, dclIsDCInVar, dclComplementCof */
/*        dclIsHazardfreeTransition */
/* tmp 6: dclComplementCof */
/*        dclIsHazardfreeTransition */
/*        mis_InitImplySet, mis_GetStatePairImplyCube, mis_GetStateImplyCL */
/*        mis_BuildMaximumCompatibleList */
/* tmp 7: dcExpand, dcWriteBin, dcReadBin, mis_BuildMaximumCompatibleList */
/* tmp 8: dclCheckTautology, hfp_AddStartTransitionCubes */
/*        xbm_IsHFOutOutTransitionPossible */
/* tmp 9: dclIrredundantMarkTautCof, fsm_EncodeSimple, fsm_BuildFunction */
/*        async_PartitionCoverElement, fsm_AddOutEdgeRequiredCubes */
/*        fsm_add_hazardfree_edge, fsm_BuildFunctionToggleFF */
/*        fsm_IsStateStable, fsm_DoInOutStableState, fsm_RequiredCubes */
/*        fsm_add_asynchronous_output */
/*        fsm_required_cubes_transition */
/*        fsm_bm_to_state_cube, fsm_bm_to_out_cube */
/*        fsm_GetNextNodeCode, xbm_GetNextStateCode */
/*        xbm_IsHFOutOutTransitionPossible */
/*        xbm_encode_synchronous_state_machine (xbmustt.c) */
/* tmp 10: fsm_BuildPartitionTable, fsm_add_hazardfree_edge */
/*         fsm_IsStateStable, fsm_DoInOutStableState */
/*         fsm_required_cubes_transition,  hfp_ToDHF */
/*         gspq_IFD_minterm (sydelay.c), xbm_BuildPartitionTable */
/*         dgsop.c */
/* tmp 11: fsm_partition_input, fsm_bms_line, fsm_bms_fill, fsm_bms_copy */
/*         dcHeuristicMinimizeHF (fsmbm.c) */
/*         fsm_AddOutputSeparation (fsmustt.c) */
/*         fsm_WithOutputPartitionTable (fsmustt.c) */
/*         nex_AddCube (nex.c) */
/*         xbm_separate_groups (xbmustt.c) */
/*         xbm_BuildStateStateTransfers (xbmfn.c) */
/* tmp 12: dclAdd() */
/* tmp 13: fsmtl_GoEdge(), dclComplementWithSharp, fsm_bmm_apply_transition */
/*         dcl_is_output_compatible(), fsm_GetAllInEdgeOutput */
/*         hfp_add_from_to_transition_out */
/* tmp 14: fsm_bmm_GetStableStartCondition, fsm_check_hazard_free_edge */
/*         dcl_is_output_compatible(), fsm_GetAllInEdgeOutput */
/*         xbm_BuildStateStateTransfers (xbmfn.c) */
/* tmp 15: gspq_IFD_minterm, xbm_find_reset_state, xbm_ess_check_ess_hazard, dclReplaceInOut */
/* tmp 16: hfp_AddFromToTransition, xbm_ess_check_ess_hazard, pinfoMerge, pinfoCopyByOut, dclReplaceInOut*/
/* tmp 17: hfp_AddFromToTransition, xbm_ess_check_ess_hazard, pluc */
/* stack1: dclTautologyCof, dclPrimesCof */
/* stack2: dclTautologyCof, dclPrimesCof */

/* number of solutions, that should be generate for the output's */
#define PINFO_OUT_SOL_CNT 5

struct _pinfo_split_struct
{
  unsigned *in_half_bit_cnt;
  
  int in_best_left_cnt;
  int in_best_right_cnt;
  int in_best_total_cnt;
  int in_best_var;

  unsigned *out_bit_cnt;
  int *out_cur_select;
  int out_cur_select_cnt;
  unsigned out_cur_sum;
  unsigned out_cur_cost;
  
  int *out_opt_select[PINFO_OUT_SOL_CNT];
  int out_opt_select_cnt[PINFO_OUT_SOL_CNT];
  unsigned out_opt_sum[PINFO_OUT_SOL_CNT];
  unsigned out_opt_cost[PINFO_OUT_SOL_CNT];
  int out_opt_grp_max_pos;
  unsigned out_opt_grp_max_cost;
  int out_opt_grp_cnt;
  
  int out_best_left_cnt;
  int out_best_right_cnt;
  int out_best_total_cnt;
  int out_best_pos;
  
  unsigned out_dest_sum;
  unsigned out_active_cnt;
  unsigned cube_cnt;
};
typedef struct _pinfo_split_struct pinfo_split_struct;

struct _pinfo_progress_struct
{
  char proc_name[10][80];
  int proc_depth;
  
  char procedure_fn_name[22];
  int procedure_max;
  
  char btree_fn_name[22];
  unsigned int btree_cnt;
  int btree_depth;
};
typedef struct _pinfo_progress_struct pinfo_progress_struct;

#define PINFO_CACHE_CL 32


/* BEGIN_LATEX pinfo.tex */
struct _pinfo_struct
{
  int in_cnt;
  int out_cnt;
  int in_words;
  int out_words;
  int in_out_words_min;
  c_int in_last_mask;
  c_int out_last_mask;
  
  dcube tmp[PINFO_TMP_CUBES];
  dcube *stack1;
  dcube *stack2;
  
  /* dclist cache, well it is a stack */
  dclist cache_cl[PINFO_CACHE_CL];
  int cache_cnt;
  
  /* universal list */
  dclist cl_u;
  
  /* progress */
  pinfo_progress_struct *progress;
  
  /* splitting */
  pinfo_split_struct *split;
 
  /* input and output labels */
  b_sl_type in_sl;
  b_sl_type out_sl;
};
typedef struct _pinfo_struct pinfo;
/* END_LATEX */

int bitcount(c_int x);


/* Init the problem information structure */
int pinfoInit(pinfo *pi);
int pinfoInitInOut(pinfo *pi, int in, int out);
int pinfoInitFromOutVar(pinfo *pi_dest, dclist *cl_dest_ptr, pinfo *pi_src, dclist cl_src, int out_var);

void pinfoDestroy(pinfo *pi);

pinfo *pinfoOpen();
pinfo *pinfoOpenInOut(int in, int out);
void pinfoClose(pinfo *pi);


int pinfoInitSplit(pinfo *pi);
void pinfoDestroySplit(pinfo *pi);

int pinfoSetInCnt(pinfo *pi, int in);
int pinfoSetOutCnt(pinfo *pi, int out);

#define pinfoGetInCnt(pi) ((pi)->in_cnt)
#define pinfoGetOutCnt(pi) ((pi)->out_cnt)

int pinfoAddInLabel(pinfo *pi, const char *s);
void pinfoDeleteInLabel(pinfo *pi, int pos);

int pinfoAddOutLabel(pinfo *pi, const char *s);
void pinfoDeleteOutLabel(pinfo *pi, int pos);


b_sl_type pinfoGetInLabelList(pinfo *pi);
b_sl_type pinfoGetOutLabelList(pinfo *pi);

int pinfoImportInLabels(pinfo *pi, const char *s, const char *delim);
int pinfoImportOutLabels(pinfo *pi, const char *s, const char *delim);

int pinfoCopyInLabels(pinfo *pi, b_sl_type sl);
int pinfoCopyOutLabels(pinfo *pi, b_sl_type sl);
int pinfoCopyLabels(pinfo *dest_pi, pinfo *src_pi);

const char *pinfoGetInLabel(pinfo *pi, int pos);
int pinfoSetInLabel(pinfo *pi, int pos, const char *s);
const char *pinfoGetOutLabel(pinfo *pi, int pos);
int pinfoSetOutLabel(pinfo *pi, int pos, const char *s);

int pinfoFindInLabelPos(pinfo *pi, const char *s);
int pinfoFindOutLabelPos(pinfo *pi, const char *s);

int pinfoCopy(pinfo *dest, pinfo *src);

int pinfoInitProgress(pinfo *pi);
void pinfoDestroyProgress(pinfo *pi);

int pinfoProcedure(pinfo *pi, char *fn_name, int curr);
int pinfoDepth(pinfo *pi, char *fn_name, int depth);

void pinfoProcedureInit(pinfo *pi, char *fn_name, int max);
void pinfoProcedureFinish(pinfo *pi);
int pinfoProcedureDo(pinfo *pi, int curr);

void pinfoBTreeInit(pinfo *pi, char *fn_name);
void pinfoBTreeFinish(pinfo *pi);
int pinfoBTreeStart(pinfo *pi);
void pinfoBTreeEnd(pinfo *pi);

int pinfoGetSplittingInVar(pinfo *pi);


int pinfoWrite(pinfo *pi, FILE *fp);
int pinfoRead(pinfo **pi, FILE *fp);

#endif /* _PINFO_H */

