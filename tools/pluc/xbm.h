/*

  xbm.h

  finite state machine for eXtended burst mode machines

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

#ifndef _XBM_H
#define _XBM_H

#include <stddef.h>
#include <stdarg.h>
#include "b_set.h"
#include "b_il.h"
#include "dcube.h"


struct _xbm_grp_struct
{
  b_il_type st_list;
  int gr_pos;
  int user_val;
};
typedef struct _xbm_grp_struct *xbm_grp_type;

struct _xbm_transition_struct
{
  int src_state_pos;
  int dest_state_pos;
  dcube level_cond;     /* xbm_GetPiCond(x) */
  dcube edge_cond;      /* xbm_GetPiCond(x) */
  dcube start_cond;     /* xbm_GetPiCond(x) */
  dcube end_cond;       /* xbm_GetPiCond(x) */
  
  dcube in_start_cond;  /* xbm_GetPiIn(x) */
  dcube in_end_cond;    /* xbm_GetPiIn(x) */
  dcube in_ddc_start_cond;    /* directed don't care start cube */
  dcube in_ddc_end_cond;      /* directed don't care end cube */
  
  int visit_cnt;              /* walk algorithm */
  int visit_max;              /* walk algorithm */
};
typedef struct _xbm_transition_struct *xbm_transition_type;

#define XBM_TRANS_VA_LIST(t)\
  4,&((t)->level_cond),&((t)->edge_cond),&((t)->start_cond),&((t)->end_cond)

struct _xbm_state_struct
{ 
  char *name;
  dclist in_self_cl;
  dcube code;
  dcube out;
  int gr_pos;
  
  int d;      /* Bellman-Ford */
  int p;      /* Bellman-Ford */
};
typedef struct _xbm_state_struct *xbm_state_type;


#define XBM_DIRECTION_NONE -1
#define XBM_DIRECTION_IN 0
#define XBM_DIRECTION_OUT 1

struct _xbm_var_struct
{
  int direction;
  char *name;
  char *name2;
  int reset_value;
  int index;
  int is_level;       /* does the variable contain level checks */
  int is_edge;        /* does the variable contain edge triggered transitions */
  int is_used;        /* is the variable used? */
  int is_delay_line;  /* in out variable with specified delay */
  double delay;       /* value of the delay line (usually ns) */
};
typedef struct _xbm_var_struct *xbm_var_type;

typedef struct _xbm_struct *xbm_type;

#define XBM_RESET_NONE 0
#define XBM_RESET_LOW  1
#define XBM_RESET_HIGH 2

struct _xbm_struct
{
  int log_level;
  int is_file_check;
  
  int is_auto_ddc_level;
  int is_auto_ddc_edge;
  int is_auto_ddc_mix;    /* level AND edge */

  char *name;
  char *reset_state_name;
  int reset_st_pos;       /* assigned by xbm_set_reset_state_pos() */
  pinfo *pi_cond;
  pinfo *pi_in;
  pinfo *pi_out;
  int inputs;
  int outputs;
  b_set_type states;
  b_set_type transitions;
  b_set_type vars;
  b_set_type groups;
  
  pinfo *pi_async;    /* temporary used by xbmustt.c */
  dclist cl_async;    /* temporary used by xbmustt.c */
  
  pinfo *pi_code;     /* has only an output part */
  
  int is_fbo;         /* 0: only feed back code */
                      /* 1: feed back code+out */
                      
  int is_safe_opt;    /* 0: do only optimization that is safe */
                      /* 1: apply unsafe optimization */
                      /* assigned and changed in xbm_Build() */
                      
  int is_sync;        /* 0: asynchronous state machine */
                      /* 1: synchronous state machine */
                
  char *pla_file_xbm_partitions;
  char *pla_file_prime_partitions;
  char *pla_file_min_partitions;
  
  char *import_file_state_codes;
  char *import_file_pla_machine;
  
  pinfo *pi_machine;  /* result of xbmfn.c */
  dclist cl_machine;  /* result of xbmfn.c */
  
  char labelbuf[1024];
  
  void (*log_cb)(void *data, int ll, const char *fmt, va_list va);
  void *log_data;
  int  is_log_cb;
  
  /* xbmess.c */
  int (*ess_cb)(xbm_type x, void *data, dcube *n1, dcube *n2, dcube *n3);
  void *ess_data;
  int total_cnt;
  int ess_cnt;

  int (*tr_cb)(xbm_type x, void *data, int tr_pos, int var_in_idx, dcube *n1, dcube *n2);
  void *tr_data;

  /* xbmwalk.c */
  int (*walk_cb)(xbm_type x, void *data, int tr_pos, dcube *cs, dcube *ct, int is_reset, int is_unique);
  void *walk_data;
  int is_all_dc_change;
  
  /* xbmvhdl.c */
  
  char *tb_entity_name;
  char *tb_arch_name;
  char *tb_conf_name;
  char *tb_component_name;
  char *tb_clr_name;
  char *tb_clk_name;
  int tb_wait_time_ns;  
  int tb_reset_type;
  
};

#define xbm_GetPiCond(xbm) ((xbm)->pi_cond)
#define xbm_GetPiCode(xbm) ((xbm)->pi_code)
#define xbm_GetPiIn(xbm) ((xbm)->pi_in)
#define xbm_GetPiOut(xbm) ((xbm)->pi_out)
#define xbm_GetPiMachine(xbm) ((xbm)->pi_machine)

#define xbm_GetSt(xbm, pos)\
  ((xbm_state_type)b_set_Get((xbm)->states, (pos)))
  
#define xbm_GetStGrPos(xbm, pos) (xbm_GetSt((xbm), (pos))->gr_pos)

#define xbm_GetTr(xbm, pos)\
  ((xbm_transition_type)b_set_Get((xbm)->transitions, (pos)))

#define xbm_GetVar(xbm, pos)\
  ((xbm_var_type)b_set_Get((xbm)->vars, (pos)))

#define xbm_GetGr(xbm, pos)\
  ((xbm_grp_type)b_set_Get((xbm)->groups, (pos)))

#define xbm_GetTrSrcStPos(xbm, pos) (xbm_GetTr((xbm), (pos))->src_state_pos)
#define xbm_GetTrDestStPos(xbm, pos) (xbm_GetTr((xbm), (pos))->dest_state_pos)

#define xbm_GetStCnt(xbm) b_set_Cnt((xbm)->states)
#define xbm_GetGrCnt(xbm) b_set_Cnt((xbm)->groups)

#define xbm_LoopSt(xbm, pos_ptr)\
  b_set_WhileLoop((xbm)->states, (pos_ptr))

#define xbm_LoopTr(xbm, pos_ptr)\
  b_set_WhileLoop((xbm)->transitions, (pos_ptr))

#define xbm_LoopVar(xbm, pos_ptr)\
  b_set_WhileLoop((xbm)->vars, (pos_ptr))

#define xbm_LoopGr(xbm, pos_ptr)\
  b_set_WhileLoop((xbm)->groups, (pos_ptr))
  
/*
#define xbm_LoopGrSt(xbm, gr_pos, st_pos)\
  b_il_WhileLoop(xbm_GetGr((xbm), (gr_pos))->st_list, (st_pos))
*/
  



/* xbm.c */

xbm_type xbm_Open();
void xbm_ClearGr(xbm_type x);
void xbm_Clear(xbm_type x);
void xbm_Close(xbm_type x);
int xbm_SetName(xbm_type x, const char *name);
int xbm_SetResetStateName(xbm_type x, const char *name);
void xbm_SetLogCB(xbm_type x, void (*log_cb)(void *data, int ll, const char *fmt, va_list va), void *data);
void xbm_ClrLogCB(xbm_type x);
void xbm_LogVA(xbm_type x, int ll, const char *fmt, va_list va);
void xbm_Log(xbm_type x, int ll, const char *fmt, ...);
void xbm_ErrorVA(xbm_type x, const char *fmt, va_list va);
void xbm_Error(xbm_type x, const char *fmt, ...);
int xbm_LoopStOutTr(xbm_type x, int st_pos, int *loop_ptr, int *tr_pos_ptr);
int xbm_LoopStInTr(xbm_type x, int st_pos, int *loop_ptr, int *tr_pos_ptr);
int xbm_LoopStOutSt(xbm_type x, int st_pos, int *loop_ptr, int *st_pos_ptr);
int xbm_LoopStInSt(xbm_type x, int st_pos, int *loop_ptr, int *st_pos_ptr);
int xbm_AddEmptySt(xbm_type x);
int xbm_AddSt(xbm_type x, const char *name);
void xbm_DeleteStGr(xbm_type x, int st_pos);
int xbm_SetStGr(xbm_type x, int st_pos, int gr_pos);
int xbm_FindStByName(xbm_type x, const char *name);
int xbm_AddEmptyTr(xbm_type x);
int xbm_FindTr(xbm_type x, int src_state_pos, int dest_state_pos);
int xbm_FindTrByName(xbm_type x, const char *from, const char *to);
void xbm_DeleteTr(xbm_type x, int tr_pos);
int xbm_Connect(xbm_type x, int src_state_pos, int dest_state_pos);
int xbm_ConnectByName(xbm_type x, const char *from, const char *to, int is_create_states);
void xbm_Disconnect(xbm_type x, int src_state_pos, int dest_state_pos);
int xbm_AddVar(xbm_type x, int direction, const char *name, int reset_value);
int xbm_AddDelayLineVar(xbm_type x, int direction, const char *name, int reset_value, double delay);
int xbm_CalcPI(xbm_type x);
int xbm_FindVar(xbm_type x, int direction, const char *name);
const char *xbm_GetVarNameStrByIndex(xbm_type x, int direction, int index);
const char *xbm_GetVarNameStr(xbm_type x, int pos);
const char *xbm_GetVarName2Str(xbm_type x, int pos);
const char *xbm_GetStNameStr(xbm_type x, int pos);
xbm_var_type xbm_GetVarByIndex(xbm_type x, int direction, int index);

dcube *xbm_GetTrSuper(xbm_type x, int tr_pos);
int xbm_GetTrCond(xbm_type x, int tr_pos, dclist in_cond_cl);

int xbm_SetCodeWidth(xbm_type x, int code_with);

int xbm_SetPLAFileNameXBMPartitions(xbm_type x, const char *name);
int xbm_SetPLAFileNamePrimePartitions(xbm_type x, const char *name);
int xbm_SetPLAFileNameMinPartitions(xbm_type x, const char *name);
int xbm_SetImportFileNameStateCodes(xbm_type x, const char *name);
int xbm_SetImportFileNamePLAMachine(xbm_type x, const char *name);

int xbm_AssignLabelsToMachine(xbm_type x);

int xbm_GetNextStateCode(xbm_type x, int st_pos, dcube *c, 
  int *dest_st_pos, dcube *dest_code, dcube *dest_out);

/*
  XBM_BUILD_OPT_MIS   do state minimization
  XBM_BUILD_OPT_FBO   apply feedback optimization
  XBM_BUILD_OPT_SYNC  create a synchronous state machine
*/

#define XBM_BUILD_OPT_MIS  0x01
#define XBM_BUILD_OPT_FBO  0x02
#define XBM_BUILD_OPT_SYNC 0x04

int xbm_Build(xbm_type x, int opt);
int xbm_GetFeedbackWidth(xbm_type x);

/* xbmbms.c */

int xbm_IsHFInOutTransitionPossible(xbm_type x, int n1, int n2);
int xbm_IsHFOutOutTransitionPossible(xbm_type x, int n1, int n2);
int xbm_ReadBMS(xbm_type x, const char *name);
int IsValidXBMFile(const char *name);


/* xbmkiss.c */

int xbm_WriteKISS(xbm_type x, const char *name);

/* xbmvcg.c */

int xbm_WriteVCG(xbm_type x, const char *name);

/* xbmdot.c */

int xbm_WriteDOT(xbm_type x, const char *name);


/* xbmmis.c */

int xbm_MinimizeStates(xbm_type x);
int xbm_NoMinimizeStates(xbm_type x);

/* xbmustt.c */

int xbm_InitAsync(xbm_type x);
void xbm_DestroyAsync(xbm_type x);
int xbm_EncodeStates(xbm_type x);   /* reads x->is_fbo */
int xbm_WriteEncoding(xbm_type x, const char *name);

/* xbmfn.c */

void xbm_DestroyMachine(xbm_type x);
int xbm_InitMachine(xbm_type x);
int xbm_DoStateSelfTransfers(xbm_type x, int (*fn)(void *data, dcube *s, dcube *e), void *data);
int xbm_DoStateStateTransfers(xbm_type x, int (*fn)(void *data, dcube *s, dcube *e), void *data);
int xbm_BuildTransferFunction(xbm_type x);  /* reads x->is_fbo */
int xbm_WritePLA(xbm_type x, const char *name);
int xbm_WriteBEX(xbm_type x, const char *name);
int xbm_Write3DEQN(xbm_type x, const char *name);

/* xbmchk.c */

int xbm_CheckTransferFunction(xbm_type x);

/* xbmess.c */

/* cubes belong to x->pi_machine */
int xbm_CalcEssentialHazards(xbm_type x, 
  int (*ess_cb)(xbm_type x, void *data, dcube *n1, dcube *n2, dcube *n3), 
  void *data);

int xbm_DoTransitions(xbm_type x, 
  int (*tr_cb)(xbm_type x, void *data, int tr_pos, int var_in_idx, dcube *n1, dcube *n2), 
  void *data);


/* xbmwalk.c */

int xbm_DoWalk(xbm_type x, 
  int (*walk_cb)(xbm_type x, void *data, int tr_pos, dcube *cs, dcube *ct, int is_reset, int is_unique), 
  void *data);
int xbm_ShowWalk(xbm_type x);

/* xbmvhdl.c */

int xbm_vhdl_init(xbm_type x);
void xbm_vhdl_destroy(xbm_type x);
int xbm_WriteTestbenchVHDL(xbm_type x, const char *name);


#endif /* _XBM_H */
