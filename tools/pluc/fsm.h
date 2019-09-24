/*

  fsm.h

  finite state machine

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

#ifndef _FSM_H
#define _FSM_H

#include <stdio.h>
#include <stdarg.h>
#include "b_set.h"
#include "b_il.h"
#include "b_sl.h"
#include "dcube.h"
#include "pinfo.h"
#include "dcex.h"

extern char *fsm_state_out_signal;
extern char *fsm_state_in_signal;


struct _fsmgrp_struct
{
  b_il_type node_group_list;
  int group_index;
};

typedef struct _fsmgrp_struct fsmgrp_struct;
typedef struct _fsmgrp_struct *fsmgrp_type;

struct _fsmnode_struct
{
  char *name;
  b_il_type out_edges;
  b_il_type in_edges;
  char *str_output;
  dcexn tree_output;
  dclist cl_output;
  dcube c_code;
  void *user_data; /* pointer to dg_node_struct, see dg_node.h */
  int user_val;
  /* shortest path problems, BFS, DFS */
  /* cormen, leiserson, rivest, p. 520 */
  int g_color;
  int g_d;
  int g_predecessor;
  int group_index;
};

typedef struct _fsmnode_struct fsmnode_struct;
typedef struct _fsmnode_struct *fsmnode_type;

struct _fsmedge_struct
{
  int source_node;
  int dest_node;
  char *str_cond;
  dclist cl_cond;     /* does not depend on number of states-variables */
  dcexn tree_cond;
  char *str_output;
  dclist cl_output;   /* does not depend on number of states-variables */
  dcexn tree_output;
  void *user_data;    /* pointer to dg_edge_struct, see dg_edge.h */
  int is_visited;
  dcube tmp_cube;     /* no automatic init or destroy is done on this cube */
};

typedef struct _fsmedge_struct fsmedge_struct;
typedef struct _fsmedge_struct *fsmedge_type;

#define FSM_LOG_FN_STACK_LEN 4

#define FSM_LINE_LEN (1024*16)

struct _fsm_struct
{
  char *name;

  int in_cnt;     /* number of input for the FSM */
  int out_cnt;    /* number of outputs for the FSM */

  b_set_type nodes;
  b_set_type edges;
  b_set_type groups;
  b_sl_type input_sl;
  b_sl_type output_sl;
  int reset_node_id;

  dcex_type dcex;
  pinfo *pi_cond;
  pinfo *pi_output;
  pinfo *pi_code;
  pinfo *pi_machine; 
  int code_width;
  
  int is_with_output;
  dclist cl_machine;
  dclist cl_machine_dc;
  
  dclist cl_m_state;
  dclist cl_m_output;
  
  pinfo *pi_m_state;
  pinfo *pi_m_output;
  
  /* async */
  pinfo *pi_async;
  dclist cl_async;
  
  /* log function */
  void (*log_fn[FSM_LOG_FN_STACK_LEN])(void *data, int ll, char *fmt, va_list va);
  void *log_data[FSM_LOG_FN_STACK_LEN];
  int log_default_level[FSM_LOG_FN_STACK_LEN];
  int log_stack_top;
  int log_dcl_max;      /* log only up to a certain number of entires */
  
  /* a buffer for fgets operations */
  char linebuf[FSM_LINE_LEN];
  
  /* a buffer for label operations */
  char labelbuf[64];
};

typedef struct _fsm_struct fsm_struct;
typedef struct _fsm_struct *fsm_type;

#define fsm_GetNode(fsm,pos) ((fsmnode_type)b_set_Get((fsm)->nodes,(pos)))
#define fsm_GetEdge(fsm,pos) ((fsmedge_type)b_set_Get((fsm)->edges,(pos)))
#define fsm_GetGroup(fsm,pos) ((fsmgrp_type)b_set_Get((fsm)->groups,(pos)))

#define fsm_GetNodeCnt(fsm) b_set_Cnt((fsm)->nodes)
#define fsm_GetGroupCnt(fsm) b_set_Cnt((fsm)->groups)

#define fsm_GetNodeName(fsm,node_id) (fsm_GetNode(fsm,node_id)->name)
#define fsm_GetNodeOutput(fsm,node_id) (fsm_GetNode(fsm,node_id)->cl_output)
#define fsm_GetNodeCode(fsm,node_id) (&(fsm_GetNode(fsm,node_id)->c_code))
#define fsm_GetNodeGroupIndex(fsm,node_id) (fsm_GetNode(fsm,node_id)->group_index)

#define fsm_GetEdgeSrcNode(fsm,edge_id) (fsm_GetEdge(fsm,edge_id)->source_node)
#define fsm_GetEdgeDestNode(fsm,edge_id) (fsm_GetEdge(fsm,edge_id)->dest_node)
#define fsm_GetEdgeOutput(fsm,edge_id) (fsm_GetEdge(fsm,edge_id)->cl_output)
#define fsm_GetEdgeCondition(fsm,edge_id) (fsm_GetEdge(fsm,edge_id)->cl_cond)

#define fsm_GetOutputPINFO(fsm) ((fsm)->pi_output)
#define fsm_GetConditionPINFO(fsm)   ((fsm)->pi_cond)
#define fsm_GetCodePINFO(fsm) ((fsm)->pi_code)

#define fsm_SetNodeUserData(fsm,pos,data) \
  (fsm_GetNode(fsm,pos)->user_data = (data))
#define fsm_SetEdgeUserData(fsm,pos,data) \
  (fsm_GetEdge(fsm,pos)->user_data = (data))

#define fsm_GetNodeUserData(fsm,pos) (fsm_GetNode(fsm,pos)->user_data)
#define fsm_GetEdgeUserData(fsm,pos) (fsm_GetEdge(fsm,pos)->user_data)

#define fsm_LoopNodes(fsm, node_id_ptr) \
  b_set_WhileLoop((fsm)->nodes, (node_id_ptr))
  
#define fsm_LoopEdges(fsm, edge_id_ptr) \
  b_set_WhileLoop((fsm)->edges, (edge_id_ptr))
  
#define fsm_LoopGroups(fsm, group_id_ptr) \
  b_set_WhileLoop((fsm)->groups, (group_id_ptr))
  
#define fsm_LoopGroupNodes(fsm, group_id, loop_ptr, node_id_ptr) \
  b_il_Loop(fsm_GetGroup((fsm), (group_id))->node_group_list, (loop_ptr), (node_id_ptr))
  
int fsm_LoopNodeOutEdges(fsm_type fsm, int node_id, int *loop_ptr, int *edge_id_ptr);
int fsm_LoopNodeInEdges(fsm_type fsm, int node_id, int *loop_ptr, int *edge_id_ptr);
int fsm_LoopNodeOutNodes(fsm_type fsm, int node_id, int *loop_ptr, int *node_id_ptr);
int fsm_LoopNodeInNodes(fsm_type fsm, int node_id, int *loop_ptr, int *node_id_ptr);

fsm_type fsm_Open();
void fsm_Clear(fsm_type fsm);
void fsm_Close(fsm_type fsm);

int fsm_SetName(fsm_type fsm, const char *name);

void fsm_LogVA(fsm_type fsm, char *fmt, va_list va);
void fsm_Log(fsm_type fsm, char *fmt, ...);
void fsm_LogLev(fsm_type fsm, int ll, char *fmt, ...);
/* expects triple: str, pi, cl, */
void fsm_LogDCL(fsm_type fsm, char *prefix, int cnt, ...);
void fsm_LogDCLLev(fsm_type fsm, int ll, char *prefix, int cnt, ...);
void fsm_PushLogFn(fsm_type fsm, void (*fn)(void *, int, char *, va_list), void *data, int ll);
void fsm_PopLogFn(fsm_type fsm);

int fsm_SetInCnt(fsm_type fsm, int in_cnt);
#define fsm_GetInCnt(fsm) ((fsm)->in_cnt)
int fsm_SetOutCnt(fsm_type fsm, int out_cnt);
#define fsm_GetOutCnt(fsm) ((fsm)->out_cnt)

#define FSM_CODE_DFF_EXTRA_OUTPUT 0
#define FSM_CODE_DFF_INCLUDED_OUTPUT 1

int fsm_SetCodeWidth(fsm_type fsm, int code_width, int code_option);
#define fsm_GetCodeWidth(fsm) ((fsm)->code_width)

dcube *fsm_GetResetCode(fsm_type fsm);
void fsm_SetResetNodeId(fsm_type fsm, int node_id);
int fsm_SetResetNodeIdByName(fsm_type fsm, const char *name);


int fsm_FindEdge(fsm_type fsm, int source_node_id, int dest_node_id);
void fsm_DeleteEdge(fsm_type fsm, int edge_id);

void fsm_DeleteNodeGroup(fsm_type fsm, int node_id);
int fsm_SetNodeGroup(fsm_type fsm, int node_id, int group_index);

int fsm_AddEmptyNode(fsm_type fsm);
void fsm_DeleteNode(fsm_type fsm, int node_id);

int fsm_Reconnect(fsm_type fsm, 
  int old_source_node_id, int old_dest_node_id, 
  int source_node_id, int dest_node_id);
int fsm_Connect(fsm_type fsm, 
  int source_node_id, int dest_node_id);
void fsm_Disconnect(fsm_type fsm, 
  int source_node_id, int dest_node_id);
int fsm_CopyEdge(fsm_type fsm, int dest_edge_id, int src_edge_id);
int fsm_ConnectAndCopy(fsm_type fsm, 
  int source_node_id, int dest_node_id, int edge_id);


int fsm_ExpandUniversalNodeByName(fsm_type fsm, char *name);

/* return position or -1 */
int fsm_AddNodeOutputCube(fsm_type fsm, int node_id, dcube *c);
int fsm_AddEdgeOutputCube(fsm_type fsm, int edge_id, dcube *c);
int fsm_AddEdgeConditionCube(fsm_type fsm, int edge_id, dcube *c);

int fsm_SetNodeOutputStr(fsm_type fsm, int node_id, char *s);
int fsm_SetEdgeOutputStr(fsm_type fsm, int edge_id, char *s);
int fsm_SetEdgeConditionStr(fsm_type fsm, int edge_id, char *s);

char *fsm_GetNodeNameStr(fsm_type fsm, int node_id);
int fsm_GetNodeIdByName(fsm_type fsm, const char *name);
int fsm_AddNodeByName(fsm_type fsm, char *name);
int fsm_ConnectByName(fsm_type fsm, char *source_node, char *dest_node, int is_create);
int fsm_FindEdgeByName(fsm_type fsm, char *source_node, char *dest_node);
void fsm_DisconnectByName(fsm_type fsm, char *source_node, char *dest_node);
void fsm_DeleteNodeByName(fsm_type fsm, char *name);
int fsm_DupNode(fsm_type fsm, int node_id);

int fsm_StrToTree(fsm_type fsm);
int fsm_TreeToCube(fsm_type fsm);

void fsm_ShowNodes(fsm_type fsm, FILE *fp);
void fsm_ShowEdges(fsm_type fsm, FILE *fp);

void fsm_Show(fsm_type fsm);

int fsm_EncodeSimple(fsm_type fsm);
int fsm_EncodeRandom(fsm_type fsm, int seed);
const char *fsm_GetInLabel(fsm_type fsm, int pos);
const char *fsm_GetOutLabel(fsm_type fsm, int pos, const char *s);
const char *fsm_GetStateLabel(fsm_type fsm, int pos, const char *prefix);
/* code_option: see fsm_SetCodeWidth() */
int fsm_AssignLabelsToMachine(fsm_type fsm, int code_option);
int fsm_BuildMachine(fsm_type fsm);
int fsm_BuildTransferfunction(fsm_type fsm);
int fsm_BuildMachineToggleFF(fsm_type fsm);


/* int fsm_MinimizeClockedMachine(fsm_type fsm); */ /* moved to fsmenc.h */
/* int fsm_BuildClockedMachine(fsm_type fsm); */   /* moved to fsmenc.h */

/* these two function INCLUDE the self condition */
int fsm_GetNodeInCover(fsm_type fsm, int node_id, dclist cl);
int fsm_GetNodeOutCover(fsm_type fsm, int node_id, dclist cl);

int fsm_GetNodeInOCover(fsm_type fsm, int node_id, dclist cl);

/* these two function EXCLUDE the self condition */
int fsm_GetNodePreCover(fsm_type fsm, int node_id, dclist cl);
int fsm_GetNodePostCover(fsm_type fsm, int node_id, dclist cl);

int fsm_GetNodePreOCover(fsm_type fsm, int node_id, dclist cl);
int fsm_GetNodePostOCover(fsm_type fsm, int node_id, dclist cl);

/* hmmm.. the following two functions should be replaced by the OCover fns */
int fsm_GetNodePreOutput(fsm_type fsm, int node_id, dclist cl);
int fsm_GetNodePostOutput(fsm_type fsm, int node_id, dclist cl);


dclist fsm_GetNodeSelfCondtion(fsm_type fsm, int node_id);
dclist fsm_GetNodeSelfOutput(fsm_type fsm, int node_id);

void fsm_GetNextNodeCode(fsm_type fsm, int node_id, dcube *c, int *dest_node_id, dcube *dest_code);

int fsm_IsValid(fsm_type fsm);

int fsm_WriteBin(fsm_type fsm, char *filename);
int fsm_ReadBin(fsm_type fsm, char *filename);

/* tries several formats */
int IsValidFSMFile(const char *filename);
int fsm_Import(fsm_type fsm, const char *filename);

/* read/write state encoding */
int fsm_WriteStateEncodingFile(fsm_type fsm, char *filename);
int fsm_EncodeByFile(fsm_type fsm, char *filename);


/* fsmsic.c */

int fsm_RestrictSIC(fsm_type fsm);
int fsm_CheckSIC(fsm_type fsm);

#define FSM_ASYNC_ENCODE_ZERO_NONE 0
#define FSM_ASYNC_ENCODE_ZERO_MAX 1
#define FSM_ASYNC_ENCODE_ZERO_RESET 2
#define FSM_ASYNC_ENCODE_WITH_OUTPUT 3
int fsm_EncodePartition(fsm_type fsm, int option);

int fsm_BuildPartitionTable(fsm_type fsm);    /* internal, used by bmsencode */
int fsm_SeparateStates(fsm_type fsm);         /* internal, used by bmsencode */
int fms_UseOutputFeedbackRemovePartitions(fsm_type fsm); /* internal */


/* fsmrc.c */
int fsm_RequiredCubes(fsm_type fsm, dclist cl_rc);

/* fsmasync.c */

int fsm_CheckHazardfreeMachine(fsm_type fsm, int is_self_transition);
int fsm_BuildAsynchronousMachine(fsm_type fsm, int do_check);

/* fsmstable.c */

#define FSM_STABLE_MINIMAL      0
#define FSM_STABLE_CONTINOUS    1
#define FSM_STABLE_IN_OUT       2
#define FSM_STABLE_FULL         3
int fsm_IsStateStable(fsm_type fsm, int node_id, int st, int is_log);
int fsm_IsStable(fsm_type fsm, int st, int is_log);
int fsm_DoMinimalStable(fsm_type fsm, int is_log);
int fsm_DoInOutStable(fsm_type fsm, int is_log);

/* fsmvhd.c */
/* requires fsm_BuildClockedMachine or fsm_BuildAsynchronousMachine */

/* #define FSM_VHDL_OPT_STATE_PORT 0x01 */
/* #define FSM_VHDL_OPT_STATE_DUB  0x02 */
#define FSM_VHDL_OPT_CLR_ON_LOW 0x04
#define FSM_VHDL_OPT_CHK_GLITCH 0x08
#define FSM_VHDL_OPT_FBO        0x10

int fsm_WriteVHDL(fsm_type fsm, char *filename, char *entity, char *arch, char *clock, char *reset, int opt);

/* fsmmoore.c */
int fsm_ConvertToMoore(fsm_type fsm);

/* fsmsp.c */
/* -1 for error */
int fsm_DoShortestPath(fsm_type fsm, int src_id, int dest_id, int fn(void *data, fsm_type fsm, int edge_id, int i, int cnt), void *data);
void fsm_ShowAllShortestPath(fsm_type fsm);

/* fsmbm.c  */
/* for enc_options see fsm_EncodePartition() */
int fsm_BuildRobustBurstModeTransferFunction(fsm_type fsm);
int fsm_BuildBurstModeMachine(fsm_type fsm, int check_level, int enc_option);

/* fsmbmm.c the new and small burst mode machine creation algorithm */
int fsm_BuildSmallBurstModeTransferFunction(fsm_type fsm, int enc_option);


/* fsmkiss.c */
int IsValidKISSFile(const char *name);
int fsm_ReadKISS(fsm_type fsm, const char *name);
int fsm_WriteKISS(fsm_type fsm, const char *name);

/* fsmqfsm.c */
int fsm_IsValidQFSMFile(const char *name);
int fsm_ReadQFSM(fsm_type fsm, const char *name);


/* fsmbms.c */
int IsValidBMSFile(const char *name);
int fsm_ReadBMS(fsm_type fsm, const char *name);

/* fsmcode.c */
int fsm_ExpandCodes(fsm_type fsm);

/* fsmmin.c */
/* old method */
int fsm_OldMinimizeStates(fsm_type fsm, int is_fbo);

/* fsmmis.c */
/* new method */
int fsm_MinimizeStates(fsm_type fsm, int is_fbo, int is_old);

#endif /* _FSM_H */
