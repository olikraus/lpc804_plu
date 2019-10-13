/*

  fsm.c

  finite state machine, based on b_set, b_il

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


#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#include "fsm.h"
#include "mwc.h"
#include "b_io.h"

/*---------------------------------------------------------------------------*/

char *fsm_state_out_signal = "zo";
char *fsm_state_in_signal = "zi";


/*---------------------------------------------------------------------------*/

static int internal_fsm_set_str(char **s, const char *name)
{
  if ( *s != NULL )
    free(*s);
  *s = NULL;
  if ( name != NULL )
  {
    *s = strdup(name);
    if ( *s == NULL )
      return 0;
  }
  return 1;
}

/*---------------------------------------------------------------------------*/

static int fsmnode_SetOutput(fsmnode_type n, char *s)
{
  return internal_fsm_set_str(&(n->str_output), s);
}

static int fsmnode_SetName(fsmnode_type n, char *s)
{
  return internal_fsm_set_str(&(n->name), s);
}

static fsmnode_type fsmnode_Open(fsm_type fsm)
{
  fsmnode_type n;
  n = (fsmnode_type)malloc(sizeof(fsmnode_struct));
  if ( n != NULL )
  {
    n->group_index = -1;
    n->name = NULL;
    n->user_data = NULL;
    n->str_output = NULL;
    n->tree_output = NULL;
    n->out_edges = b_il_Open();
    if ( n->out_edges != NULL )
    {
      n->in_edges = b_il_Open();
      if ( n->in_edges != NULL )
      {
        if ( dclInit(&(n->cl_output)) != 0 )
        {
          n->str_output = NULL;
          if ( dcInit(fsm->pi_code, &(n->c_code)) != 0 )
          {
            return n;
          }
          dclDestroy(n->cl_output);
        }
        b_il_Close(n->in_edges);
      }
      b_il_Close(n->out_edges);
    }
    free(n);
  }
  return NULL;
}

static void fsmnode_Close(fsmnode_type n)
{
  dcDestroy(&(n->c_code));
  dclDestroy(n->cl_output);
  fsmnode_SetOutput(n, NULL);
  fsmnode_SetName(n, NULL);
  b_il_Close(n->in_edges);
  b_il_Close(n->out_edges);
  n->in_edges = NULL;
  n->out_edges = NULL;
  if ( n->tree_output != NULL )
    dcexnClose(n->tree_output);
  free(n);
}

static int fsmnode_Write(fsmnode_type n, FILE *fp)
{
  if ( b_io_WriteString(fp, n->name) == 0 )       return 0;
  if ( b_il_Write(n->out_edges, fp) == 0 )        return 0;
  if ( b_il_Write(n->in_edges, fp) == 0 )         return 0;
  if ( b_io_WriteString(fp, n->str_output) == 0 ) return 0;
  return 1;
}

static int fsmnode_Read(fsmnode_type n, FILE *fp)
{
  fsmnode_SetOutput(n, NULL);
  fsmnode_SetName(n, NULL);
  if ( b_io_ReadAllocString(fp, &(n->name)) == 0 )        return 0;
  if ( b_il_Read(n->out_edges, fp) == 0 )                 return 0;
  if ( b_il_Read(n->in_edges, fp) == 0 )                  return 0;
  if ( b_io_ReadAllocString(fp, &(n->str_output)) == 0 )  return 0;
  return 1;
}


/*---------------------------------------------------------------------------*/

static fsmgrp_type fsmgrp_Open(b_il_type il, int group_index)
{
  fsmgrp_type grp;
  grp = (fsmgrp_type)malloc(sizeof(fsmgrp_struct));
  if ( grp != NULL )
  {
    grp->group_index = group_index;
    grp->node_group_list = b_il_Open();
    if ( grp->node_group_list != NULL )
    {
      if ( il != NULL )
      {
        if ( b_il_Copy(grp->node_group_list, il) != 0 )
        {
          return grp;
        }
      }
      else
      {
        return grp;
      }
      b_il_Close(grp->node_group_list);
    }
    free(grp);
  }
  return NULL;
}

static void fsmgrp_Clear(fsmgrp_type grp)
{
  b_il_Clear(grp->node_group_list);
  grp->group_index = -1;
}

static void fsmgrp_Close(fsmgrp_type grp)
{
  fsmgrp_Clear(grp);
  b_il_Close(grp->node_group_list);
  free(grp);
}

/*---------------------------------------------------------------------------*/

static int fsmedge_SetCond(fsmedge_type e, char *s)
{
  return internal_fsm_set_str(&(e->str_cond), s);
}

static int fsmedge_SetOutput(fsmedge_type e, char *s)
{
  return internal_fsm_set_str(&(e->str_output), s);
}

static fsmedge_type fsmedge_Open(fsm_type fsm)
{
  fsmedge_type e;
  e = (fsmedge_type)malloc(sizeof(fsmedge_struct));
  if ( e != NULL )
  {
    e->user_data = NULL;
    e->source_node = -1;
    e->dest_node = -1;
    e->str_cond = 0;
    e->tree_cond = NULL;
    e->str_output = 0;
    e->tree_output = NULL;
    if ( dclInit(&(e->cl_output)) != 0 )
    {
      if ( dclInit(&(e->cl_cond)) != 0 )
      {
        return e;
      }
      dclDestroy(e->cl_output);
    }
    free(e);
  }
  return NULL;
}

static void fsmedge_Close(fsmedge_type e)
{
  fsmedge_SetCond(e, NULL);
  fsmedge_SetOutput(e, NULL);
  dclDestroy(e->cl_output);
  dclDestroy(e->cl_cond);
  e->source_node = -1;
  e->dest_node = -1;
  if ( e->tree_cond != NULL )
    dcexnClose(e->tree_cond);
  if ( e->tree_output != NULL )
    dcexnClose(e->tree_output);
  free(e);
}

static int fsmedge_Write(fsmedge_type e, FILE *fp)
{
  if ( b_io_WriteInt(fp, e->source_node) == 0 )           return 0;
  if ( b_io_WriteInt(fp, e->dest_node) == 0 )             return 0;
  if ( b_io_WriteString(fp, e->str_cond) == 0 )           return 0;
  if ( b_io_WriteString(fp, e->str_output) == 0 )         return 0;
  return 1;
}

static int fsmedge_Read(fsmedge_type e, FILE *fp)
{
  fsmedge_SetOutput(e, NULL);
  fsmedge_SetCond(e, NULL);
  if ( b_io_ReadInt(fp, &(e->source_node)) == 0 )         return 0;
  if ( b_io_ReadInt(fp, &(e->dest_node)) == 0 )           return 0;
  if ( b_io_ReadAllocString(fp, &(e->str_cond)) == 0 )    return 0;
  if ( b_io_ReadAllocString(fp, &(e->str_output)) == 0 )  return 0;
  return 1;
}


/*---------------------------------------------------------------------------*/

static void fsm_log_fn(void *data, int ll, char *fmt, va_list va)
{
  vprintf(fmt, va);
  puts("");
}

static int fsm_Init(fsm_type fsm)
{
  fsm->name = NULL;

  fsm->in_cnt = 0;
  fsm->out_cnt = 0;
  
  fsm->log_fn[0] = fsm_log_fn;
  fsm->log_data[0] = fsm;
  fsm->log_stack_top = 0;
  fsm->log_dcl_max = 300;

  fsm->reset_node_id = -1;
  fsm->code_width = 0;

  fsm->pi_async = NULL;
  fsm->cl_async = NULL;

  fsm->is_with_output = 1;
  fsm->cl_machine = NULL;
  fsm->cl_machine_dc = NULL;
  fsm->cl_m_state = NULL;
  fsm->cl_m_output = NULL;

  fsm->linebuf[0] = '\0';
  fsm->labelbuf[0] = '\0';

  fsm->nodes = b_set_Open();
  fsm->edges = b_set_Open();
  fsm->groups = b_set_Open();
  fsm->input_sl = b_sl_Open();
  fsm->output_sl = b_sl_Open();

  fsm->dcex = dcexOpen();
  
  fsm->pi_cond = pinfoOpen();
  fsm->pi_output = pinfoOpen();
  fsm->pi_code = pinfoOpen();
  fsm->pi_machine = pinfoOpen();
  dclInit(&(fsm->cl_machine));
  dclInit(&(fsm->cl_machine_dc));
  
  dclInit(&(fsm->cl_m_state));
  dclInit(&(fsm->cl_m_output));

  fsm->pi_m_state = pinfoOpen();
  fsm->pi_m_output = pinfoOpen();
  
  if (  fsm->nodes          == NULL || 
        fsm->edges          == NULL ||
        fsm->groups         == NULL ||
        fsm->input_sl       == NULL || 
        fsm->output_sl      == NULL ||
        fsm->dcex           == NULL ||
        fsm->pi_cond        == NULL ||
        fsm->pi_output      == NULL ||
        fsm->pi_code        == NULL ||
        fsm->pi_machine     == NULL ||
        fsm->cl_machine     == NULL ||
        fsm->cl_machine_dc  == NULL ||
        fsm->cl_m_state     == NULL ||
        fsm->cl_m_output    == NULL ||
        fsm->pi_m_state     == NULL ||
        fsm->pi_m_output    == NULL
        )
  {
    if ( fsm->nodes != NULL )          b_set_Close(fsm->nodes);
    if ( fsm->edges != NULL )          b_set_Close(fsm->edges);
    if ( fsm->groups != NULL )         b_set_Close(fsm->groups);
    if ( fsm->input_sl != NULL )       b_sl_Close(fsm->input_sl);
    if ( fsm->output_sl != NULL )      b_sl_Close(fsm->output_sl);
    if ( fsm->dcex != NULL )           dcexClose(fsm->dcex);
    if ( fsm->pi_cond != NULL )        pinfoClose(fsm->pi_cond);
    if ( fsm->pi_output != NULL )      pinfoClose(fsm->pi_output);
    if ( fsm->pi_code != NULL )        pinfoClose(fsm->pi_code);
    if ( fsm->pi_machine != NULL )     pinfoClose(fsm->pi_machine);
    if ( fsm->cl_machine != NULL )     dclDestroy(fsm->cl_machine);
    if ( fsm->cl_machine_dc != NULL )  dclDestroy(fsm->cl_machine_dc);
    if ( fsm->cl_m_state != NULL )     dclDestroy(fsm->cl_m_state);
    if ( fsm->cl_m_output != NULL )    dclDestroy(fsm->cl_m_output);
    if ( fsm->pi_m_state != NULL )     pinfoClose(fsm->pi_m_state);
    if ( fsm->pi_m_output != NULL )    pinfoClose(fsm->pi_m_output);


    return 0;
  }
        
  return 1;
}

fsm_type fsm_Open()
{
  fsm_type fsm;
  fsm = (fsm_type)malloc(sizeof(fsm_struct));
  if ( fsm != NULL )
  {
    if ( fsm_Init(fsm) != 0 )
    {
      return fsm;
    }
    free(fsm);
  }
  return NULL;
}

void fsm_Clear(fsm_type fsm)
{
  int i;
  i = -1;
  while( b_set_WhileLoop(fsm->nodes, &i) != 0 )
    fsmnode_Close(fsm_GetNode(fsm, i));
  b_set_Clear(fsm->nodes);
  i = -1;
  while( b_set_WhileLoop(fsm->edges, &i) != 0 )
    fsmedge_Close(fsm_GetEdge(fsm, i));
  b_set_Clear(fsm->edges);
  i = -1;
  while( b_set_WhileLoop(fsm->groups, &i) != 0 )
    fsmgrp_Close(fsm_GetGroup(fsm, i));
  b_set_Clear(fsm->groups);
  fsm->reset_node_id = -1;
}

void fsm_Close(fsm_type fsm)
{
  fsm_Clear(fsm);

  internal_fsm_set_str(&(fsm->name), NULL);


  if ( fsm->nodes != NULL )          b_set_Close(fsm->nodes);
  if ( fsm->edges != NULL )          b_set_Close(fsm->edges);
  if ( fsm->groups != NULL )         b_set_Close(fsm->groups);
  if ( fsm->input_sl != NULL )       b_sl_Close(fsm->input_sl);
  if ( fsm->output_sl != NULL )      b_sl_Close(fsm->output_sl);
  if ( fsm->dcex != NULL )           dcexClose(fsm->dcex);
  if ( fsm->pi_cond != NULL )        pinfoClose(fsm->pi_cond);
  if ( fsm->pi_output != NULL )      pinfoClose(fsm->pi_output);
  if ( fsm->pi_code != NULL )        pinfoClose(fsm->pi_code);
  if ( fsm->pi_machine != NULL )     pinfoClose(fsm->pi_machine);
  if ( fsm->cl_machine != NULL )     dclDestroy(fsm->cl_machine);
  if ( fsm->cl_machine_dc != NULL )  dclDestroy(fsm->cl_machine_dc);
  if ( fsm->cl_m_state != NULL )     dclDestroy(fsm->cl_m_state);
  if ( fsm->cl_m_output != NULL )    dclDestroy(fsm->cl_m_output);
  if ( fsm->pi_m_state != NULL )     pinfoClose(fsm->pi_m_state);
  if ( fsm->pi_m_output != NULL )    pinfoClose(fsm->pi_m_output);

  if ( fsm->pi_async != NULL )       pinfoClose(fsm->pi_async);
  if ( fsm->cl_async != NULL )       dclDestroy(fsm->cl_async);

  free(fsm);
}

int fsm_SetName(fsm_type fsm, const char *name)
{
  return internal_fsm_set_str(&(fsm->name), name);
}

void fsm_LogVA(fsm_type fsm, char *fmt, va_list va)
{
  fsm->log_fn[fsm->log_stack_top](fsm->log_data[fsm->log_stack_top], 
    fsm->log_default_level[fsm->log_stack_top], fmt, va);
}

void fsm_Log(fsm_type fsm, char *fmt, ...)
{
  va_list va;
  va_start(va, fmt);
  fsm->log_fn[fsm->log_stack_top](fsm->log_data[fsm->log_stack_top], 
    fsm->log_default_level[fsm->log_stack_top], fmt, va);
  va_end(va);
}

void fsm_LogLev(fsm_type fsm, int ll, char *fmt, ...)
{
  va_list va;
  va_start(va, fmt);
  fsm->log_fn[fsm->log_stack_top](fsm->log_data[fsm->log_stack_top], 
    ll, fmt, va);
  va_end(va);
}

/* contains triples char*, pinfo *, dclist */
void fsm_LogDCL(fsm_type fsm, char *prefix, int cnt, ...)
{
  va_list va;
  int line = 0;
  int line_cnt;
  int width[10];
  char buf[1024*2];
  int i;
  char *s;
  pinfo *pi;
  dclist cl;
  line_cnt = 0;
  va_start(va, cnt);
  for( i = 0; i < cnt; i++ )
  {
    s = va_arg(va, char *);
    pi = va_arg(va, pinfo *);
    cl = va_arg(va, dclist);
    width[i] = strlen(s)+1;
    if ( width[i] < pi->in_cnt+pi->out_cnt+3 )
      width[i] = pi->in_cnt+pi->out_cnt+3;
    if ( line_cnt < dclCnt(cl) )
      line_cnt = dclCnt(cl);
  }
  va_end(va);
  
  if ( line_cnt > fsm->log_dcl_max )
  {
    fsm_Log(fsm, "FSM: Logging of DCL suppressed (%d > %d).", line_cnt, fsm->log_dcl_max);
    return;
  }

  va_start(va, cnt);
  buf[0] = '\0';
  for( i = 0; i < cnt; i++ )
  {
    s = va_arg(va, char *);
    pi = va_arg(va, pinfo *);
    cl = va_arg(va, dclist);
    sprintf(buf+strlen(buf), "%*s", -width[i], s);
  }
  va_end(va);
  fsm_Log(fsm, "%s%s", prefix, buf);

  for( line = 0; line < line_cnt; line++ )
  {
    va_start(va, cnt);
    buf[0] = '\0';
    for( i = 0; i < cnt; i++ )
    {
      s = va_arg(va, char *);
      pi = va_arg(va, pinfo *);
      cl = va_arg(va, dclist);
      if ( line < dclCnt(cl))
        sprintf(buf+strlen(buf), "%*s", width[i], dcToStr(pi, dclGet(cl, line), " ", "  "));
      else
        sprintf(buf+strlen(buf), "%*s", width[i], "");
    }
    va_end(va);
    fsm_Log(fsm, "%s%s", prefix, buf);
  }
}

/* contains triples char*, pinfo *, dclist */
void fsm_LogDCLLev(fsm_type fsm, int ll, char *prefix, int cnt, ...)
{
  va_list va;
  int line = 0;
  int line_cnt;
  int width[10];
  char buf[1024*2];
  int i;
  char *s;
  pinfo *pi;
  dclist cl;
  line_cnt = 0;
  va_start(va, cnt);
  for( i = 0; i < cnt; i++ )
  {
    s = va_arg(va, char *);
    pi = va_arg(va, pinfo *);
    cl = va_arg(va, dclist);
    width[i] = strlen(s)+1;
    if ( width[i] < pi->in_cnt+pi->out_cnt+3 )
      width[i] = pi->in_cnt+pi->out_cnt+3;
    if ( line_cnt < dclCnt(cl) )
      line_cnt = dclCnt(cl);
  }
  va_end(va);
  
  if ( line_cnt > fsm->log_dcl_max )
  {
    fsm_LogLev(fsm, ll, "FSM: Logging of DCL suppressed (%d > %d).", line_cnt, fsm->log_dcl_max);
    return;
  }

  va_start(va, cnt);
  buf[0] = '\0';
  for( i = 0; i < cnt; i++ )
  {
    s = va_arg(va, char *);
    pi = va_arg(va, pinfo *);
    cl = va_arg(va, dclist);
    sprintf(buf+strlen(buf), "%*s", -width[i], s);
  }
  va_end(va);
  fsm_LogLev(fsm, 0, "%s%s", prefix, buf);

  for( line = 0; line < line_cnt; line++ )
  {
    va_start(va, cnt);
    buf[0] = '\0';
    for( i = 0; i < cnt; i++ )
    {
      s = va_arg(va, char *);
      pi = va_arg(va, pinfo *);
      cl = va_arg(va, dclist);
      if ( line < dclCnt(cl))
        sprintf(buf+strlen(buf), "%*s", width[i], dcToStr(pi, dclGet(cl, line), " ", "  "));
      else
        sprintf(buf+strlen(buf), "%*s", width[i], "");
    }
    va_end(va);
    fsm_LogLev(fsm, ll, "%s%s", prefix, buf);
  }
}

void fsm_PushLogFn(fsm_type fsm, void (*fn)(void *, int, char *, va_list), void *data, int ll)
{
  fsm->log_stack_top++;
  fsm->log_fn[fsm->log_stack_top] = fn;
  fsm->log_data[fsm->log_stack_top] = data;
  fsm->log_default_level[fsm->log_stack_top] = ll;
}

void fsm_PopLogFn(fsm_type fsm)
{
  if ( fsm->log_stack_top <= 0 )
    return;
  fsm->log_stack_top--;
}

/*---------------------------------------------------------------------------*/

int fsm_LoopNodeOutEdges(fsm_type fsm, int node_id, int *loop_ptr, int *edge_id_ptr)
{
  if ( b_il_WhileLoop(fsm_GetNode(fsm,node_id)->out_edges, loop_ptr) == 0 )
    return 0;
  *edge_id_ptr = b_il_GetVal(fsm_GetNode(fsm,node_id)->out_edges, *loop_ptr);
  return 1;
}

int fsm_LoopNodeInEdges(fsm_type fsm, int node_id, int *loop_ptr, int *edge_id_ptr)
{
  if ( b_il_WhileLoop(fsm_GetNode(fsm,node_id)->in_edges, loop_ptr) == 0 )
    return 0;
  *edge_id_ptr = b_il_GetVal(fsm_GetNode(fsm,node_id)->in_edges, *loop_ptr);
  return 1;
}

int fsm_LoopNodeOutNodes(fsm_type fsm, int node_id, int *loop_ptr, int *node_id_ptr)
{
  int edge_id;
  if ( fsm_LoopNodeOutEdges(fsm, node_id, loop_ptr, &edge_id) == 0 )
    return 0;
  *node_id_ptr = fsm_GetEdgeDestNode(fsm, edge_id);
  return 1;
}

int fsm_LoopNodeInNodes(fsm_type fsm, int node_id, int *loop_ptr, int *node_id_ptr)
{
  int edge_id;
  if ( fsm_LoopNodeInEdges(fsm, node_id, loop_ptr, &edge_id) == 0 )
    return 0;
  *node_id_ptr = fsm_GetEdgeSrcNode(fsm, edge_id);
  return 1;
}


/*---------------------------------------------------------------------------*/

/* returns node pos or -1 */
int fsm_AddEmptyNode(fsm_type fsm)
{
  fsmnode_type n;
  int pos;
  n = fsmnode_Open(fsm);
  if ( n != NULL )
  {
    pos = b_set_Add(fsm->nodes, n);
    if ( pos >= 0 )
    {
      return pos;
    }
    fsmnode_Close(n);
  }
  fsm_Log(fsm, "FSM memory error: Can not create node.");
  return -1;
}

/* returns edge pos or -1 */
int fsm_AddEmptyEdge(fsm_type fsm)
{
  fsmedge_type e;
  int pos;
  e = fsmedge_Open(fsm);
  if ( e != NULL )
  {
    pos = b_set_Add(fsm->edges, e);
    if ( pos >= 0 )
    {
      return pos;
    }
    fsmedge_Close(e);
  }
  fsm_Log(fsm, "FSM memory error: Can not create edge.");
  return -1;
}

/*---------------------------------------------------------------------------*/

/* returns edge id or -1 */
static int fsm_find_edge(fsm_type fsm, int source_node_id, int dest_node_id)
{
  fsmnode_type n;
  fsmedge_type e;
  int i;
  if ( source_node_id >= 0 )
  {
    n = fsm_GetNode(fsm, source_node_id);
    for(i = 0; i < b_il_GetCnt(n->out_edges); i++ )
    {
      e = fsm_GetEdge(fsm, b_il_GetVal(n->out_edges, i));
      assert(e != NULL);
      assert(e->source_node == source_node_id);
      if ( e->dest_node == dest_node_id )
        return b_il_GetVal(n->out_edges, i);
    }
    return -1;
  }

  if ( dest_node_id >= 0 )
  {
    n = fsm_GetNode(fsm, dest_node_id);
    for(i = 0; i < b_il_GetCnt(n->in_edges); i++ )
    {
      e = fsm_GetEdge(fsm, b_il_GetVal(n->in_edges, i));
      assert(e != NULL);
      assert(e->dest_node == dest_node_id);
      if ( e->source_node == source_node_id )
        return b_il_GetVal(n->in_edges, i);
    }
    return -1;
  }
  
  return -1;
}

/* assumes an unconnected edge */
static int fsm_connect_edge(fsm_type fsm, int pos, int source_node_id, int dest_node_id)
{
  fsmedge_type e = fsm_GetEdge(fsm, pos);
  fsmnode_type n;
  assert(e != NULL);
  assert(e->source_node < 0);
  assert(e->dest_node < 0);

  if ( source_node_id >= 0 )  
  {
    n = fsm_GetNode(fsm, source_node_id);
    assert(b_il_Find(n->out_edges, pos) < 0);
  }
  if ( source_node_id < 0 || b_il_Add(n->out_edges, pos) >= 0 )
  {
    if ( dest_node_id >= 0 )
    {
      n = fsm_GetNode(fsm, dest_node_id);
      assert(b_il_Find(n->in_edges, pos) < 0);
    }
    if ( dest_node_id < 0 || b_il_Add(n->in_edges, pos) >= 0 )
    {
      e->source_node = source_node_id;
      e->dest_node = dest_node_id;
      return 1;
    }
    n = fsm_GetNode(fsm, source_node_id);
    b_il_DelByVal(n->out_edges, pos);
  }
  return 0;
}


static void fsm_disconnect_edge(fsm_type fsm, int pos)
{
  fsmedge_type e = fsm_GetEdge(fsm, pos);
  fsmnode_type n;
  
  if ( e->source_node >= 0 )
  {
    n = fsm_GetNode(fsm, e->source_node);
    assert(b_il_Find(n->out_edges, pos) >= 0);
    b_il_DelByVal(n->out_edges, pos);
    e->source_node = -1;
  }
  
  if ( e->dest_node >= 0 )
  {
    n = fsm_GetNode(fsm, e->dest_node);
    assert(b_il_Find(n->in_edges, pos) >= 0);
    b_il_DelByVal(n->in_edges, pos);
    e->dest_node = -1;
  }
}

/*---------------------------------------------------------------------------*/

/* returns edge pos or -1 */
int fsm_FindEdge(fsm_type fsm, int source_node_id, int dest_node_id)
{
  return fsm_find_edge(fsm, source_node_id, dest_node_id);
}

/* connect existing edge */
int fsm_ConnectEdge(fsm_type fsm, int edge_id, 
  int source_node_id, int dest_node_id)
{
  fsm_disconnect_edge(fsm, edge_id);
  return fsm_connect_edge(fsm, edge_id, source_node_id, dest_node_id);
}

void fsm_DeleteEdge(fsm_type fsm, int edge_id)
{
  fsm_disconnect_edge(fsm, edge_id);
  fsmedge_Close(fsm_GetEdge(fsm, edge_id));
  b_set_Del(fsm->edges, edge_id);
}

/*---------------------------------------------------------------------------*/

int fsm_Reconnect(fsm_type fsm, 
  int old_source_node_id, int old_dest_node_id, 
  int source_node_id, int dest_node_id)
{
  int edge_id;
  assert( fsm_FindEdge(fsm, source_node_id, dest_node_id) < 0 );
  edge_id = fsm_FindEdge(fsm, old_source_node_id, old_dest_node_id);
  if ( edge_id < 0 )
    return -1;
  return fsm_ConnectEdge(fsm, edge_id, source_node_id, dest_node_id);
}

int fsm_Connect(fsm_type fsm, int source_node_id, int dest_node_id)
{
  int edge_id;
  edge_id = fsm_FindEdge(fsm, source_node_id, dest_node_id);
  if ( edge_id >= 0 )
    return edge_id;
  edge_id = fsm_AddEmptyEdge(fsm);
  if ( edge_id >= 0 )
    if ( fsm_connect_edge(fsm, edge_id, source_node_id, dest_node_id) == 0 )
      return fsm_DeleteEdge(fsm, edge_id), -1;
  return edge_id;
}

void fsm_Disconnect(fsm_type fsm, int source_node_id, int dest_node_id)
{
  int edge_id;
  edge_id = fsm_FindEdge(fsm, source_node_id, dest_node_id);
  if ( edge_id < 0 )
    return;
  fsm_DeleteEdge(fsm, edge_id);
}

/* Copy the contents from src_edge to dest_edge */
/* At the moment, only the dclist is copied. */
/* This does not change the connection of nodes! */
int fsm_CopyEdge(fsm_type fsm, int dest_edge_id, int src_edge_id)
{
  if ( dclCopy(fsm_GetConditionPINFO(fsm), 
    fsm_GetEdgeCondition(fsm,dest_edge_id), 
    fsm_GetEdgeCondition(fsm,src_edge_id)) == 0 )
    return 0;
  if ( dclCopy(fsm_GetOutputPINFO(fsm), 
    fsm_GetEdgeOutput(fsm,dest_edge_id), 
    fsm_GetEdgeOutput(fsm,src_edge_id)) == 0 )
    return 0;
  return 1;
}

int fsm_ConnectAndCopy(fsm_type fsm, int source_node_id, int dest_node_id, int edge_id)
{
  int new_edge_id;
  new_edge_id = fsm_Connect(fsm, source_node_id, dest_node_id);
  if ( new_edge_id < 0 )
    return -1;
  if ( fsm_CopyEdge(fsm, new_edge_id, edge_id) == 0 )
  {
    fsm_DeleteEdge(fsm, new_edge_id);
    return -1;
  }
  return new_edge_id;
}

/*---------------------------------------------------------------------------*/

/* returns position or -1 */
int fsm_AddNodeOutputCube(fsm_type fsm, int node_id, dcube *c)
{
  return dclAdd(fsm_GetOutputPINFO(fsm), fsm_GetNodeOutput(fsm, node_id), c);
}

/* returns position or -1 */
int fsm_AddEdgeOutputCube(fsm_type fsm, int edge_id, dcube *c)
{
  return dclAdd(fsm_GetOutputPINFO(fsm), fsm_GetEdgeOutput(fsm, edge_id), c);
}

/* returns position or -1 */
int fsm_AddEdgeConditionCube(fsm_type fsm, int edge_id, dcube *c)
{
  return dclAdd(fsm_GetConditionPINFO(fsm), fsm_GetEdgeCondition(fsm, edge_id), c);
}

/*---------------------------------------------------------------------------*/

int fsm_SetNodeOutputStr(fsm_type fsm, int node_id, char *s)
{
  return fsmnode_SetOutput( fsm_GetNode(fsm, node_id), s);
}

int fsm_SetEdgeOutputStr(fsm_type fsm, int edge_id, char *s)
{
  return fsmedge_SetOutput( fsm_GetEdge(fsm, edge_id), s);
}

int fsm_SetEdgeConditionStr(fsm_type fsm, int edge_id, char *s)
{
  return fsmedge_SetCond( fsm_GetEdge(fsm, edge_id), s);
}

/*---------------------------------------------------------------------------*/

void fsm_DeleteNodeGroup(fsm_type fsm, int node_id)
{
  fsmnode_type n;
  n = fsm_GetNode(fsm, node_id);
  
  if ( n->group_index < 0 )
    return;     /* nothing to do */
  
  assert( n->group_index < b_set_Max(fsm->groups) );
  assert( fsm_GetGroup(fsm,n->group_index) != NULL );
  assert( fsm_GetGroup(fsm,n->group_index)->group_index == n->group_index );
  b_il_DelByVal(fsm_GetGroup(fsm,n->group_index)->node_group_list, node_id);
  n->group_index = -1;
}

int fsm_SetNodeGroup(fsm_type fsm, int node_id, int group_index)
{
  fsmgrp_type grp;
  fsm_DeleteNodeGroup(fsm, node_id);

  if ( group_index < 0 || 
      group_index >= b_set_Max(fsm->groups) || 
      fsm_GetGroup(fsm,group_index) == NULL )
  {
    grp = fsmgrp_Open(NULL, group_index);
    if ( grp == NULL )
      return 0;
    if ( b_set_Set(fsm->groups, group_index, grp) == 0 )
      return 0;
    fsm_LogLev(fsm, 0, "FSM: Add group, index %d.", group_index);
  }
  else
  {
    grp = fsm_GetGroup(fsm, group_index);
  }
  
  if ( b_il_AddUnique(grp->node_group_list, node_id) < 0 )
    return 0;
  
  fsm_GetNode(fsm, node_id)->group_index = group_index;
  fsm_LogLev(fsm, 0, "FSM: Group-index %d for state '%s'.", 
    group_index, fsm_GetNodeNameStr(fsm, node_id));
  
  assert( fsm_GetNode(fsm, node_id)->group_index < b_set_Max(fsm->groups) );
  return 1;
}

/*---------------------------------------------------------------------------*/

void fsm_DeleteNode(fsm_type fsm, int node_id)
{
  fsmnode_type n;
  n = fsm_GetNode(fsm, node_id);

  fsm_DeleteNodeGroup(fsm, node_id);
  
  while(b_il_GetCnt(n->out_edges) > 0)
    fsm_DeleteEdge(fsm, b_il_GetVal(n->out_edges, 0));

  while(b_il_GetCnt(n->in_edges) > 0)
    fsm_DeleteEdge(fsm, b_il_GetVal(n->in_edges, 0));
    
  fsmnode_Close(fsm_GetNode(fsm, node_id));
  b_set_Del(fsm->nodes, node_id);
  
}

/*---------------------------------------------------------------------------*/

char *fsm_GetNodeNameStr(fsm_type fsm, int node_id)
{
  static char s[64];
  if ( node_id < 0 )
  {
    sprintf(s, "<illegal %d>", node_id);
    return s;
  }
  if ( fsm_GetNodeName(fsm, node_id) != NULL )
    return fsm_GetNodeName(fsm, node_id);
  sprintf(s, "<node%d>", node_id);
  return s;
}

int fsm_GetNodeIdByName(fsm_type fsm, const char *name)
{
  int i;
  i = -1;
  while( b_set_WhileLoop(fsm->nodes, &i) != 0 )
    if ( strcmp( fsm_GetNode(fsm, i)->name, name ) == 0 )
      return i;
  return -1;
}

/* returns node_id or -1 */
int fsm_AddNodeByName(fsm_type fsm, char *name)
{
  int node_id = fsm_GetNodeIdByName(fsm, name);
  if ( node_id >= 0 )
    return node_id;
  node_id = fsm_AddEmptyNode(fsm);
  if ( node_id < 0 )
    return -1;
  if ( fsmnode_SetName(fsm_GetNode(fsm, node_id), name) == 0 )
    return fsm_DeleteNode(fsm, node_id), -1;
  return node_id;
}

/* returns node_id or -1 */
int fsm_AddNewNodeByName(fsm_type fsm, char *name)
{
  int node_id;
  node_id = fsm_AddEmptyNode(fsm);
  if ( node_id < 0 )
    return -1;
  if ( fsmnode_SetName(fsm_GetNode(fsm, node_id), name) == 0 )
    return fsm_DeleteNode(fsm, node_id), -1;
  return node_id;
}

/* returns edge_id or -1 */
/* is_create != 0: creates new nodes if required */
int fsm_ConnectByName(fsm_type fsm, char *source_node, char *dest_node, int is_create)
{
  int source_node_id;
  int dest_node_id;

  if ( is_create != 0 )
  {
    source_node_id = fsm_AddNodeByName(fsm, source_node);
    dest_node_id = fsm_AddNodeByName(fsm, dest_node);
  }
  else
  {
    source_node_id = fsm_GetNodeIdByName(fsm, source_node);
    dest_node_id = fsm_GetNodeIdByName(fsm, dest_node);
  }

  if ( source_node_id >= 0 && dest_node_id >= 0 )
    return fsm_Connect(fsm, source_node_id, dest_node_id);
  return -1;
}

int fsm_FindEdgeByName(fsm_type fsm, char *source_node, char *dest_node)
{
  int source_node_id;
  int dest_node_id;

  source_node_id = fsm_GetNodeIdByName(fsm, source_node);
  dest_node_id = fsm_GetNodeIdByName(fsm, dest_node);

  if ( source_node_id < 0 || dest_node_id < 0 )
    return -1;

  return fsm_FindEdge(fsm, source_node_id, dest_node_id);
}

void fsm_DisconnectByName(fsm_type fsm, char *source_node, char *dest_node)
{
  int source_node_id;
  int dest_node_id;

  source_node_id = fsm_GetNodeIdByName(fsm, source_node);
  dest_node_id = fsm_GetNodeIdByName(fsm, dest_node);

  if ( source_node_id >= 0 && dest_node_id >= 0 )
    fsm_Disconnect(fsm, source_node_id, dest_node_id);
}

void fsm_DeleteNodeByName(fsm_type fsm, char *name)
{
  int node_id = fsm_GetNodeIdByName(fsm, name);
  if ( node_id >= 0 )
    fsm_DeleteNode(fsm, node_id);
}

/* create a copy of a reference node */
/* returns a new node_id or -1 */
int fsm_DupNode(fsm_type fsm, int node_id)
{
  int new_node_id;
  int loop;
  int edge_id;
  
  new_node_id = fsm_AddNewNodeByName(fsm, fsm_GetNodeName(fsm, node_id));
  if ( new_node_id < 0 )
    return -1;
  
  while( fsm_LoopNodeOutEdges(fsm, node_id, &loop, &edge_id) == 0 )
  {
    if ( fsm_GetEdgeSrcNode(fsm,edge_id) != fsm_GetEdgeDestNode(fsm,edge_id) )
      if ( fsm_ConnectAndCopy(fsm, 
        new_node_id, fsm_GetEdgeDestNode(fsm, edge_id), edge_id) < 0 )
      {
        fsm_DeleteNode(fsm, new_node_id);
        return -1;
      }
  }

  while( fsm_LoopNodeInEdges(fsm, node_id, &loop, &edge_id) == 0 )
  {
    if ( fsm_GetEdgeSrcNode(fsm,edge_id) != fsm_GetEdgeDestNode(fsm,edge_id) )
      if ( fsm_ConnectAndCopy(fsm, 
        fsm_GetEdgeSrcNode(fsm, edge_id), new_node_id, edge_id) < 0 )
      {
        fsm_DeleteNode(fsm, new_node_id);
        return -1;
      }
  }
  
  edge_id = fsm_FindEdge(fsm, node_id, node_id);
  if ( fsm_ConnectAndCopy(fsm, new_node_id, new_node_id, edge_id) < 0 )
    return -1;
  
  return new_node_id;
}


/*---------------------------------------------------------------------------*/

int fsm_ExpandUniversalNode(fsm_type fsm, int node_id)
{
  int loop_id;
  int edge_id;
  int curr_edge_id;
  int curr_node_id;
  
  loop_id = -1;
  edge_id = -1;
  while(fsm_LoopNodeInEdges(fsm, node_id, &loop_id, &edge_id) != 0)
  {
    curr_node_id = -1;
    while( fsm_LoopNodes(fsm, &curr_node_id) != 0 )
    {
      if ( curr_node_id != node_id && curr_node_id != fsm_GetEdgeSrcNode(fsm, edge_id) )
      {
        curr_edge_id = fsm_Connect(fsm, fsm_GetEdgeSrcNode(fsm, edge_id), curr_node_id);
        if ( curr_edge_id >= 0 )
        {
          if ( dclJoin(fsm_GetConditionPINFO(fsm), 
            fsm_GetEdgeCondition(fsm,curr_edge_id),
            fsm_GetEdgeCondition(fsm,edge_id)) == 0 )
            return 0;
          if ( dclJoin(fsm_GetOutputPINFO(fsm), 
            fsm_GetEdgeOutput(fsm,curr_edge_id),
            fsm_GetEdgeOutput(fsm,edge_id)) == 0 )
            return 0;
        }
      }
    }
  }
  
  loop_id = -1;
  edge_id = -1;
  while(fsm_LoopNodeOutEdges(fsm, node_id, &loop_id, &edge_id) != 0)
  {
    curr_node_id = -1;
    while( fsm_LoopNodes(fsm, &curr_node_id) != 0 )
    {
      if ( curr_node_id != node_id && curr_node_id != fsm_GetEdgeDestNode(fsm, edge_id) )
      {
        curr_edge_id = fsm_Connect(fsm, curr_node_id, fsm_GetEdgeDestNode(fsm, edge_id));
        if ( curr_edge_id >= 0 )
        {
          if ( dclJoin(fsm_GetConditionPINFO(fsm), 
            fsm_GetEdgeCondition(fsm,curr_edge_id),
            fsm_GetEdgeCondition(fsm,edge_id)) == 0 )
            return 0;
          if ( dclJoin(fsm_GetOutputPINFO(fsm), 
            fsm_GetEdgeOutput(fsm,curr_edge_id),
            fsm_GetEdgeOutput(fsm,edge_id)) == 0 )
            return 0;
        }
      }
    }
  }
  
  fsm_DeleteNode(fsm, node_id);
  
  return 1;
}

int fsm_ExpandUniversalNodeByName(fsm_type fsm, char *name)
{
  int node_id = fsm_GetNodeIdByName(fsm, name);
  if ( node_id < 0 )
    return 1;
  return fsm_ExpandUniversalNode(fsm, node_id);
}

/*---------------------------------------------------------------------------*/

void fsm_ShowNodes(fsm_type fsm, FILE *fp)
{
  int i, j;
  fsmnode_type n;
  fsmedge_type e;
  j = -1;
  while( b_set_WhileLoop(fsm->nodes, &j) != 0 )
  {
    n = fsm_GetNode(fsm, j);
    fprintf(fp, "%5d out: %d", j, b_il_GetCnt(n->out_edges));
    if ( b_il_GetCnt(n->out_edges) > 0 )
    {
      fprintf(fp, " (");
      for( i = 0; i < b_il_GetCnt(n->out_edges); i++ )
      {
        e = fsm_GetEdge(fsm, b_il_GetVal(n->out_edges, i));
        assert( e != NULL );
        if ( i > 0 )
          fprintf(fp, ", ");
        fprintf(fp, "%d", e->dest_node);
      }
      fprintf(fp, ")");
    }
    fprintf(fp, "\n");
    
    fprintf(fp, "%5s  in: %d", "", b_il_GetCnt(n->in_edges));
    if ( b_il_GetCnt(n->in_edges) > 0 )
    {
      fprintf(fp, " (");
      for( i = 0; i < b_il_GetCnt(n->in_edges); i++ )
      {
        e = fsm_GetEdge(fsm, b_il_GetVal(n->in_edges, i));
        assert( e != NULL );
        if ( i > 0 )
          fprintf(fp, ", ");
        fprintf(fp, "%d", e->source_node);
      }
      fprintf(fp, ")");
    }
    fprintf(fp, "\n");
  }
}


void fsm_ShowEdges(fsm_type fsm, FILE *fp)
{
  int i, j;
  i = -1;
  while( fsm_LoopEdges(fsm, &i) != 0 )
  {
    for( j = 0; j < dclCnt(fsm_GetEdgeCondition(fsm, i)); j++ )
    {
      fprintf(fp, "%s ", dcToStr(fsm_GetConditionPINFO(fsm), dclGet(fsm_GetEdgeCondition(fsm, i), j), "", ""));
      fprintf(fp, "%s ", dcToStr(fsm_GetCodePINFO(fsm), fsm_GetNodeCode(fsm,fsm_GetEdgeSrcNode(fsm,i)), "", ""));
      fprintf(fp, " | %s ", dcToStr(fsm_GetCodePINFO(fsm), fsm_GetNodeCode(fsm,fsm_GetEdgeDestNode(fsm,i)), "", ""));
      fprintf(fp, "%s -> ", fsm_GetNodeName(fsm, fsm_GetEdgeSrcNode(fsm,i)));
      fprintf(fp, "%s", fsm_GetNodeName(fsm, fsm_GetEdgeDestNode(fsm,i)));
    
      fprintf(fp, "\n");
    }
  }
}

/*---------------------------------------------------------------------------*/

void fsm_tree_by_str(fsm_type fsm, dcexn *n, char *s, int type)
{
  if ( *n != NULL )
    dcexnClose(*n);
  *n = NULL;
  if ( s != NULL )
    *n = dcexParser(fsm->dcex, s, type);
}


int fsm_StrToTree(fsm_type fsm)
{
  int pos;
  fsmnode_type n;
  fsmedge_type e;

  dcexClearVariables(fsm->dcex);

  pos = -1;
  while( b_set_WhileLoop(fsm->nodes, &pos) != 0 )
  {
    n = fsm_GetNode(fsm, pos);
    fsm_tree_by_str(fsm, &(n->tree_output), n->str_output, DCEX_TYPE_ASSIGNMENT);
  }
  
  pos = -1;
  while( b_set_WhileLoop(fsm->edges, &pos) != 0 )
  {
    e = fsm_GetEdge(fsm, pos);
    fsm_tree_by_str(fsm, &(e->tree_output), e->str_output, DCEX_TYPE_ASSIGNMENT);
    fsm_tree_by_str(fsm, &(e->tree_cond), e->str_cond, DCEX_TYPE_BOOLE);
  }
  
  b_sl_Copy(fsm->input_sl, fsm->dcex->in_variables);
  b_sl_Copy(fsm->output_sl, fsm->dcex->out_variables);
  
  return 1;
}

/*---------------------------------------------------------------------------*/

void fsm_cubes_by_tree(fsm_type fsm, pinfo *pi, dclist cl, dcexn n, int type)
{
  if ( n == NULL )
    return;
  dcexEval(fsm->dcex, pi, cl, n, type);
}

int fsm_TreeToCube(fsm_type fsm)
{
  int pos;
  fsmnode_type n;
  fsmedge_type e;

  pos = -1;
  while( b_set_WhileLoop(fsm->nodes, &pos) != 0 )
  {
    n = fsm_GetNode(fsm, pos);
    dclClear(n->cl_output);
  }

  pos = -1;
  while( b_set_WhileLoop(fsm->edges, &pos) != 0 )
  {
    e = fsm_GetEdge(fsm, pos);
    dclClear(e->cl_output);
    dclClear(e->cl_cond);
  }
  
  pinfoSetInCnt(fsm->pi_cond, b_sl_GetCnt(fsm->input_sl));
  pinfoSetOutCnt(fsm->pi_cond, 0);
  
  pinfoSetInCnt(fsm->pi_output, b_sl_GetCnt(fsm->input_sl));
  pinfoSetOutCnt(fsm->pi_output, b_sl_GetCnt(fsm->output_sl));

  pos = -1;
  while( b_set_WhileLoop(fsm->nodes, &pos) != 0 )
  {
    n = fsm_GetNode(fsm, pos);
    fsm_cubes_by_tree(fsm, fsm->pi_output, n->cl_output, n->tree_output, DCEX_TYPE_ASSIGNMENT);
  }
  
  pos = -1;
  while( b_set_WhileLoop(fsm->edges, &pos) != 0 )
  {
    e = fsm_GetEdge(fsm, pos);
    fsm_cubes_by_tree(fsm, fsm->pi_output, e->cl_output, e->tree_output, DCEX_TYPE_ASSIGNMENT);
    fsm_cubes_by_tree(fsm, fsm->pi_output, e->cl_cond, e->tree_cond, DCEX_TYPE_BOOLE);
  }
  return 1;
}

/*---------------------------------------------------------------------------*/

void fsm_Show(fsm_type fsm)
{
  int pos;
  int loop;
  int dest_node_id;
  fsmnode_type n;
  fsmedge_type e;
  
  pos = -1;
  while( fsm_LoopNodes(fsm, &pos) != 0 )
  {
    n = fsm_GetNode(fsm, pos);
    printf("Node %d\n", pos);
    printf("Output '%s': ", n->str_output==NULL?"(nul)":n->str_output);
    dcexShow(fsm->dcex, n->tree_output);
    printf("\n");
    dclShow(fsm->pi_output, n->cl_output);

      
    printf("out nodes: ");
    loop = -1;
    while( fsm_LoopNodeOutNodes(fsm, pos, &loop, &dest_node_id) != 0 )
    {
      printf("%s ", fsm_GetNodeName(fsm, dest_node_id));
    }
    printf("\n");
  }

  pos = -1;
  while( fsm_LoopEdges(fsm, &pos) != 0 )
  {
    e = fsm_GetEdge(fsm, pos);
    printf("Edge %d (%d -> %d)\n", pos, e->source_node, e->dest_node);
    printf("Output '%s': ", e->str_output==NULL?"(nul)":e->str_output);
    dcexShow(fsm->dcex, e->tree_output);
    printf("\n");
    dclShow(fsm->pi_output, e->cl_output);
    printf("Condition '%s': ", e->str_cond==NULL?"(nul)":e->str_cond);
    dcexShow(fsm->dcex, e->tree_cond);
    printf("\n");
    dclShow(fsm->pi_cond, e->cl_cond);
  }
}


/*---------------------------------------------------------------------------*/

int fsm_SetInCnt(fsm_type fsm, int in_cnt)
{
  int edge;
  fsm->in_cnt = in_cnt;
  if ( pinfoSetInCnt(fsm->pi_cond, in_cnt) == 0 )
    return 0;
  if ( pinfoSetInCnt(fsm->pi_output, in_cnt) == 0 )
    return 0;
  if ( pinfoSetInCnt(fsm->pi_machine, fsm->code_width+fsm->pi_cond->in_cnt) == 0 )
    return 0;
  edge = -1;
  while(fsm_LoopEdges(fsm, &edge) != 0)
  {
    dclRealClear(fsm_GetEdgeOutput(fsm, edge));
    dclRealClear(fsm_GetEdgeCondition(fsm, edge));
  }
  dclRealClear(fsm->cl_machine);
  dclRealClear(fsm->cl_machine_dc);
  return 1;
}

int fsm_SetOutCnt(fsm_type fsm, int out_cnt)
{
  int edge, node;
  fsm->out_cnt = out_cnt;
  if ( pinfoSetOutCnt(fsm->pi_output, out_cnt+1) == 0 )
    return 0;
  if ( pinfoSetOutCnt(fsm->pi_machine, fsm->code_width+out_cnt) == 0 )
    return 0;
  node = -1;
  while(fsm_LoopNodes(fsm, &node) != 0)
  {
    dclRealClear(fsm_GetNodeOutput(fsm, node));
  }
  edge = -1;
  while(fsm_LoopEdges(fsm, &edge) != 0)
  {
    dclRealClear(fsm_GetEdgeOutput(fsm, edge));
    dclRealClear(fsm_GetEdgeCondition(fsm, edge));
  }
  dclRealClear(fsm->cl_machine);
  dclRealClear(fsm->cl_machine_dc);
  return 1;
}

int fsm_SetCodeWidth(fsm_type fsm, int code_width, int code_option)
{
  int node;

  if ( pinfoSetInCnt(fsm_GetCodePINFO(fsm), code_width) == 0 )
    return 0;
  if ( pinfoSetOutCnt(fsm_GetCodePINFO(fsm), code_width) == 0 )
    return 0;
  switch(code_option)
  {
    case FSM_CODE_DFF_INCLUDED_OUTPUT:
      if ( pinfoSetOutCnt(fsm->pi_machine, code_width) == 0 )
        return 0;
      break;
    case FSM_CODE_DFF_EXTRA_OUTPUT:
    default:
      if ( pinfoSetOutCnt(fsm->pi_machine, code_width+fsm->out_cnt) == 0 )
        return 0;
      break;
  }
  if ( pinfoSetInCnt(fsm->pi_machine, code_width+fsm->in_cnt) == 0 )
    return 0;
  
  fsm->code_width = code_width;
  node = -1;
  while(fsm_LoopNodes(fsm, &node) != 0)
  {
    if (dcAdjustByPinfo(fsm_GetCodePINFO(fsm), fsm_GetNodeCode(fsm,node))==0)
      return 0;
  }
    
  dclRealClear(fsm->cl_machine);
  dclRealClear(fsm->cl_machine_dc);
  return 1;
}

dcube *fsm_GetResetCode(fsm_type fsm)
{
  if ( fsm->reset_node_id < 0 )
    return NULL;
  return fsm_GetNodeCode(fsm, fsm->reset_node_id);
}

void fsm_SetResetNodeId(fsm_type fsm, int node_id)
{
  fsm->reset_node_id = node_id;
}

int fsm_SetResetNodeIdByName(fsm_type fsm, const char *name)
{
  int node_id = fsm_GetNodeIdByName(fsm, name);
  if ( node_id < 0 )
    return 0;
  fsm_SetResetNodeId(fsm, node_id);
  return 1;
}

/*---------------------------------------------------------------------------*/

int fsm_GetEncodeStateCnt(fsm_type fsm)
{
  int n, g;
  int cnt = 0;
  g = -1;
  while( fsm_LoopGroups(fsm, &g) != 0 )
    if ( b_il_GetCnt(fsm_GetGroup(fsm, g)->node_group_list) > 0 )
      cnt++;
  n = -1;
  while( fsm_LoopNodes(fsm, &n) != 0 )
    if ( fsm_GetNode(fsm, n)->group_index < 0 )
      cnt++;
  return cnt;
}

int fsm_EncodeSimple(fsm_type fsm)
{
  int state_cnt = b_set_Cnt(fsm->nodes);
  int log_2_state_cnt;
  pinfo *pi = fsm_GetCodePINFO(fsm);
  dcube *c;
  int node_id;
  int reset_node_id = fsm->reset_node_id;
  int group_id;
  int reset_group_id = -1;
  int loop;
 

  if ( reset_node_id >= 0 )
    reset_group_id = fsm_GetNode(fsm, reset_node_id)->group_index;
  
  /* state_cnt = b_set_Cnt(fsm->nodes); */
  state_cnt = fsm_GetEncodeStateCnt(fsm);
  if ( state_cnt <= 0 )
    return 1;
  
  state_cnt--;
  log_2_state_cnt = 0;
  while( state_cnt != 0 )
  {
    state_cnt /= 2;
    log_2_state_cnt++;
  }
  
  if ( fsm_SetCodeWidth(fsm, log_2_state_cnt, FSM_CODE_DFF_EXTRA_OUTPUT) == 0 )
    return 0;
  
  if ( log_2_state_cnt == 0 )	/* 2019: added */
    return 1;
    
  c = &(pi->tmp[9]);
  dcOutSetAll(pi, c, 0);

  node_id = -1;
  while( fsm_LoopNodes(fsm, &node_id) != 0 )
  {
    dcInSetAll(fsm_GetCodePINFO(fsm), 
      fsm_GetNodeCode(fsm, node_id), CUBE_IN_MASK_DC );
    dcOutSetAll(fsm_GetCodePINFO(fsm), 
      fsm_GetNodeCode(fsm, node_id), 0 );
  }
  
  if ( fsm->reset_node_id >= 0 )
  {
    if ( reset_group_id >= 0 )
    {
      loop = -1;
      node_id = -1;
      while( fsm_LoopGroupNodes(fsm, reset_group_id, &loop, &node_id) != 0 )
      {
        dcCopy(pi, fsm_GetNodeCode(fsm, fsm->reset_node_id), c);
        dcCopyOutToIn(pi, fsm_GetNodeCode(fsm, fsm->reset_node_id), 0, pi, fsm_GetNodeCode(fsm, fsm->reset_node_id));
        fsm_Log(fsm, "FSM: State '%s' encoded with %s (reset, group %d).", 
          fsm_GetNodeNameStr(fsm, node_id), dcToStr(pi, fsm_GetNodeCode(fsm, fsm->reset_node_id), "/", ""),
          reset_group_id);
      }
    }
    else
    {
      dcCopy(pi, fsm_GetNodeCode(fsm, fsm->reset_node_id), c);
      dcCopyOutToIn(pi, fsm_GetNodeCode(fsm, fsm->reset_node_id), 0, pi, fsm_GetNodeCode(fsm, fsm->reset_node_id));
    }
    dcIncOut(pi, c);
  }
  
  group_id = -1;
  while( fsm_LoopGroups(fsm, &group_id) != 0 )
  {
    if ( reset_group_id != group_id )
    {
      loop = -1;
      node_id = -1;
      while( fsm_LoopGroupNodes(fsm, group_id, &loop, &node_id) != 0 )
      {
        dcCopy(pi, fsm_GetNodeCode(fsm, node_id), c);
        dcCopyOutToIn(pi, fsm_GetNodeCode(fsm, node_id), 0, pi, fsm_GetNodeCode(fsm, node_id));
        fsm_Log(fsm, "FSM: State '%s' encoded with %s (group %d).", 
          fsm_GetNodeNameStr(fsm, node_id), dcToStr(pi, fsm_GetNodeCode(fsm, node_id), "/", ""), group_id);
      }
      dcIncOut(pi, c);
    }
  }

  
  node_id = -1;
  while( fsm_LoopNodes(fsm, &node_id) != 0 )
  {
    if ( node_id != fsm->reset_node_id && fsm_GetNode(fsm, node_id)->group_index < 0 )
    {
      dcCopy(pi, fsm_GetNodeCode(fsm, node_id), c);
      dcCopyOutToIn(pi, fsm_GetNodeCode(fsm, node_id), 0, pi, fsm_GetNodeCode(fsm, node_id));
      fsm_Log(fsm, "FSM: State '%s' encoded with %s (normal).", 
        fsm_GetNodeNameStr(fsm, node_id), dcToStr(pi, fsm_GetNodeCode(fsm, node_id), "/", ""));
      dcIncOut(pi, c);
    }
  }
  
  return 1;
}

int fsm_EncodeRandom(fsm_type fsm, int seed)
{
  int state_cnt = b_set_Cnt(fsm->nodes);
  int log_2_state_cnt;
  pinfo *pi = fsm_GetCodePINFO(fsm);
  dcube *c;
  int node_id;
  dclist cl;
  int pos;
 
  if ( state_cnt <= 0 )
    return 1;
  
  state_cnt--;
  log_2_state_cnt = 0;
  while( state_cnt != 0 )
  {
    state_cnt /= 2;
    log_2_state_cnt++;
  }
  
  if ( fsm_SetCodeWidth(fsm, log_2_state_cnt, FSM_CODE_DFF_EXTRA_OUTPUT) == 0 )
    return 0;
    
  c = &(pi->tmp[9]);
  dcOutSetAll(pi, c, 0);
  
  if ( dclInit(&cl) == 0 )
    return 0;

  if ( fsm->reset_node_id >= 0 )
    dcIncOut(pi, c);
  for(;;)
  {
    if ( dclAdd(pi, cl, c) < 0 )
      return dclDestroy(cl), 0;
    if ( dcIncOut(pi, c) == 0 )
      break;
  }

  
  if ( fsm->reset_node_id >= 0 )
  {
    dcOutSetAll(pi, c, 0);
    dcCopy(pi, fsm_GetNodeCode(fsm, fsm->reset_node_id), c);
    dcIncOut(pi, c);
  }
  
  node_id = -1;
  srand(seed);
  while( fsm_LoopNodes(fsm, &node_id) != 0 )
  {
    if ( node_id != fsm->reset_node_id )
    {
      pos = rand() % dclCnt(cl);
      dcCopy(pi, fsm_GetNodeCode(fsm, node_id), dclGet(cl, pos));
      dclDeleteCube(pi, cl, pos);
    }
  }
  
  return dclDestroy(cl), 1;
}


/*---------------------------------------------------------------------------*/
const char *fsm_GetInLabel(fsm_type fsm, int pos)
{
  char *l = NULL;
  if ( pos < 0 ) 
    return NULL;
  if ( pos >= fsm_GetInCnt(fsm) )
    return NULL;
  if ( pos < b_sl_GetCnt(fsm->input_sl) )
    l = b_sl_GetVal(fsm->input_sl, pos);
  if ( l == NULL )
  {
    l = fsm->labelbuf;
    sprintf(l, "i%d", pos);
  }
  return l;
}

const char *fsm_GetOutLabel(fsm_type fsm, int pos, const char *post)
{
  char *l = NULL;
  if ( post == NULL )
    post = "";
  if ( pos < 0 ) 
    return NULL;
  if ( pos >= fsm_GetOutCnt(fsm) )
    return NULL;
  if ( pos < b_sl_GetCnt(fsm->output_sl) )
  {
    l = b_sl_GetVal(fsm->output_sl, pos);
    sprintf(fsm->labelbuf, "%s%s", l, post);
    l = fsm->labelbuf;
  }
  if ( l == NULL )
  {
    l = fsm->labelbuf;
    sprintf(l, "o%d%s", pos, post);
  }
  return l;
}

const char *fsm_GetStateLabel(fsm_type fsm, int pos, const char *prefix)
{
  if ( strlen(prefix) > 32 )		/* labelbuf has 64 bytes */
    return NULL;
  if ( pos < 0 ) 
    return NULL;
  if ( pos >= fsm_GetCodeWidth(fsm) )
    return NULL;
  sprintf(fsm->labelbuf, "%s%d", prefix, pos);
  return fsm->labelbuf;
}

/*---------------------------------------------------------------------------*/
/*
  copies all known labels into pi_machine.
*/
int fsm_AssignLabelsToMachine(fsm_type fsm, int code_option)
{
  int i, j;
  b_sl_type in_sl = pinfoGetInLabelList(fsm->pi_machine);
  b_sl_type out_sl = pinfoGetOutLabelList(fsm->pi_machine);
  int state_label_code_width = fsm_GetCodeWidth(fsm);
  int in_out_label_code_width = 0;
  
  if ( in_sl == NULL || out_sl == NULL )
    return 0;  
    
  if ( code_option == FSM_CODE_DFF_INCLUDED_OUTPUT )
  {
    state_label_code_width = state_label_code_width - fsm_GetOutCnt(fsm);
    in_out_label_code_width = fsm_GetOutCnt(fsm);
  }

  
  b_sl_Clear(in_sl);
  j = 0;

  for( i = 0; i < fsm_GetInCnt(fsm); i++ )
    if ( b_sl_Set(in_sl, j++, fsm_GetInLabel(fsm, i)) == 0 )
      return 0;

  for( i = 0; i < state_label_code_width; i++ )
    if ( b_sl_Set(in_sl, j++, fsm_GetStateLabel(fsm, i, fsm_state_in_signal)) == 0 )
      return 0;

  for( i = 0; i < in_out_label_code_width; i++ )
    if ( b_sl_Set(in_sl, j++, fsm_GetOutLabel(fsm, i, "_fb")) == 0 )
      return 0;

  b_sl_Clear(out_sl);
  j = 0;

  for( i = 0; i < state_label_code_width; i++ )
    if ( b_sl_Set(out_sl, j++, fsm_GetStateLabel(fsm, i, fsm_state_out_signal)) == 0 )
      return 0;

  for( i = 0; i < fsm_GetOutCnt(fsm); i++ )
    if ( b_sl_Set(out_sl, j++, fsm_GetOutLabel(fsm, i, "")) == 0 )
      return 0;

  return 1;  
}

/*---------------------------------------------------------------------------*/

/*
  In  z^n  | z^n+1  Out
  
  assumes: Valid encoding (e.g. fsm_EncodeSimple or fsm_EncodePartition)
  creates: Transfer function (cl->machine, pi->machine)
  
  should be renamed to Transferfunction???
  
*/
int fsm_BuildMachine(fsm_type fsm)
{
  int node_id;
  int src_node_id;
  int edge_id;
  int loop;
  dcube *state_code;
  dcube *src_state_code;
  dcube *c;
  dclist cl;
  int i, cnt;
  
  dclRealClear(fsm->cl_machine);
  dclRealClear(fsm->cl_machine_dc);
  c = &(fsm->pi_machine->tmp[9]);
  node_id = -1;
  while( fsm_LoopNodes(fsm, &node_id) != 0 )
  {
    loop = -1;
    state_code = fsm_GetNodeCode(fsm, node_id);
    while( fsm_LoopNodeInEdges(fsm, node_id, &loop, &edge_id) != 0 )
    {
      src_node_id = fsm_GetEdgeSrcNode(fsm, edge_id);
      src_state_code = fsm_GetNodeCode(fsm, src_node_id);
      cl = fsm_GetEdgeCondition(fsm, edge_id);
      cnt = dclCnt(cl);
      for( i = 0; i < cnt; i++ )
      {
        dcInSetAll(fsm->pi_machine, c, CUBE_IN_MASK_DC);
        dcOutSetAll(fsm->pi_machine, c, 0);
        
        dcCopyInToIn(  fsm->pi_machine, c, 0,                    fsm->pi_cond, dclGet(cl, i));
        dcCopyOutToIn( fsm->pi_machine, c, fsm->pi_cond->in_cnt, fsm->pi_code, src_state_code);
        dcCopyOutToOut(fsm->pi_machine, c, 0,                    fsm->pi_code, state_code);
        
        if ( dcIsIllegal(fsm->pi_machine, c) == 0 )
          if ( dclAdd(fsm->pi_machine, fsm->cl_machine, c) < 0 )
            return 0;
      }
      if ( fsm->is_with_output != 0 )
      {
        cl = fsm_GetEdgeOutput(fsm, edge_id);
        cnt = dclCnt(cl);
        for( i = 0; i < cnt; i++ )
        {
          dcInSetAll(fsm->pi_machine, c, CUBE_IN_MASK_DC);
          dcOutSetAll(fsm->pi_machine, c, 0);

          dcCopyInToIn(  fsm->pi_machine, c, 0,                     fsm->pi_output, dclGet(cl, i));
          dcCopyOutToIn( fsm->pi_machine, c, fsm->pi_cond->in_cnt,  fsm->pi_code,   src_state_code);
          dcCopyOutToOut(fsm->pi_machine, c, fsm->pi_code->out_cnt, fsm->pi_output, dclGet(cl, i));

          if ( dcIsIllegal(fsm->pi_machine, c) == 0 )
            if ( dclAdd(fsm->pi_machine, fsm->cl_machine, c) < 0 )
              return 0;
        }
      }
    }
  }

  return fsm_AssignLabelsToMachine(fsm, FSM_CODE_DFF_EXTRA_OUTPUT);
}

/*
  differs a little bit from the function above:
    - ignores the 'fsm->is_with_output' flag
    - creates a DC set
*/
int fsm_BuildTransferfunction(fsm_type fsm)
{
  int node_id;
  int src_node_id;
  int edge_id;
  int loop;
  dcube *state_code;
  dcube *src_state_code;
  dcube *c;
  dclist cl;
  dclist cl_off;
  int i, cnt;
  
  if ( dclInit(&cl_off) == 0 )
    return 0;
  
  dclRealClear(fsm->cl_machine);
  dclRealClear(fsm->cl_machine_dc);
  c = &(fsm->pi_machine->tmp[9]);
  node_id = -1;
  while( fsm_LoopNodes(fsm, &node_id) != 0 )
  {
    loop = -1;
    state_code = fsm_GetNodeCode(fsm, node_id);
    while( fsm_LoopNodeInEdges(fsm, node_id, &loop, &edge_id) != 0 )
    {
      src_node_id = fsm_GetEdgeSrcNode(fsm, edge_id);
      src_state_code = fsm_GetNodeCode(fsm, src_node_id);
      cl = fsm_GetEdgeOutput(fsm, edge_id);
      cnt = dclCnt(cl);
      for( i = 0; i < cnt; i++ )
      {
        dcInSetAll(fsm->pi_machine, c, CUBE_IN_MASK_DC);
        dcOutSetAll(fsm->pi_machine, c, 0);
        
        dcCopyInToIn(  fsm->pi_machine, c, 0,                     fsm->pi_cond,   dclGet(cl, i));
        dcCopyOutToIn( fsm->pi_machine, c, fsm->pi_cond->in_cnt,  fsm->pi_code,   src_state_code);
        dcCopyOutToOut(fsm->pi_machine, c, 0,                     fsm->pi_code,   state_code);
        dcCopyOutToOut(fsm->pi_machine, c, fsm->pi_code->out_cnt, fsm->pi_output, dclGet(cl, i));
        
        if ( dcIsIllegal(fsm->pi_machine, c) == 0 )
          if ( dclAdd(fsm->pi_machine, fsm->cl_machine, c) < 0 )
            return dclDestroy(cl_off), 0;
            
        dcInvOut(fsm->pi_machine, c);
        if ( dcIsIllegal(fsm->pi_machine, c) == 0 )
          if ( dclAdd(fsm->pi_machine, cl_off, c) < 0 )
            return dclDestroy(cl_off), 0;
      }
    }
  }
  
  dclRealClear(fsm->cl_machine_dc);
  if ( dclAdd(fsm->pi_machine, fsm->cl_machine_dc, &(fsm->pi_machine->tmp[0])) < 0 )
    return dclDestroy(cl_off), 0;
  if ( dclSubtract(fsm->pi_machine, fsm->cl_machine_dc, fsm->cl_machine) == 0 )
    return dclDestroy(cl_off), 0;
  if ( dclSubtract(fsm->pi_machine, fsm->cl_machine_dc, cl_off) == 0 )
    return dclDestroy(cl_off), 0;
  
  dclDestroy(cl_off);

  return fsm_AssignLabelsToMachine(fsm, FSM_CODE_DFF_EXTRA_OUTPUT);
}

/*
  In  z^n  | z^n+1  Out
  
  assumes: Valid encoding (e.g. fsm_EncodeSimple or fsm_EncodePartition)
  creates: Transfer function (cl->machine, pi->machine) for Toggle FFs
  
*/
int fsm_BuildMachineToggleFF(fsm_type fsm)
{
  int node_id;
  int src_node_id;
  int edge_id;
  int loop;
  dcube *state_code;
  dcube *src_state_code;
  dcube *c;
  dcube *code_tmp;
  dclist cl;
  int i, cnt;
  
  dclRealClear(fsm->cl_machine);
  dclRealClear(fsm->cl_machine_dc);
  c = &(fsm->pi_machine->tmp[9]);
  code_tmp = &(fsm->pi_code->tmp[9]);
  node_id = -1;
  while( fsm_LoopNodes(fsm, &node_id) != 0 )
  {
    loop = -1;
    state_code = fsm_GetNodeCode(fsm, node_id);
    while( fsm_LoopNodeInEdges(fsm, node_id, &loop, &edge_id) != 0 )
    {
      src_node_id = fsm_GetEdgeSrcNode(fsm, edge_id);
      src_state_code = fsm_GetNodeCode(fsm, src_node_id);
      cl = fsm_GetEdgeCondition(fsm, edge_id);
      cnt = dclCnt(cl);
      for( i = 0; i < cnt; i++ )
      {
        dcInSetAll(fsm->pi_machine, c, CUBE_IN_MASK_DC);
        dcOutSetAll(fsm->pi_machine, c, 0);
        
        dcCopyInToIn(  fsm->pi_machine, c, 0,                    fsm->pi_cond, dclGet(cl, i));
        dcCopyOutToIn( fsm->pi_machine, c, fsm->pi_cond->in_cnt, fsm->pi_code, src_state_code);
        
        dcXorOut(fsm->pi_code, code_tmp, src_state_code, state_code);
        dcCopyOutToOut(fsm->pi_machine, c, 0,                    fsm->pi_code, code_tmp);
        
        if ( dcIsIllegal(fsm->pi_machine, c) == 0 )
          if ( dclAdd(fsm->pi_machine, fsm->cl_machine, c) < 0 )
            return 0;
      }
      if ( fsm->is_with_output != 0 )
      {
        cl = fsm_GetEdgeOutput(fsm, edge_id);
        cnt = dclCnt(cl);
        for( i = 0; i < cnt; i++ )
        {
          dcInSetAll(fsm->pi_machine, c, CUBE_IN_MASK_DC);
          dcOutSetAll(fsm->pi_machine, c, 0);

          dcCopyInToIn(  fsm->pi_machine, c, 0,                     fsm->pi_output, dclGet(cl, i));
          dcCopyOutToIn( fsm->pi_machine, c, fsm->pi_cond->in_cnt,  fsm->pi_code,   src_state_code);
          dcCopyOutToOut(fsm->pi_machine, c, fsm->pi_code->out_cnt, fsm->pi_output, dclGet(cl, i));

          if ( dcIsIllegal(fsm->pi_machine, c) == 0 )
            if ( dclAdd(fsm->pi_machine, fsm->cl_machine, c) < 0 )
              return 0;
        }
      }
    }
  }
  return 1;
}

  
/*---------------------------------------------------------------------------*/


/* source node of both edges are the same */
void fsm_ErrorOutEdgeIntersection(fsm_type fsm, int e1, int e2)
{
  fsm_Log(fsm, "FSM Check: Outgoing transitions [%s %s] and [%s %s] intersect.",
    fsm_GetNodeName(fsm, fsm_GetEdgeSrcNode(fsm, e1)),
    fsm_GetNodeName(fsm, fsm_GetEdgeDestNode(fsm, e1)),
    fsm_GetNodeName(fsm, fsm_GetEdgeSrcNode(fsm, e2)),
    fsm_GetNodeName(fsm, fsm_GetEdgeDestNode(fsm, e2))
    );
}

void fsm_ErrorInputCoverLargerThanOutputCover(fsm_type fsm, int node_id)
{
  fsm_Log(fsm, "FSM Check: The input cover of node %s is larger than output cover.",
    fsm_GetNodeName(fsm, node_id));
}

void fsm_WarningNoStableState(fsm_type fsm, int node_id, dclist cl)
{
  fsm_Log(fsm, "FSM Check: Node %s is not stable for some inputs.",
    fsm_GetNodeName(fsm, node_id));
}




/*---------------------------------------------------------------------------*/

int fsm_GetNodeInCover(fsm_type fsm, int node_id, dclist cl)
{
  int loop;
  int edge_id;

  dclClear(cl);
  
  loop = -1;
  edge_id = -1;
  while( fsm_LoopNodeInEdges(fsm, node_id, &loop, &edge_id) != 0 )
  {
    if ( dclSCCUnion(fsm_GetConditionPINFO(fsm), cl, fsm_GetEdgeCondition(fsm, edge_id)) == 0 )
      return 0;
  }
  return 1;
}

int fsm_GetNodeOutCover(fsm_type fsm, int node_id, dclist cl)
{
  int loop;
  int edge_id;

  dclClear(cl);
  
  loop = -1;
  edge_id = -1;
  while( fsm_LoopNodeOutEdges(fsm, node_id, &loop, &edge_id) != 0 )
  {
    if ( dclJoin(fsm_GetConditionPINFO(fsm), cl, fsm_GetEdgeCondition(fsm, edge_id)) == 0 )
      return 0;
  }
  return 1;
}

int fsm_GetNodeInOCover(fsm_type fsm, int node_id, dclist cl)
{
  int loop;
  int edge_id;

  dclClear(cl);
  
  loop = -1;
  edge_id = -1;
  while( fsm_LoopNodeInEdges(fsm, node_id, &loop, &edge_id) != 0 )
  {
    if ( dclSCCUnion(fsm_GetOutputPINFO(fsm), cl, fsm_GetEdgeOutput(fsm, edge_id)) == 0 )
      return 0;
  }
  return 1;
}


/*---------------------------------------------------------------------------*/

int fsm_GetNodePreCover(fsm_type fsm, int node_id, dclist cl)
{
  int loop;
  int edge_id;

  dclClear(cl);
  
  loop = -1;
  edge_id = -1;
  while( fsm_LoopNodeInEdges(fsm, node_id, &loop, &edge_id) != 0 )
    if ( node_id != fsm_GetEdgeSrcNode(fsm, edge_id) )
      if ( dclSCCUnion(fsm_GetConditionPINFO(fsm), cl, fsm_GetEdgeCondition(fsm, edge_id)) == 0 )
        return 0;
  return 1;
}

int fsm_GetNodePostCover(fsm_type fsm, int node_id, dclist cl)
{
  int loop;
  int edge_id;

  dclClear(cl);
  
  loop = -1;
  edge_id = -1;
  while( fsm_LoopNodeOutEdges(fsm, node_id, &loop, &edge_id) != 0 )
    if ( node_id != fsm_GetEdgeDestNode(fsm, edge_id) )
      if ( dclJoin(fsm_GetConditionPINFO(fsm), cl, fsm_GetEdgeCondition(fsm, edge_id)) == 0 )
        return 0;
  return 1;
}

int fsm_GetNodePreOCover(fsm_type fsm, int node_id, dclist cl)
{
  int loop;
  int edge_id;

  dclClear(cl);
  
  loop = -1;
  edge_id = -1;
  while( fsm_LoopNodeInEdges(fsm, node_id, &loop, &edge_id) != 0 )
    if ( node_id != fsm_GetEdgeSrcNode(fsm, edge_id) )
      if ( dclSCCUnion(fsm_GetOutputPINFO(fsm), cl, fsm_GetEdgeOutput(fsm, edge_id)) == 0 )
        return 0;
  return 1;
}

int fsm_GetNodePostOCover(fsm_type fsm, int node_id, dclist cl)
{
  int loop;
  int edge_id;

  dclClear(cl);
  
  loop = -1;
  edge_id = -1;
  while( fsm_LoopNodeOutEdges(fsm, node_id, &loop, &edge_id) != 0 )
    if ( node_id != fsm_GetEdgeDestNode(fsm, edge_id) )
      if ( dclJoin(fsm_GetOutputPINFO(fsm), cl, fsm_GetEdgeOutput(fsm, edge_id)) == 0 )
        return 0;
  return 1;
}


dclist fsm_GetNodeSelfCondtion(fsm_type fsm, int node_id)
{
  int edge_id = fsm_FindEdge(fsm, node_id, node_id);
  if ( edge_id < 0 )
    return NULL;
  return fsm_GetEdgeCondition(fsm, edge_id);
}

int fsm_GetNodePreOutput(fsm_type fsm, int node_id, dclist cl)
{
  int loop;
  int edge_id;

  dclClear(cl);
  
  loop = -1;
  edge_id = -1;
  while( fsm_LoopNodeInEdges(fsm, node_id, &loop, &edge_id) != 0 )
    if ( node_id != fsm_GetEdgeSrcNode(fsm, edge_id) )
      if ( dclSCCUnion(fsm_GetOutputPINFO(fsm), cl, fsm_GetEdgeOutput(fsm, edge_id)) == 0 )
        return 0;
  return 1;
}

int fsm_GetNodePostOutput(fsm_type fsm, int node_id, dclist cl)
{
  int loop;
  int edge_id;

  dclClear(cl);
  
  loop = -1;
  edge_id = -1;
  while( fsm_LoopNodeOutEdges(fsm, node_id, &loop, &edge_id) != 0 )
    if ( node_id != fsm_GetEdgeDestNode(fsm, edge_id) )
      if ( dclJoin(fsm_GetOutputPINFO(fsm), cl, fsm_GetEdgeOutput(fsm, edge_id)) == 0 )
        return 0;
  return 1;
}

dclist fsm_GetNodeSelfOutput(fsm_type fsm, int node_id)
{
  int edge_id = fsm_FindEdge(fsm, node_id, node_id);
  if ( edge_id < 0 )
    return NULL;
  return fsm_GetEdgeOutput(fsm, edge_id);
}

/*---------------------------------------------------------------------------*/

/* should I better use fsm_GetNextStateCode()??? */
/*
  If the FSM is in state 'node_id' there is target state 'dest_node_id' that
  will be reached under condition 'c'.
  'c' is a cube of fsm_GetConditionPINFO(fsm).
  'c' MUST be a minterm!!!
  'dest_code' is a cube of fsm_GetCodePINFO(fsm).
*/
void fsm_GetNextNodeCode(fsm_type fsm, int node_id, dcube *c, int *dest_node_id, dcube *dest_code)
{
  int loop;
  int edge_id;
  int n;
  dcube *tmp = &(fsm->pi_machine->tmp[9]);
  
  /* there are two ways to calculate the target code: cl_machine and the graph */
  /* we do use the graph first, because cl_machine might not yet exist. */
  
  loop = -1;
  edge_id = -1;
  while( fsm_LoopNodeOutEdges(fsm, node_id, &loop, &edge_id) != 0 )
  {
    if ( dclIsSingleSubSet(fsm_GetConditionPINFO(fsm), fsm_GetEdgeCondition(fsm, edge_id), c) != 0 )
    {
      /* great, transition found */
      n = fsm_GetEdgeDestNode(fsm, edge_id);
      if ( dest_node_id != NULL )
        *dest_node_id = n;
      if ( dest_code != NULL )
        dcCopy(fsm_GetCodePINFO(fsm), dest_code, fsm_GetNodeCode(fsm, n));
      return;
    }
  }
  
  /* There was no valid state, so check the machine function */
  
  dcCopyInToIn(fsm->pi_machine, tmp, 0,                 
    fsm_GetConditionPINFO(fsm), c);
  dcCopyInToIn(fsm->pi_machine, tmp, fsm_GetInCnt(fsm), 
    fsm_GetCodePINFO(fsm),      fsm_GetNodeCode(fsm, node_id));

  dclResult(fsm->pi_machine, tmp, fsm->cl_machine);
  
  if ( dest_node_id != NULL )
    *dest_node_id = -1;
  if ( dest_code != NULL )
  {
    dcCopyOutToIn(fsm_GetCodePINFO(fsm), dest_code, 0, fsm->pi_machine, tmp);
    dcCopyOutToOut(fsm_GetCodePINFO(fsm), dest_code, 0, fsm->pi_machine, tmp);
  }
  
}

/*---------------------------------------------------------------------------*/

int fsm_is_empty_intersection(fsm_type fsm, int e1, int e2)
{
  dclist cl1, cl2;
  int i1, i2;

  cl1 = fsm_GetEdgeCondition(fsm, e1);
  cl2 = fsm_GetEdgeCondition(fsm, e2);
  for( i1 = 0; i1 < dclCnt(cl1); i1++ )
    for( i2 = 0; i2 < dclCnt(cl2); i2++ )
      if ( dcIsDeltaInNoneZero(fsm->pi_cond, dclGet(cl1, i1), dclGet(cl2, i2)) == 0 )
        return 0;
  return 1;
}

/* if invalid and is_valid != NULL: *is_valid is set to 0 */
void fsm_CheckNodeOutputIntersection(fsm_type fsm, int node_id, int *is_valid)
{
  int l1, l2;
  int e1, e2;
  
  l1 = -1;
  while( fsm_LoopNodeOutEdges(fsm, node_id, &l1, &e1) != 0 )
  {
    l2 = l1;
    while( fsm_LoopNodeOutEdges(fsm, node_id, &l2, &e2) != 0 )
    {
      if ( fsm_is_empty_intersection(fsm, e1, e2) == 0 )
      {
        fsm_ErrorOutEdgeIntersection(fsm, e1, e2);
        if ( is_valid != NULL )
          *is_valid = 0;
      }
    }
  }
}


/* if invalid and is_valid != NULL: *is_valid is set to 0 */
int fsm_CheckNodeInOutRelation(fsm_type fsm, int node_id, int *is_valid)
{
  dclist cl_in;
  dclist cl_out;
  dclist tmp;
  int edge_id;
  if ( dclInitVA(3, &cl_in, &cl_out, &tmp) == 0 )
    return 0;
  if ( fsm_GetNodeInCover(fsm, node_id, cl_in) == 0 )
    return dclDestroyVA(3, cl_in, cl_out, tmp), 0;
  if ( fsm_GetNodeOutCover(fsm, node_id, cl_out) == 0 )
    return dclDestroyVA(3, cl_in, cl_out, tmp), 0;
  
  if ( dclCopy(fsm_GetConditionPINFO(fsm), tmp, cl_in) == 0 )
    return dclDestroyVA(3, cl_in, cl_out, tmp), 0;
  if( dclSubtract(fsm->pi_cond, tmp, cl_out) == 0 )
    return dclDestroyVA(3, cl_in, cl_out, tmp), 0;
  if ( dclCnt(tmp) != 0 )
  {
    if( is_valid != NULL )
      *is_valid = 0;
    fsm_ErrorInputCoverLargerThanOutputCover(fsm,node_id);
  }
  edge_id = fsm_FindEdge(fsm, node_id, node_id);
  if ( edge_id >= 0  )
  {
    if ( dclSubtract(fsm->pi_cond, cl_out, fsm_GetEdgeCondition(fsm, edge_id)) == 0 )
      return dclDestroyVA(3, cl_in, cl_out, tmp), 0;
    if ( dclSubtract(fsm->pi_cond, cl_in, fsm_GetEdgeCondition(fsm, edge_id)) == 0 )
      return dclDestroyVA(3, cl_in, cl_out, tmp), 0;
  }
  if ( dclIntersectionList(fsm->pi_cond, tmp, cl_in, cl_out) == 0 )
    return dclDestroyVA(3, cl_in, cl_out, tmp), 0;
  if ( dclCnt(tmp) != 0 )
  {
    if( is_valid != NULL )
      *is_valid = 0;
    fsm_WarningNoStableState(fsm,node_id,tmp);
  }
  return dclDestroyVA(3, cl_in, cl_out, tmp), 1;
}


/* if invalid and is_valid != NULL: *is_valid is set to 0 */
int fsm_CheckNode(fsm_type fsm, int node_id, int *is_valid)
{
  fsm_CheckNodeOutputIntersection(fsm, node_id, is_valid);
  if ( fsm_CheckNodeInOutRelation(fsm, node_id, is_valid) == 0 )
    return 0;
  return 1;
}

int fsm_Check(fsm_type fsm, int *is_valid)
{
  int node_id;
  node_id = -1;
  while( fsm_LoopNodes(fsm, &node_id) != 0 )
  {
    if ( fsm_CheckNode(fsm, node_id, is_valid) == 0 )
      return 0;
  }
  return 1;
}

int fsm_IsValid(fsm_type fsm)
{
  int is_valid = 1;
  if ( fsm_Check(fsm, &is_valid) == 0 )
    return 0;
  return is_valid;
}

/*---------------------------------------------------------------------------*/

static int fsm_write_node_el(FILE *fp, void *el, void *ud)
{
  return fsmnode_Write((fsmnode_type)el, fp);
}

static int fsm_write_edge_el(FILE *fp, void *el, void *ud)
{
  return fsmedge_Write((fsmedge_type)el, fp);
}

#define FSM_CHECK 0x0f2111
#define FSM_VERSION 0x01
int fsm_Write(fsm_type fsm, FILE *fp)
{
  if ( b_io_WriteInt(fp, FSM_CHECK) == 0 )                         return 0;
  if ( b_io_WriteInt(fp, FSM_VERSION) == 0 )                       return 0;
  if ( b_set_Write(fsm->nodes, fp, fsm_write_node_el, NULL) == 0 ) return 0;
  if ( b_set_Write(fsm->edges, fp, fsm_write_edge_el, NULL) == 0 ) return 0;
  if ( b_sl_Write(fsm->input_sl, fp) == 0 )                        return 0;
  if ( b_sl_Write(fsm->output_sl, fp) == 0 )                       return 0;
  if ( b_io_WriteInt(fp, fsm->reset_node_id) == 0 )                return 0;
  return 1;
}

int fsm_WriteBin(fsm_type fsm, char *filename)
{
  FILE *fp;
  fp = fopen(filename, "wb");
  if ( fp == NULL )
    return 0;
  if ( fsm_Write(fsm, fp) == 0 )
    return fclose(fp), 0;
  return fclose(fp), 1;
}

static void *fsm_read_node_el(FILE *fp, void *ud)
{
  fsmnode_type n = fsmnode_Open((fsm_type)ud);
  if ( n == NULL )
    return NULL;
  if ( fsmnode_Read(n, fp) == 0 )
    return fsmnode_Close(n), (void *)NULL;
  return n;
}

static void *fsm_read_edge_el(FILE *fp, void *ud)
{
  fsmedge_type e = fsmedge_Open((fsm_type)ud);
  if ( e == NULL )
    return NULL;
  if ( fsmedge_Read(e, fp) == 0 )
    return fsmedge_Close(e), (void *)NULL;
  return e;
}

int fsm_Read(fsm_type fsm, FILE *fp)
{
  int check;
  int version;
  if ( b_io_ReadInt(fp, &check) == 0 )                           return 0;
  if ( b_io_ReadInt(fp, &version) == 0 )                         return 0;
  if ( b_set_Read(fsm->nodes, fp, fsm_read_node_el, fsm) == 0 )  return 0;
  if ( b_set_Read(fsm->edges, fp, fsm_read_edge_el, fsm) == 0 )  return 0;
  if ( b_sl_Read(fsm->input_sl, fp) == 0 )                       return 0;
  if ( b_sl_Read(fsm->output_sl, fp) == 0 )                      return 0;
  if ( b_io_ReadInt(fp, &fsm->reset_node_id) == 0 )              return 0;
  return 1;
}

int fsm_ReadBin(fsm_type fsm, char *filename)
{
  FILE *fp;
  fsm_Clear(fsm);
  fp = fopen(filename, "rb");
  if ( fp == NULL )
    return 0;
  if ( fsm_Read(fsm, fp) == 0 )
    return fclose(fp), 0;
  return fclose(fp), 1;
}

/*---------------------------------------------------------------------------*/

int IsValidFSMFile(const char *filename)
{
  if ( IsValidKISSFile(filename) )
    return 1;
  if ( IsValidBMSFile(filename) )
    return 1;
  if ( fsm_IsValidQFSMFile(filename) )
    return 1;
  return 0;
}

int fsm_Import(fsm_type fsm, const char *filename)
{
  if ( IsValidKISSFile(filename) )
  {
    fsm_Log(fsm, "IMPORT: '%s' (KISS file).", filename);
    return fsm_ReadKISS(fsm, filename);
  }

  if ( IsValidBMSFile(filename) )
  {
    fsm_Log(fsm, "IMPORT: '%s' (BMS file).", filename);
    return fsm_ReadBMS(fsm, filename);
  }

  if ( fsm_IsValidQFSMFile(filename) )
  {
    fsm_Log(fsm, "IMPORT: '%s' (QFSM file).", filename);
    return fsm_ReadQFSM(fsm, filename);
  }
    
  fsm_Log(fsm, "IMPORT: Unknown file format ('%s').", filename);
  return 0;
}


/*---------------------------------------------------------------------------*/

int fsm_WriteStateEncodingFP(fsm_type fsm, FILE *fp)
{
  int node;
  fprintf(fp, "# state encoding\n");
  fprintf(fp, "%d\n", fsm_GetCodeWidth(fsm));
  node = -1;
  while( fsm_LoopNodes(fsm, &node) != 0 )
  {

    fprintf(fp, "%s:%s\n", 
      dcToStr( fsm->pi_code, fsm_GetNodeCode(fsm, node), "", ""),
      fsm_GetNodeName(fsm,node)==NULL?"":fsm_GetNodeName(fsm,node)
      );
  }
  return 1;
}

int fsm_WriteStateEncodingFile(fsm_type fsm, char *filename)
{
  FILE *fp;
  fp = fopen(filename, "w");
  if ( fp == NULL )
    return 0;
  if ( fsm_WriteStateEncodingFP(fsm, fp) == 0 )
    return fclose(fp), 0;
  return fclose(fp), 1;
}


int fsm_EncodeByFP(fsm_type fsm, FILE *fp)
{
  char *l = fsm->linebuf;
  char *s;
  int node;
  int width;
  int cnt;
  
  for(;;)
  {
    s = fgets(l, FSM_LINE_LEN, fp);
    if ( s == NULL )
      return 0;
    if ( isdigit((int)(unsigned char)*s) != 0 )
      break;
  }
  width = atoi(s);
  
  if ( fsm_SetCodeWidth(fsm, width, FSM_CODE_DFF_EXTRA_OUTPUT) == 0 )
    return 0;

  cnt = 0;
  for(;;)
  {
    s = fgets(l, FSM_LINE_LEN, fp);
    if ( s == NULL )
      break;
    if ( *s == '#' )
      continue;
    if ( *s == '\0' )
      continue;
    if ( s[strlen(s)-1] == '\n' )
      s[strlen(s)-1] = '\0';
    s += strspn(l, "01")+1;
    
    if ( s == l )
      return 0;

    node = fsm_GetNodeIdByName(fsm, s);
    if ( node < 0 )
      return 0;
    cnt++;
    
    if ( dcSetByStr( fsm_GetCodePINFO(fsm), fsm_GetNodeCode(fsm, node), l) == 0 )
      return 0;
  }
  
  if ( cnt != b_set_Cnt( fsm->nodes ) )
    return 0;
  
  return 1;
}


int fsm_EncodeByFile(fsm_type fsm, char *filename)
{
  FILE *fp;
  fp = fopen(filename, "r");
  if ( fp == NULL )
    return 0;
  if ( fsm_EncodeByFP(fsm, fp) == 0 )
    return fclose(fp), 0;
  return fclose(fp), 1;
}


