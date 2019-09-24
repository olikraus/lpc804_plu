/*

  xbm.c

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

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>

#include "xbm.h"


int internal_xbm_set_str(char **s, const char *name)
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

static xbm_grp_type xbm_grp_Open(b_il_type il, int gr_pos)
{
  xbm_grp_type grp;
  grp = (xbm_grp_type)malloc(sizeof(struct _xbm_grp_struct));
  if ( grp != NULL )
  {
    grp->gr_pos = gr_pos;
    grp->st_list = b_il_Open();
    if ( grp->st_list != NULL )
    {
      if ( il != NULL )
      {
        if ( b_il_Copy(grp->st_list, il) != 0 )
        {
          return grp;
        }
      }
      else
      {
        return grp;
      }
      b_il_Close(grp->st_list);
    }
    free(grp);
  }
  return NULL;
}

static void xbm_grp_Clear(xbm_grp_type grp)
{
  b_il_Clear(grp->st_list);
  grp->gr_pos = -1;
}

static void xbm_grp_Close(xbm_grp_type grp)
{
  xbm_grp_Clear(grp);
  b_il_Close(grp->st_list);
  free(grp);
}


/*---------------------------------------------------------------------------*/

static xbm_state_type xbm_state_Open(xbm_type x)
{
  xbm_state_type s;
  s = (xbm_state_type)malloc(sizeof(struct _xbm_state_struct));
  if ( s != NULL )
  {
    s->name = NULL;
    s->gr_pos = -1;
    s->d = 0;
    s->p = -1;
    if ( dclInit(&(s->in_self_cl)) != 0 )
    {
      if ( dcInit(xbm_GetPiOut(x), &(s->out)) != 0 )
      {
        dcOutSetAll(xbm_GetPiOut(x), &(s->out), 0);
        if ( dcInit(xbm_GetPiCode(x), &(s->code)) != 0 )
        {
          dcOutSetAll(xbm_GetPiCode(x), &(s->code), 0);
          return s;
        }
        dcDestroy(&(s->out));
      }
      dclDestroy(s->in_self_cl);
    }
    free(s);
  }
  return NULL;
}

static void xbm_state_Close(xbm_state_type s)
{
  if ( s->name != NULL )
    free(s->name);
  dcDestroy(&(s->out));
  dcDestroy(&(s->code));
  dclDestroy(s->in_self_cl);
  free(s);
}

static int xbm_state_SetName(xbm_state_type s, const char *name)
{
  return internal_xbm_set_str(&(s->name), name);
}

static int xbm_state_adjust(pinfo *pi, xbm_state_type s)
{
  if ( dcAdjustByPinfo(pi, &(s->code)) == 0 )
    return 0;
  dcOutSetAll(pi, &(s->code), 0);
  return 1;
}

/*---------------------------------------------------------------------------*/

static xbm_transition_type xbm_transition_Open(xbm_type x)
{
  xbm_transition_type t;
  t = (xbm_transition_type)malloc(sizeof(struct _xbm_transition_struct));
  if ( t != NULL )
  {
    t->src_state_pos = -1;
    t->dest_state_pos = -1;
    t->visit_cnt = 0;
    t->visit_max = 0;
    if ( dcInitVA( xbm_GetPiCond(x), XBM_TRANS_VA_LIST(t) ) != 0 )
    {
      dcInSetAll(xbm_GetPiCond(x),  &(t->level_cond), 0);
      dcOutSetAll(xbm_GetPiCond(x), &(t->level_cond), 0);
      dcInSetAll(xbm_GetPiCond(x),  &(t->edge_cond), 0);
      dcOutSetAll(xbm_GetPiCond(x), &(t->edge_cond), 0);
      dcInSetAll(xbm_GetPiCond(x),  &(t->start_cond), 0);
      dcOutSetAll(xbm_GetPiCond(x), &(t->start_cond), 0);
      dcInSetAll(xbm_GetPiCond(x),  &(t->end_cond), 0);
      dcOutSetAll(xbm_GetPiCond(x), &(t->end_cond), 0);
      if ( dcInitVA(xbm_GetPiIn(x), 4, &(t->in_start_cond), &(t->in_end_cond), 
                  &(t->in_ddc_start_cond), &(t->in_ddc_end_cond)) != 0 )
      {
        dcInSetAll(xbm_GetPiIn(x),    &(t->in_start_cond), CUBE_IN_MASK_DC);
        dcInSetAll(xbm_GetPiIn(x),    &(t->in_end_cond), CUBE_IN_MASK_DC);
        dcInSetAll(xbm_GetPiIn(x),    &(t->in_ddc_start_cond), CUBE_IN_MASK_DC);
        dcInSetAll(xbm_GetPiIn(x),    &(t->in_ddc_end_cond), CUBE_IN_MASK_DC);
        return t;
      }
      dcDestroyVA( XBM_TRANS_VA_LIST(t) );
    }
    free(t);
  }
  return NULL;
}

void xbm_transition_Close(xbm_transition_type t)
{
  dcDestroyVA( 4, &(t->in_start_cond), &(t->in_end_cond), 
      &(t->in_ddc_start_cond), &(t->in_ddc_end_cond) );
  dcDestroyVA( XBM_TRANS_VA_LIST(t) );
  free(t);
}

int xbm_transition_adjust(xbm_type x, xbm_transition_type t)
{
  return dcAdjustByPinfoVA( xbm_GetPiCond(x), XBM_TRANS_VA_LIST(t) );
}

/*---------------------------------------------------------------------------*/

xbm_var_type xbm_var_Open(xbm_type x)
{
  xbm_var_type  v;
  v = (xbm_var_type)malloc(sizeof(struct _xbm_var_struct));
  if ( v != NULL )
  {
    v->is_level = 0;
    v->is_edge = 0;
    v->is_used = 0;
    v->direction = XBM_DIRECTION_IN;
    v->is_delay_line = 0;
    v->delay = 0;
    v->name = NULL;
    v->name2 = NULL;
    v->reset_value = 0;
    v->index = -1;
    return v;
  }
  return NULL;
}

void xbm_var_Close(xbm_var_type v)
{
  if ( v->name != NULL )
    free(v->name);
  free(v);
}

void xbm_var_SetDirection(xbm_var_type v, int direction)
{
  v->direction = direction;
}

int xbm_var_SetName(xbm_var_type v, const char *name)
{
  char *ext = "";
  if ( internal_xbm_set_str(&(v->name), name) == 0 )
    return 0;
  if ( v->name2 != NULL )
    free(v->name2);
  if ( name != NULL )
  {
    if ( v->is_delay_line )
    {
      if ( v->direction == XBM_DIRECTION_IN )
        ext = "_in";
      else
        ext = "_out";
    }
    v->name2 = (char *)malloc(strlen(name)+strlen(ext)+1);
    if ( v->name2 == NULL )
      return 0;
    strcpy(v->name2, name);
    strcat(v->name2, ext);
  }
  return 1;
}

void xbm_var_SetResetValue(xbm_var_type v, int reset_value)
{
  v->reset_value = reset_value;
}


/*---------------------------------------------------------------------------*/


xbm_type xbm_Open()
{
  xbm_type x;
  x = (xbm_type)malloc(sizeof(struct _xbm_struct));
  if ( x != NULL )
  {
    x->log_level = 4;
    x->is_file_check = 0;
    x->is_auto_ddc_level = 1;
    x->is_auto_ddc_edge = 0;
    x->is_auto_ddc_mix = 0;
    x->pla_file_xbm_partitions = NULL;
    x->pla_file_prime_partitions = NULL;
    x->pla_file_min_partitions = NULL;
    x->import_file_state_codes = NULL;
    x->import_file_pla_machine = NULL;
    x->pi_async = NULL;
    x->cl_async = NULL;
    x->pi_machine = NULL;
    x->cl_machine = NULL;
    x->name = NULL;
    x->reset_state_name = NULL;
    x->reset_st_pos = -1;
    x->inputs = 0;
    x->outputs = 0;
    x->is_fbo = 0;
    x->is_sync = 0;
    x->is_log_cb = 0;
    x->is_all_dc_change = 1;
    x->is_safe_opt = 0;       /* allow unsafe minimization */
    
    
    x->tb_entity_name = NULL;
    x->tb_arch_name = NULL;
    x->tb_conf_name = NULL;
    x->tb_component_name = NULL;
    x->tb_clr_name = NULL;
    x->tb_clk_name = NULL;
    x->tb_wait_time_ns = 100;
    x->tb_reset_type = XBM_RESET_LOW;
    
    x->pi_cond = pinfoOpen();
    if ( x->pi_cond != NULL )
    {
      x->states = b_set_Open();
      if ( x->states != NULL )
      {
        x->transitions = b_set_Open();
        if ( x->transitions != NULL )
        {
          x->vars = b_set_Open();
          if ( x->vars != NULL )
          {
            x->groups = b_set_Open();
            if ( x->groups != NULL )
            {
              x->pi_in = pinfoOpen();
              if ( x->pi_in != NULL )
              {
                x->pi_out = pinfoOpen();
                if ( x->pi_out != NULL )
                {
                  x->pi_code = pinfoOpen();
                  if ( x->pi_code != NULL )
                  {
                    if ( xbm_vhdl_init(x) != 0 )
                    {
                      return x;
                    }
                    xbm_vhdl_destroy(x);
                  }
                  pinfoClose(x->pi_code);
                }
                pinfoClose(x->pi_out);
              }
              b_set_Close(x->groups);
            }
            b_set_Close(x->vars);
          }
          b_set_Close(x->transitions);
        }
        b_set_Close(x->states);
      }
      pinfoClose(x->pi_cond);
    }
    free(x);
  }
  return NULL;
}

void xbm_ClearGr(xbm_type x)
{
  int pos;
  
  pos = -1;
  while( xbm_LoopGr(x, &pos) != 0 )
    xbm_grp_Close(xbm_GetGr(x, pos));
  
  pos = -1;
  while( xbm_LoopSt(x, &pos) != 0 )
    xbm_GetSt(x, pos)->gr_pos = -1;

  b_set_Clear(x->groups);
}

void xbm_Clear(xbm_type x)
{
  int pos;

  xbm_DestroyAsync(x);
  xbm_DestroyMachine(x);

  xbm_ClearGr(x);

  pos = -1;
  while( xbm_LoopVar(x, &pos) != 0 )
    xbm_var_Close(xbm_GetVar(x, pos));
    
  pos = -1;
  while( xbm_LoopSt(x, &pos) != 0 )
    xbm_state_Close(xbm_GetSt(x, pos));

  pos = -1;
  while( xbm_LoopTr(x, &pos) != 0 )
    xbm_transition_Close(xbm_GetTr(x, pos));

  b_set_Clear(x->vars);
  b_set_Clear(x->transitions);
  b_set_Clear(x->states);

  internal_xbm_set_str(&(x->name), NULL);
  internal_xbm_set_str(&(x->reset_state_name), NULL);

  xbm_SetPLAFileNameXBMPartitions(x, NULL);
  xbm_SetPLAFileNamePrimePartitions(x, NULL);
  xbm_SetPLAFileNameMinPartitions(x, NULL);
  xbm_SetImportFileNameStateCodes(x, NULL);
  xbm_SetImportFileNamePLAMachine(x, NULL);

}

void xbm_Close(xbm_type x)
{
  xbm_Clear(x);

  xbm_vhdl_destroy(x);
  
  b_set_Close(x->groups);
  b_set_Close(x->vars);
  b_set_Close(x->transitions);
  b_set_Close(x->states);
  pinfoClose(x->pi_cond);
  pinfoClose(x->pi_in);
  pinfoClose(x->pi_out);
  pinfoClose(x->pi_code);
  free(x);
}

int xbm_SetName(xbm_type x, const char *name)
{
  return internal_xbm_set_str(&(x->name), name);
}

int xbm_SetResetStateName(xbm_type x, const char *name)
{
  return internal_xbm_set_str(&(x->reset_state_name), name);
}

void xbm_SetLogCB(xbm_type x, void (*log_cb)(void *data, int ll, const char *fmt, va_list va), void *data)
{
  x->log_cb = log_cb;
  x->log_data = data;
  x->is_log_cb = 1;
}

void xbm_ClrLogCB(xbm_type x)
{
  x->is_log_cb = 0;
}

void xbm_LogVA(xbm_type x, int ll, const char *fmt, va_list va)
{
  if ( x->is_file_check != 0 )
    return;
  if ( ll >= x->log_level )
  {
    if ( x->is_log_cb != 0 )
    {
      x->log_cb(x->log_data, ll, fmt, va);
    }
    else
    {
      vprintf(fmt, va);
      puts("");
    }
  }
}


void xbm_Log(xbm_type x, int ll, const char *fmt, ...)
{
  va_list va;
  if ( x->is_file_check != 0 )
    return;
  va_start(va, fmt);
  xbm_LogVA(x, ll, fmt, va);
  va_end(va);
}

void xbm_ErrorVA(xbm_type x, const char *fmt, va_list va)
{
  vprintf(fmt, va);
  puts("");
}

void xbm_Error(xbm_type x, const char *fmt, ...)
{
  va_list va;
  if ( x->is_file_check != 0 )
    return;
  va_start(va, fmt);
  xbm_ErrorVA(x, fmt, va);
  va_end(va);
}

int xbm_LoopStOutTr(xbm_type x, int st_pos, int *loop_ptr, int *tr_pos_ptr)
{
  while( xbm_LoopTr(x, loop_ptr) != 0 )
  {
    if ( xbm_GetTrSrcStPos(x, *loop_ptr) == st_pos )
    {
      *tr_pos_ptr = *loop_ptr;
      return 1;
    }
  }
  return 0;
}

int xbm_LoopStInTr(xbm_type x, int st_pos, int *loop_ptr, int *tr_pos_ptr)
{
  while( xbm_LoopTr(x, loop_ptr) != 0 )
  {
    if ( xbm_GetTrDestStPos(x, *loop_ptr) == st_pos )
    {
      *tr_pos_ptr = *loop_ptr;
      return 1;
    }
  }
  return 0;
}


int xbm_LoopStOutSt(xbm_type x, int st_pos, int *loop_ptr, int *st_pos_ptr)
{
  int tr_pos;
  if ( xbm_LoopStOutTr(x, st_pos, loop_ptr, &tr_pos) == 0 )
    return 0;
  *st_pos_ptr = xbm_GetTrDestStPos(x, tr_pos);
  return 1;
}

int xbm_LoopStInSt(xbm_type x, int st_pos, int *loop_ptr, int *st_pos_ptr)
{
  int tr_pos;
  if ( xbm_LoopStInTr(x, st_pos, loop_ptr, &tr_pos) == 0 )
    return 0;
  *st_pos_ptr = xbm_GetTrSrcStPos(x, tr_pos);
  return 1;
}

int xbm_AddEmptySt(xbm_type x)
{
  xbm_state_type s;
  int pos;
  s = xbm_state_Open(x);
  if ( s != NULL )
  {
    pos = b_set_Add(x->states, s);
    if ( pos >= 0 )
      return pos;
    xbm_state_Close(s);
  }
  return -1;
}

int xbm_AddSt(xbm_type x, const char *name)
{
  xbm_state_type s;
  int pos;
  s = xbm_state_Open(x);
  if ( s != NULL )
  {
    if ( xbm_state_SetName(s, name) != 0 )
    {
      pos = b_set_Add(x->states, s);
      if ( pos >= 0 )
        return pos;
    }
    xbm_state_Close(s);
  }
  return -1;
}

void xbm_DeleteStGr(xbm_type x, int st_pos)
{
  xbm_state_type s;
  xbm_grp_type g;
  s = xbm_GetSt(x, st_pos);
  if ( s->gr_pos >= 0 )
  {
    g = xbm_GetGr(x, s->gr_pos);
    b_il_DelByVal(g->st_list, st_pos);
    s->gr_pos = -1;
  }
}


int xbm_SetStGr(xbm_type x, int st_pos, int gr_pos)
{
  xbm_grp_type g;
  xbm_DeleteStGr(x, st_pos);
  if ( gr_pos < 0 || 
      gr_pos >= b_set_Max(x->groups) || 
      xbm_GetGr(x,gr_pos) == NULL )
  {
    g = xbm_grp_Open(NULL, gr_pos);
    if ( g == NULL )
      return 0;
    if ( b_set_Set(x->groups, gr_pos, g) == 0 )
      return 0;
  }
  else
  {
    g = xbm_GetGr(x, gr_pos);
  }
  if ( b_il_AddUnique(g->st_list, st_pos) < 0 )
    return 0;
  xbm_GetSt(x, st_pos)->gr_pos = gr_pos;
  return 1;
}

int xbm_FindStByName(xbm_type x, const char *name)
{
  int pos = -1;
  while( xbm_LoopSt(x, &pos) != 0 )
    if ( strcmp( xbm_GetSt(x, pos)->name, name ) == 0 )
      return pos;
  return -1;
}

int xbm_AddEmptyTr(xbm_type x)
{
  xbm_transition_type t;
  int pos;
  t = xbm_transition_Open(x);
  if ( t != NULL )
  {
    pos = b_set_Add(x->transitions, t);
    if ( pos >= 0 )
      return pos;
    xbm_transition_Close(t);
  }
  return -1;
}

/* returns edge id or -1 */
static int xbm_find_transtion(xbm_type x, int src_state_pos, int dest_state_pos)
{
  int i;
  int tr_pos;
  
  if ( src_state_pos >= 0 )
  {
    i = -1;
    tr_pos = -1;
    while( xbm_LoopStOutTr(x, src_state_pos, &i, &tr_pos) != 0 )
      if ( xbm_GetTrDestStPos(x, tr_pos) == dest_state_pos )
        return tr_pos;
    return -1;
  }

  if ( dest_state_pos >= 0 )
  {
    i = -1;
    tr_pos = -1;
    while( xbm_LoopStInTr(x, src_state_pos, &i, &tr_pos) != 0 )
      if ( xbm_GetTrSrcStPos(x, tr_pos) == src_state_pos )
        return tr_pos;
    return -1;
  }
  
  return -1;
}

/* assumes an unconnected edge */
static int xbm_connect_transition(xbm_type x, int tr_pos, int src_state_pos, int dest_state_pos)
{
  xbm_transition_type t = xbm_GetTr(x, tr_pos);
  //xbm_state_type s = NULL;
  
  assert(t != NULL);
  assert(t->src_state_pos < 0);
  assert(t->dest_state_pos < 0);

  if ( src_state_pos >= 0 && dest_state_pos >= 0 )
  {
    t->src_state_pos = src_state_pos;
    t->dest_state_pos = dest_state_pos;
    return 1;
  }

  return 0;
}

static void xbm_disconnect_transition(xbm_type x, int tr_pos)
{
  xbm_transition_type t = xbm_GetTr(x, tr_pos);
  //xbm_state_type s = NULL;
  
  if ( t->src_state_pos >= 0 )
  {
    /* s = xbm_GetSt(x, t->src_state_pos); */
    /* assert(b_il_Find(s->out_tr_pos, tr_pos) >= 0); */
    /* b_il_DelByVal(s->out_tr_pos, tr_pos); */
    t->src_state_pos = -1;
  }
  
  if ( t->dest_state_pos >= 0 )
  {
    /* s = xbm_GetSt(x, t->dest_state_pos); */
    /* assert(b_il_Find(s->in_tr_pos, tr_pos) >= 0); */
    /* b_il_DelByVal(s->in_tr_pos, tr_pos); */
    t->dest_state_pos = -1;
  }
}

/* returns transition pos or -1 */
int xbm_FindTr(xbm_type x, int src_state_pos, int dest_state_pos)
{
  return xbm_find_transtion(x, src_state_pos, dest_state_pos);
}

int xbm_FindTrByName(xbm_type x, const char *from, const char *to)
{
  int src_state_pos = xbm_FindStByName(x, from);
  int dest_state_pos = xbm_FindStByName(x, to);
  return xbm_FindTr(x, src_state_pos, dest_state_pos);
}

/* connect existing edge */
int xbm_ConnectTr(xbm_type x, int tr_pos, 
  int src_state_pos, int dest_state_pos)
{
  xbm_disconnect_transition(x, tr_pos);
  return xbm_connect_transition(x, tr_pos, src_state_pos, dest_state_pos);
}

void xbm_DeleteTr(xbm_type x, int tr_pos)
{
  xbm_disconnect_transition(x, tr_pos);
  xbm_transition_Close(xbm_GetTr(x, tr_pos));
  b_set_Del(x->transitions, tr_pos);
}

int xbm_Connect(xbm_type x, int src_state_pos, int dest_state_pos)
{
  int tr_pos;
  tr_pos = xbm_FindTr(x, src_state_pos, dest_state_pos);
  if ( tr_pos >= 0 )
    return tr_pos;
  tr_pos = xbm_AddEmptyTr(x);
  if ( tr_pos >= 0 )
    if ( xbm_connect_transition(x, tr_pos, src_state_pos, dest_state_pos) == 0 )
      return xbm_DeleteTr(x, tr_pos), -1;
  return tr_pos;
}

int xbm_ConnectByName(xbm_type x, const char *from, const char *to, int is_create_states)
{
  int src_state_pos = xbm_FindStByName(x, from);
  int dest_state_pos = xbm_FindStByName(x, to);
  
  if ( is_create_states != 0 )
  {
    if ( src_state_pos < 0 )
      src_state_pos = xbm_AddSt(x, from);
    if ( dest_state_pos < 0 )
      dest_state_pos = xbm_AddSt(x, to);
  }
  
  if ( src_state_pos < 0 || dest_state_pos < 0)
    return 0;
  
  return xbm_Connect(x, src_state_pos, dest_state_pos);
}

void xbm_Disconnect(xbm_type x, int src_state_pos, int dest_state_pos)
{
  int tr_pos;
  for(;;)
  {
    tr_pos = xbm_FindTr(x, src_state_pos, dest_state_pos);
    if ( tr_pos < 0 )
      break;
    xbm_DeleteTr(x, tr_pos);
  }
}


static int xbm_add_var(xbm_type x, int direction, const char *name, int reset_value, int is_delay_line, double delay)
{
  xbm_var_type v;
  int pos;
  v = xbm_var_Open(x);
  if ( v != NULL )
  {
    xbm_var_SetDirection(v, direction);
    xbm_var_SetResetValue(v, reset_value);
    v->is_delay_line = is_delay_line;
    v->delay = delay;
    if ( xbm_var_SetName(v, name) != 0 )
    {
      pos = b_set_Add(x->vars, v);
      if ( pos >= 0 )
      {
        return pos;
      }
    }
    xbm_var_Close(v);
  }
  return -1;
}

int xbm_AddVar(xbm_type x, int direction, const char *name, int reset_value)
{
  return xbm_add_var(x, direction, name, reset_value, 0, 0);
}

int xbm_AddDelayLineVar(xbm_type x, int direction, const char *name, int reset_value, double delay)
{
  return xbm_add_var(x, direction, name, reset_value, 1, delay);
}

int xbm_CalcPI(xbm_type x)
{
  int pos = -1;
  int in_idx = 0;
  int out_idx = 0;
  
  while( xbm_LoopVar(x, &pos) != 0 )
  {
    switch( xbm_GetVar(x, pos)->direction )
    {
      case XBM_DIRECTION_IN:
        xbm_GetVar(x, pos)->index = in_idx;
        in_idx++;
        break;
      case XBM_DIRECTION_OUT:
        xbm_GetVar(x, pos)->index = out_idx;
        out_idx++;
        break;
    }
  }
  
  x->inputs = in_idx;
  x->outputs = out_idx;
  
  if ( pinfoSetInCnt(xbm_GetPiCond(x), in_idx+out_idx) == 0 )
    return 0;
    
  if ( pinfoSetOutCnt(xbm_GetPiCond(x), 0) == 0 )
    return 0;

  if ( pinfoSetInCnt(xbm_GetPiIn(x), in_idx) == 0 )
    return 0;

  if ( pinfoSetOutCnt(xbm_GetPiOut(x), out_idx) == 0 )
    return 0;
      
  return 1;
}

int xbm_FindVar(xbm_type x, int direction, const char *name)
{
  int pos = -1;
  
  while( xbm_LoopVar(x, &pos) != 0 )
  {
    if ( direction == XBM_DIRECTION_NONE || 
         xbm_GetVar(x, pos)->direction == direction )
    {
      if ( xbm_GetVar(x, pos)->name != NULL )
      {
        if ( strcmp(xbm_GetVar(x, pos)->name, name) == 0 )
        {
          return pos;
        }
      }
    }
  }
  return -1;
}

const char *xbm_GetVarNameStr(xbm_type x, int pos)
{
  static char s[32];
  if ( xbm_GetVar(x, pos)->name == NULL )
  {
    sprintf(s, "%d", pos);
    return s;
  }
  return xbm_GetVar(x, pos)->name;
}

const char *xbm_GetVarName2Str(xbm_type x, int pos)
{
  static char s[32];
  if ( xbm_GetVar(x, pos)->name2 == NULL )
  {
    sprintf(s, "%d", pos);
    return s;
  }
  return xbm_GetVar(x, pos)->name2;
}

const char *xbm_GetVarNameStrByIndex(xbm_type x, int direction, int index)
{
  int pos = -1;
  
  while( xbm_LoopVar(x, &pos) != 0 )
  {
    if ( direction == XBM_DIRECTION_NONE || 
         xbm_GetVar(x, pos)->direction == direction )
    {
      if ( xbm_GetVar(x, pos)->index == index )
      {
        return xbm_GetVarNameStr(x, pos);
      }
    }
  }
  return "";
}

const char *xbm_GetVarName2StrByIndex(xbm_type x, int direction, int index)
{
  int pos = -1;
  
  while( xbm_LoopVar(x, &pos) != 0 )
  {
    if ( direction == XBM_DIRECTION_NONE || 
         xbm_GetVar(x, pos)->direction == direction )
    {
      if ( xbm_GetVar(x, pos)->index == index )
      {
        return xbm_GetVarName2Str(x, pos);
      }
    }
  }
  return "";
}

const char *xbm_GetStNameStr(xbm_type x, int pos)
{
  static char s[32];
  if ( pos < 0 || pos >= b_set_Max(x->states) )
    return "???";
  if ( xbm_GetSt(x, pos)->name == NULL )
  {
    sprintf(s, "%d", pos);
    return s;
  }
  return xbm_GetSt(x, pos)->name;
}

xbm_var_type xbm_GetVarByIndex(xbm_type x, int direction, int index)
{
  int pos = -1;
  
  while( xbm_LoopVar(x, &pos) != 0 )
  {
    if ( direction == XBM_DIRECTION_NONE || 
         xbm_GetVar(x, pos)->direction == direction )
    {
      if ( xbm_GetVar(x, pos)->index == index )
      {
        return xbm_GetVar(x, pos);
      }
    }
  }
  return NULL;
}


/*---------------------------------------------------------------------------*/

dcube *xbm_GetTrSuper(xbm_type x, int tr_pos)
{
  pinfo *pi = xbm_GetPiIn(x);
  dcube *c = &(pi->tmp[2]);
  dcOrIn( pi, c, 
    &(xbm_GetTr(x, tr_pos)->in_start_cond), 
    &(xbm_GetTr(x, tr_pos)->in_end_cond) );
  return c;
}

int xbm_GetTrCond(xbm_type x, int tr_pos, dclist in_cond_cl)
{
  return dclSharp(xbm_GetPiIn(x), in_cond_cl, 
    xbm_GetTrSuper(x, tr_pos), 
    &(xbm_GetTr(x, tr_pos)->in_end_cond));
}

int xbm_SetCodeWidth(xbm_type x, int code_width)
{
  int st_pos = -1;
  
  if ( pinfoSetOutCnt(xbm_GetPiCode(x), code_width) == 0 )
    return 0;
  
  while( xbm_LoopSt(x, &st_pos) != 0 )
  {
    if ( xbm_state_adjust(xbm_GetPiCode(x), xbm_GetSt(x, st_pos)) == 0 )
      return 0;
  }
  
  xbm_Log(x, 0, "XBM: Code-width set to '%d'.", code_width);
      
  return 1;
}

int xbm_SetPLAFileNameXBMPartitions(xbm_type x, const char *name)
{
  return internal_xbm_set_str(&(x->pla_file_xbm_partitions), name);
}

int xbm_SetPLAFileNamePrimePartitions(xbm_type x, const char *name)
{
  return internal_xbm_set_str(&(x->pla_file_prime_partitions), name);
}

int xbm_SetPLAFileNameMinPartitions(xbm_type x, const char *name)
{
  return internal_xbm_set_str(&(x->pla_file_min_partitions), name);
}

int xbm_SetImportFileNameStateCodes(xbm_type x, const char *name)
{
  return internal_xbm_set_str(&(x->import_file_state_codes), name);
}

int xbm_SetImportFileNamePLAMachine(xbm_type x, const char *name)
{
  return internal_xbm_set_str(&(x->import_file_pla_machine), name);
}


const char *xbm_GetOutLabel(xbm_type x, int index, const char *post)
{
  if ( post == NULL )
    post = "";
  sprintf(x->labelbuf, "%s%s", 
    xbm_GetVarName2StrByIndex(x, XBM_DIRECTION_OUT, index), post);
  return x->labelbuf;
}

const char *xbm_GetStateLabel(xbm_type x, int index, const char *prefix)
{
  if ( prefix == NULL )
    prefix = "";
  sprintf(x->labelbuf, "%s%d", prefix, index);
  return x->labelbuf;
}

/*
  copies all known labels into pi_machine.
*/
int xbm_AssignLabelsToMachine(xbm_type x)
{
  int i, j;
  b_sl_type in_sl;
  b_sl_type out_sl;

  if ( x->pi_machine == NULL )
    return 1;

  in_sl = pinfoGetInLabelList(x->pi_machine);
  out_sl = pinfoGetOutLabelList(x->pi_machine);
  
  if ( in_sl == NULL || out_sl == NULL )
    return 0;  
    
  
  b_sl_Clear(in_sl);
  j = 0;

  for( i = 0; i < x->inputs; i++ )
    if ( b_sl_Set(in_sl, j++, xbm_GetVarName2StrByIndex(x, XBM_DIRECTION_IN, i)) == 0 )
      return 0;

  for( i = 0; i < xbm_GetPiCode(x)->out_cnt; i++ )
    if ( b_sl_Set(in_sl, j++, xbm_GetStateLabel(x, i, "zi")) == 0 )
      return 0;

  if ( x->is_fbo != 0 )
    for( i = 0; i < x->outputs; i++ )
      if ( b_sl_Set(in_sl, j++, xbm_GetOutLabel(x, i, "_fb")) == 0 )
        return 0;

  b_sl_Clear(out_sl);
  j = 0;

  for( i = 0; i < xbm_GetPiCode(x)->out_cnt; i++ )
    if ( b_sl_Set(out_sl, j++, xbm_GetStateLabel(x, i, "zo")) == 0 )
      return 0;

  for( i = 0; i < x->outputs; i++ )
    if ( b_sl_Set(out_sl, j++, xbm_GetOutLabel(x, i, "")) == 0 )
      return 0;

  return 1;  
}

/*
  copied from fsm_GetNextNodeCode
  
  If the XBM is in state 'st_pos' there is target state 'dest_node_id' that
  will be reached under condition 'c'.
  'c' is a cube of xbm_GetPiIn(x).
  'c' MUST be a minterm!!!
  'dest_code' is a cube of xbm_GetPiCode(x).
  'dest_out' is a cube of xbm_GetPiOut(x).
*/
int xbm_GetNextStateCode(xbm_type x, int st_pos, dcube *c, int *dest_st_pos, dcube *dest_code, dcube *dest_out)
{
  int loop;
  int tr_pos;
  int n = 0;
  dcube *tmp;
  
  /* there are two ways to calculate the target code: cl_machine and the graph */
  /* we do use the graph first, because cl_machine might not yet exist. */
  
  loop = -1;
  tr_pos = -1;
  while( xbm_LoopStOutTr(x, st_pos, &loop, &tr_pos) != 0 )
  {
    if ( dcIsInSubSet(xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_end_cond), c) != 0 )
    {
      /* great, transition found */
      n = xbm_GetTrDestStPos(x, tr_pos);
      if ( dest_st_pos != NULL )
        *dest_st_pos = n;
      if ( dest_code != NULL )
        dcCopy(xbm_GetPiCode(x), dest_code, &(xbm_GetSt(x,n)->code));
      if ( dest_out != NULL )
        dcCopy(xbm_GetPiOut(x), dest_out, &(xbm_GetSt(x,n)->out));
      return 1;
    }
  }
  
  /* There was no valid state, so check the machine function */
  
  if ( x->pi_machine == NULL )
    return 0;

  tmp = &(x->pi_machine->tmp[9]);
  
  dcCopyInToIn(x->pi_machine, tmp, 0,         xbm_GetPiIn(x),   c);
  dcCopyInToIn(x->pi_machine, tmp, x->inputs, xbm_GetPiCode(x), &(xbm_GetSt(x,n)->code));

  dclResult(x->pi_machine, tmp, x->cl_machine);
  
  if ( dest_st_pos != NULL )
    *dest_st_pos = -1;
  if ( dest_code != NULL )
  {
    dcCopyOutToOutRange(xbm_GetPiCode(x), dest_code, 0, x->pi_machine, tmp, 
      0,                         xbm_GetPiCode(x)->out_cnt);
  }
  if ( dest_out != NULL )
  {
    dcCopyOutToOutRange(xbm_GetPiOut(x),  dest_out,  0, x->pi_machine, tmp, 
      xbm_GetPiCode(x)->out_cnt, xbm_GetPiOut(x)->out_cnt);
  }
  return 1;  
}

/*---------------------------------------------------------------------------*/

/*
  stores the boolean function in x->pi_machine and x->cl_machine
  
  XBM_BUILD_OPT_MIS   do state minimization
  XBM_BUILD_OPT_FBO   apply feedback optimization
  XBM_BUILD_OPT_SYNC  create synchronous machine
*/

int xbm_Build(xbm_type x, int opt)
{

  xbm_Log(x, 5, "XBM: Build procedure for machine function started.");
  xbm_Log(x, 2, "XBM: Build option value is '%04x'.", opt);

  x->is_fbo = 0;
  if ( (opt & XBM_BUILD_OPT_FBO) != 0 )
    x->is_fbo = 1;

  x->is_sync = 0;
  if ( (opt & XBM_BUILD_OPT_SYNC) != 0 )
    x->is_sync = 1;
    
  x->is_safe_opt = 0;
  
try_again:

  if ( (opt & XBM_BUILD_OPT_MIS) != 0 )
  {
    xbm_Log(x, 4, "XBM: State reduction.");
    if ( xbm_MinimizeStates(x) == 0 )
    {
      xbm_Error(x, "XBM: State reduction failed.");
      return 0;
    }
  }
  else
  {
    if ( xbm_NoMinimizeStates(x) == 0 )
      return 0;
  }

  xbm_Log(x, 4, "XBM: State encoding.");
  if ( xbm_EncodeStates(x) == 0 )
  {
    xbm_Error(x, "XBM: State encoding failed.");
    return 0;
  }
    
  xbm_Log(x, 4, "XBM: Build machine function.");
  if ( xbm_BuildTransferFunction(x) == 0 )
  {
    
    if ( x->is_safe_opt == 0 )
    {
      x->is_safe_opt = 1;
      xbm_Log(x, 4, "XBM: Build machine function failed. Retry...");
      goto try_again;
    }
    
    xbm_Error(x, "XBM: Build machine function failed.");
    return 0;
  }

  xbm_Log(x, 4, "XBM: Check machine function.");
  if ( xbm_CheckTransferFunction(x) == 0 )
  {
    xbm_Error(x, "XBM: Check machine function failed.");
    return 0;
  }

  if ( xbm_AssignLabelsToMachine(x) == 0 )
  {
    xbm_Error(x, "XBM: Label assignment failed.");
    return 0;
  }

  if ( x->pi_machine != NULL && x->cl_machine != NULL )
  {
    int fb = xbm_GetPiCode(x)->out_cnt;        


    if ( x->is_fbo != 0 )
      fb += xbm_GetPiOut(x)->out_cnt;
    xbm_Log(x, 5, "XBM: Machine function created (feed-backs: %d, implicants: %d, literals: %d).",
      fb,
      dclCnt(x->cl_machine), 
      dclGetLiteralCnt(x->pi_machine, x->cl_machine) );
  }
  else
  {
    xbm_Error(x, "XBM: Failed.");
    return 0;
  }
  
    
  return 1;
}

int xbm_GetFeedbackWidth(xbm_type x)
{
  if ( x->is_fbo != 0 )
    return xbm_GetPiCode(x)->out_cnt + xbm_GetPiOut(x)->out_cnt;
  return xbm_GetPiCode(x)->out_cnt;
}
