/*

  fsmbms.c
  
  read a burst mode specification

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
  
  
  11 feb 2002  Oliver Kraus <olikraus@yahoo.com>
      working on XBM extentions...
  
  
  A+  rising edge
  A-  falling edge
  A*  directed don't care
  A#  high level
  A~  low level
  
  BMS spec:
  http://www.cs.columbia.edu/~nowick/minimalist/node4.html
  
  file := input|output|reset|line
  input := "input" name level
  output := "output" name level
  reset := "reset" name
  line := name name inlist "|" outlist
  inlist := {transition}
  outlist := {transition}
  transition := name "+" | name "-"
  
*/

#include "fsm.h"
#include <assert.h>
#include <ctype.h>
#include <string.h>
#include "mwc.h"
#include "b_ff.h"

#define BMS_STATE_NAME 64

#define BMS_MAX_INPUT 128
#define BMS_MAX_OUTPUT 128

static char bms_initial_input_values[BMS_MAX_INPUT];
static char bms_initial_output_values[BMS_MAX_OUTPUT];

#define BMS_VAR_NONE 0
#define BMS_VAR_EDGE 1
#define BMS_VAR_LEVEL 2
static char bms_variable_type[BMS_MAX_INPUT];

static int bms_state = 0;
static char bms_reset_name[BMS_STATE_NAME];


static void fsm_bms_skipspace(char **s) 
{  
  if ( **(unsigned char **)s >= 128 )
    return;
  while(isspace(**(unsigned char **)s))
    (*s)++;
}

static char *fsm_bms_getid(char **s)
{
   static char t[256];
   int i = 0;
   while( (**s >= '0' || **s == '!' || **s == '(' || **s == ')' || **s == '/') && **s != '~' )
   {
      if ( i < 255 )
        t[i] = **s;
      (*s)++;
      i++;
   }
   t[i] = '\0';
   return t;
}

static int fsm_bms_getval(char **s)
{
   int result = 0;
   while( **s >= '0' && **s <= '9' ) 
   {
      result = result*10+(**s)-'0';
      (*s)++;
   }
   return result;
}


static void fsm_bms_clear(fsm_type fsm)
{
  int i;
  
  assert(fsm->input_sl != NULL);
  assert(fsm->output_sl != NULL);
  bms_state = 0;

  for( i = 0; i < BMS_MAX_INPUT; i++ )
    bms_variable_type[i] = BMS_VAR_NONE;

  fsm_Clear(fsm);
  b_sl_Clear(fsm->input_sl);
  b_sl_Clear(fsm->output_sl);
}

static int fsm_bms_add_input(fsm_type fsm, const char *name, int val)
{
  int pos;
  pos = b_sl_Add(fsm->input_sl, name);
  if ( pos < 0 )
    return 0;
  if ( pos >= BMS_MAX_INPUT )
    return 0;
  bms_initial_input_values[pos] = (char)val;
  return 1;
}

static int fsm_bms_add_output(fsm_type fsm, const char *name, int val)
{
  int pos;
  pos = b_sl_Add(fsm->output_sl, name);
  if ( pos < 0 )
    return 0;
  if ( pos >= BMS_MAX_OUTPUT )
    return 0;
  bms_initial_output_values[pos] = (char)val;
  return 1;
}

static int fsm_bms_get_input_pos(fsm_type fsm, const char *name)
{
  return b_sl_Find(fsm->input_sl, name);
}

static int fsm_bms_get_output_pos(fsm_type fsm, const char *name)
{
  return b_sl_Find(fsm->output_sl, name);
}


static int fsm_bms_line(fsm_type fsm, pinfo *pi, char *s, int *is_xbm)
{
  char *t;
  int v;
  int i;
  int is_output;
  char from[BMS_STATE_NAME];
  char to[BMS_STATE_NAME];
  dcube *c;
  int edge_id;
  
  if ( *s == '#' )
    return 1;

  if ( *s == ';' )
    return 1;
  
  fsm_bms_skipspace(&s);
  if ( *s == '\0' )
    return 1;
    
  if ( bms_state == 0 )
  {

    if ( strncasecmp(s, "name", 4) == 0 )
    {
      s += 4;
      fsm_bms_skipspace(&s);
      t = fsm_bms_getid(&s);
      return fsm_SetName(fsm, t);
    }
    if ( strncasecmp(s, "input", 5) == 0 )
    {
      s += 5;
      fsm_bms_skipspace(&s);
      t = fsm_bms_getid(&s);
      fsm_bms_skipspace(&s);
      v = 0;
      if ( *s != '\0' )
        v = fsm_bms_getval(&s);
      return fsm_bms_add_input(fsm, t, v);
    }
    if ( strncasecmp(s, "output", 6) == 0 )
    {
      s += 6;
      fsm_bms_skipspace(&s);
      t = fsm_bms_getid(&s);
      fsm_bms_skipspace(&s);
      v = 0;
      if ( *s != '\0' )
        v = fsm_bms_getval(&s);
      return fsm_bms_add_output(fsm, t, v);
    }
    if ( strncasecmp(s, "reset", 5) == 0 )
    {
      s += 5;
      fsm_bms_skipspace(&s);
      t = fsm_bms_getid(&s);
      strncpy(bms_reset_name, t, BMS_STATE_NAME);
      bms_reset_name[BMS_STATE_NAME-1] = '\0';
      return 1;
    }

    
    bms_state = 1;
    if ( fsm_SetInCnt(fsm, b_sl_GetCnt(fsm->input_sl)) == 0 )
      return fsm_Log(fsm, "BMS read: Out of memory."), 0;
      
    if ( fsm_SetOutCnt(fsm, b_sl_GetCnt(fsm->output_sl)) == 0 )
      return fsm_Log(fsm, "BMS read: Out of memory."), 0;
      
    if ( pinfoSetInCnt(pi, b_sl_GetCnt(fsm->input_sl)+b_sl_GetCnt(fsm->output_sl)) == 0 )
      return fsm_Log(fsm, "BMS read: Out of memory."), 0;
    
    fsm_Log(fsm, "BMS read: Burst mode specification with %d inputs and %d outputs.", 
      b_sl_GetCnt(fsm->input_sl), b_sl_GetCnt(fsm->output_sl));
  }

  c = &(pi->tmp[11]);
  dcInSetAll(pi, c, 0);
  
  t = fsm_bms_getid(&s);
  if ( strlen(t)+1 >= BMS_STATE_NAME )
    return 0;
  strcpy(from, t);
  fsm_bms_skipspace(&s);
  t = fsm_bms_getid(&s);
  if ( strlen(t)+1 >= BMS_STATE_NAME )
    return 0;
  strcpy(to, t);
  fsm_bms_skipspace(&s);
  
  while( *s != '\0' )
  {
    t = fsm_bms_getid(&s);
    if ( t == NULL )
    {
      fsm_Log(fsm, "BMS read: Illegal identifier.");
      return 0;
    }
    if ( *t == '\0' )
    {
      fsm_Log(fsm, "BMS read: Empty identifier.");
      return 0;
    }

    is_output = 0;
    i = fsm_bms_get_input_pos(fsm, t);
    if ( i < 0 )
    {
      is_output = 1;
      i = fsm_bms_get_output_pos(fsm, t);
      if ( i < 0 )
      {
        fsm_Log(fsm, "BMS read: Unknown signal '%s'.", t);
        return 0;
      }
    }
    
    fsm_bms_skipspace(&s);
      
    if ( *s != '+' && *s != '-' && *s != '#' && *s != '~' && *s != '*' )
    {
      fsm_Log(fsm, "BMS read: '+', '-', '#', '~' or '*' expected after '%s'.", t);
      s++;
      continue;
    }
    
    v = 0;
    if ( *s == '+' || *s == '#' )
      v = 1;
    else if ( *s == '*' )
    {
      if ( is_xbm != NULL )
        *is_xbm = 1;
      v = 2;
    }

    if ( is_output == 0 )
    {    
      if ( *s == '+' || *s == '-' )
      {
        if ( bms_variable_type[i] == BMS_VAR_LEVEL )
        {
          fsm_Log(fsm, "BMS read: Variable '%s' must be either a level oder edge sensitive.", t);
          return 0;
        }
        bms_variable_type[i] = BMS_VAR_EDGE;
      }
      else if ( *s == '#' || *s == '~' )
      {
        if ( is_xbm != NULL )
          *is_xbm = 1;
        if ( bms_variable_type[i] == BMS_VAR_EDGE )
        {
          fsm_Log(fsm, "BMS read: Variable '%s' must be either a level oder edge sensitive.", t);
          return 0;
        }
        bms_variable_type[i] = BMS_VAR_LEVEL;
      }
    }
    
    s++;
      
    fsm_bms_skipspace(&s);
    while ( *s == '|' )
    {
      s++;
      fsm_bms_skipspace(&s);
    }
      

    if ( is_output != 0 )
    {
      dcSetIn(c, i+b_sl_GetCnt(fsm->input_sl), 1+v);
    }
    else
    {
      dcSetIn(c, i, 1+v);
    }
  }

  edge_id = fsm_FindEdgeByName(fsm, from, to);
  if ( edge_id >= 0 )
    return fsm_Log(fsm, "BMS read: Edge from %s to %s already exists.", from, to), 0;

  edge_id = fsm_ConnectByName(fsm, from, to, 1);
  if ( edge_id < 0 )
    return 0;
  
  if ( dcInit(pi, &(fsm_GetEdge(fsm, edge_id)->tmp_cube) ) == 0 )
    return fsm_Log(fsm, "BMS read: Out of memory (dcInit)."), 0;

  /* printf("line output %s\n", dcToStr(pi, c, "", "")); */
  dcCopy(pi, &(fsm_GetEdge(fsm, edge_id)->tmp_cube), c);
  
  return 1;
  
}

static int fsm_bms_reset_name(fsm_type fsm, const char *name)
{
  int node_id = -1;
  fsm->reset_node_id = -1;
  while( fsm_LoopNodes(fsm, &node_id) != 0 )
  {
    if ( fsm_GetNodeName(fsm, node_id) != 0 )
      if ( strcasecmp(name, fsm_GetNodeName(fsm, node_id) ) == 0 )
      {
        fsm->reset_node_id = node_id;
      }
  }
  return 1;
}

static int fsm_bms_fill(fsm_type fsm, pinfo *pi, int is_xbm)
{ 
  int node_id;
  int edge_id;
  int loop;
  int i;
  int cnt = b_sl_GetCnt(fsm->input_sl)+b_sl_GetCnt(fsm->output_sl);
  int something_was_done;
  int turn = 0;
  dcube *c;
  c = &(pi->tmp[11]);

  if ( fsm->reset_node_id >= 0 )
  {

    /*
    edge_id = fsm_FindEdge(fsm, fsm->reset_node_id, fsm->reset_node_id);
    if ( edge_id >= 0 )
      return fsm_Log(fsm, "BMS read: Reset edge from %s to %s already exists.", fsm->reset_node_id, fsm->reset_node_id), 0;
    */
    edge_id = fsm_Connect(fsm, fsm->reset_node_id, fsm->reset_node_id);
    if ( edge_id < 0 )
      return 0;

    if ( dcInit(pi, &(fsm_GetEdge(fsm, edge_id)->tmp_cube) ) == 0 )
      return fsm_Log(fsm, "BMS read: Out of memory (dcInit)."), 0;

    for( i = 0; i < b_sl_GetCnt(fsm->input_sl); i++ )
      dcSetIn(&(fsm_GetEdge(fsm, edge_id)->tmp_cube), i, 
        bms_initial_input_values[i]+1);

    for( i = b_sl_GetCnt(fsm->input_sl); 
         i < b_sl_GetCnt(fsm->output_sl)+b_sl_GetCnt(fsm->input_sl); 
         i++ )
      dcSetIn(&(fsm_GetEdge(fsm, edge_id)->tmp_cube), i,
        bms_initial_output_values[i]+1);
  }

  do
  {  
    turn++;
    something_was_done = 0;
    node_id = -1;
    while(fsm_LoopNodes(fsm, &node_id) != 0)
    {
      edge_id = -1;
      loop = -1;
      dcInSetAll(pi, c, 0);
      while(fsm_LoopNodeInEdges(fsm, node_id, &loop, &edge_id) != 0)
        dcOrIn( pi, c, c, &(fsm_GetEdge(fsm, edge_id)->tmp_cube) );

      if ( is_xbm == 0 ) 
      {
        /* there must be no 11 code inside the cube */
        for( i = 0; i < cnt; i++ )
          if ( dcGetIn(c, i) == 3 )
            return fsm_Log(fsm, "BMS read: Signal %s is ambiguous for destination node %s.",
              i < b_sl_GetCnt(fsm->input_sl)?
              b_sl_GetVal(fsm->input_sl, i):
              b_sl_GetVal(fsm->output_sl, i-b_sl_GetCnt(fsm->input_sl)),
              fsm_GetNodeName(fsm, node_id)==NULL?"<unknown>":fsm_GetNodeName(fsm, node_id)
              ), 0;
      }
      /*
      printf("Node %s is %s\n", fsm_GetNodeName(fsm, node_id), dcToStr(pi, c, " ", ""));
      */

      /*
      edge_id = -1;
      loop = -1;
      while(fsm_LoopNodeOutEdges(fsm, node_id, &loop, &edge_id) != 0)
        for( i = 0; i < cnt; i++ )
          if ( dcGetIn(c, i) != 0 )
            if ( dcGetIn(c, i) == dcGetIn(&(fsm_GetEdge(fsm, edge_id)->tmp_cube), i)
              &&  dcGetIn(c, i) != 3 )
              return fsm_Log(fsm, "BMS read: Signal %s assigned twice (source node %s).",
                i < b_sl_GetCnt(fsm->input_sl)?
                b_sl_GetVal(fsm->input_sl, i):
                b_sl_GetVal(fsm->output_sl, i-b_sl_GetCnt(fsm->input_sl)),
                fsm_GetNodeName(fsm, node_id)==NULL?"<unknown>":fsm_GetNodeName(fsm, node_id)
                ), 0;
      */

      edge_id = -1;
      loop = -1;
      while(fsm_LoopNodeOutEdges(fsm, node_id, &loop, &edge_id) != 0)
        for( i = 0; i < cnt; i++ )
          if ( dcGetIn(c, i) != 0 )
            if ( dcGetIn(&(fsm_GetEdge(fsm, edge_id)->tmp_cube), i) == 0 )
            {
              if ( i >= b_sl_GetCnt(fsm->input_sl) )
                dcSetIn(&(fsm_GetEdge(fsm, edge_id)->tmp_cube), i, dcGetIn(c, i));
              else if ( bms_variable_type[i] == BMS_VAR_EDGE )
                dcSetIn(&(fsm_GetEdge(fsm, edge_id)->tmp_cube), i, dcGetIn(c, i));
              else
                dcSetIn(&(fsm_GetEdge(fsm, edge_id)->tmp_cube), i, 3);
              /*
              printf("%d: %s -> %s  %d = %d\n",
                turn,
                fsm_GetNodeName(fsm, fsm_GetEdgeSrcNode(fsm, edge_id)),
                fsm_GetNodeName(fsm, fsm_GetEdgeDestNode(fsm, edge_id)),
                i, 
                dcGetIn(c, i)-1
                );
              */
              something_was_done = 1;
            }
    }
  } while( something_was_done != 0 );
  
  return 1;
}

static int fsm_bms_copy(fsm_type fsm, pinfo *pi)
{
  int edge_id;
  int in_cnt = b_sl_GetCnt(fsm->input_sl);
  int out_cnt = b_sl_GetCnt(fsm->output_sl);
  dcube *oc;
  dcube *cc;
  dcube *c;
  
  oc = &(fsm_GetOutputPINFO(fsm)->tmp[11]);
  cc = &(fsm_GetConditionPINFO(fsm)->tmp[11]);

  dcInSetAll(fsm_GetOutputPINFO(fsm), oc, CUBE_IN_MASK_DC);
  dcOutSetAll(fsm_GetOutputPINFO(fsm), oc, CUBE_OUT_MASK);

  dcInSetAll(fsm_GetConditionPINFO(fsm), cc, CUBE_IN_MASK_DC);
  dcOutSetAll(fsm_GetConditionPINFO(fsm), cc, CUBE_OUT_MASK);
  
  edge_id = -1;
  while( fsm_LoopEdges(fsm, &edge_id) != 0 )
  {
    c = &(fsm_GetEdge(fsm, edge_id)->tmp_cube);
    dcCopyInToInRange(fsm_GetConditionPINFO(fsm), cc, 0, pi, c, 0, in_cnt);
    dcCopyInToInRange(fsm_GetOutputPINFO(fsm), oc, 0, pi, c, 0, in_cnt);
    dcCopyInToOutRange(fsm_GetOutputPINFO(fsm), oc, 0, pi, c, in_cnt, out_cnt);
    dcSetOut(oc, out_cnt, 1);
    
    if ( dcIsIllegal(fsm_GetConditionPINFO(fsm), cc) != 0 )
    {
      fsm_Log(fsm, "BMS read: Transition %s --> %s.",
        fsm_GetNodeNameStr(fsm, fsm_GetEdgeSrcNode(fsm, edge_id)), 
        fsm_GetNodeNameStr(fsm, fsm_GetEdgeDestNode(fsm, edge_id)));
      return fsm_Log(fsm, "BMS read: Illegal specification (condition, open loop and/or unused variables)."), 0;
    }

    if ( dcIsIllegal(fsm_GetConditionPINFO(fsm), oc) != 0 )
    {
      fsm_Log(fsm, "BMS read: Transition %s --> %s.",
        fsm_GetNodeNameStr(fsm, fsm_GetEdgeSrcNode(fsm, edge_id)), 
        fsm_GetNodeNameStr(fsm, fsm_GetEdgeDestNode(fsm, edge_id)));
      return fsm_Log(fsm, "BMS read: Illegal specification (output, open loop and/or unused variables)."), 0;
    }

    dclRealClear(fsm_GetEdgeOutput(fsm, edge_id));
    if ( dclAdd(fsm_GetOutputPINFO(fsm), fsm_GetEdgeOutput(fsm, edge_id), oc) < 0 )
      return fsm_Log(fsm, "BMS read: Out of memory (dclAdd)."), 0;

    dclRealClear(fsm_GetEdgeCondition(fsm, edge_id));
    if ( dclAdd(fsm_GetConditionPINFO(fsm), fsm_GetEdgeCondition(fsm, edge_id), cc) < 0 )
      return fsm_Log(fsm, "BMS read: Out of memory (dclAdd)."), 0;
  }
  return 1;
}

void fsm_bms_destroy_cube(fsm_type fsm)
{
  int edge_id;
  edge_id = -1;
  while( fsm_LoopEdges(fsm, &edge_id) != 0 )
    dcDestroy(&(fsm_GetEdge(fsm, edge_id)->tmp_cube));
}

int fsm_bms_full_stable(fsm_type fsm)
{
  int node_id, edge_id, loop;
  dclist cl_self, cl_self_o;
  dclist cl;
  int i;
  dcube *oc;

  oc = &(fsm_GetOutputPINFO(fsm)->tmp[11]);
  
  if ( dclInit(&cl_self) == 0 )
    return 0;

  if ( dclInit(&cl_self_o) == 0 )
    return dclDestroy(cl_self), 0;
  
  node_id = -1;
  while( fsm_LoopNodes(fsm, &node_id) != 0 )
  {
    dclClear(cl_self);
    dclClear(cl_self_o);


    if ( fsm_GetNodePostCover(fsm, node_id, cl_self) == 0 )
      return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;

    if ( dclComplement(fsm_GetConditionPINFO(fsm), cl_self) == 0 )
    {
      fsm_Log(fsm, "BMS read: Out of memory (dclComplement).");
      return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;
    }

    loop = -1;
    edge_id = -1;
    dcInSetAll(fsm_GetOutputPINFO(fsm), oc, CUBE_IN_MASK_DC);
    dcOutSetAll(fsm_GetOutputPINFO(fsm), oc, 0);
    while( fsm_LoopNodeInEdges(fsm, node_id, &loop, &edge_id) != 0 )
    {
      cl = fsm_GetEdgeOutput(fsm, edge_id);
      for( i = 0; i < dclCnt(cl); i++)
        dcOrOut(fsm_GetOutputPINFO(fsm), oc, oc, dclGet(cl, i));
    }

    if ( fsm_GetNodePostOCover(fsm, node_id, cl_self_o) == 0 )
      return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;
      
    for( i = 0; i < dclCnt(cl_self_o); i++ )
      dcOutSetAll(fsm_GetOutputPINFO(fsm), dclGet(cl_self_o, i), CUBE_OUT_MASK);

    if ( dclComplement(fsm_GetOutputPINFO(fsm), cl_self_o) == 0 )
    {
      fsm_Log(fsm, "BMS read: Out of memory (dclComplement).");
      return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;
    }

    for( i = 0; i < dclCnt(cl_self_o); i++ )
      dcCopyOut(fsm_GetOutputPINFO(fsm), dclGet(cl_self_o, i), oc);

    edge_id = fsm_Connect(fsm, node_id, node_id);
    if ( edge_id < 0 )
    {
      fsm_Log(fsm, "BMS read: Out of memory (fsm_Connect).");
      return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;
    }
    
    dclRealClear(fsm_GetEdgeCondition(fsm, edge_id));
    if ( dclCopy(fsm_GetConditionPINFO(fsm), fsm_GetEdgeCondition(fsm, edge_id), cl_self) == 0 )
    {
      fsm_Log(fsm, "BMS read: Out of memory (dclCopy).");
      return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;
    }
    
    dclRealClear(fsm_GetEdgeOutput(fsm, edge_id));
    if ( dclCopy(fsm_GetOutputPINFO(fsm), fsm_GetEdgeOutput(fsm, edge_id), cl_self_o) == 0 )
    {
      fsm_Log(fsm, "BMS read: Out of memory (dclCopy).");
      return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;
    }
  }

  return dclDestroy(cl_self), dclDestroy(cl_self_o), 1;
}


int fsm_bms_io_stable(fsm_type fsm)
{
  dclist cl_self, cl_self_o;
  dcube *oc;
  dcube *cc;
  int node_id, edge_id, e1, l1, e2, l2, i1, i2;
  dclist cl_in, cl_out;
  dclist cl_in_o, cl_out_o;

  cc = &(fsm_GetConditionPINFO(fsm)->tmp[11]);
  oc = &(fsm_GetOutputPINFO(fsm)->tmp[11]);
  
  if ( dclInit(&cl_self) == 0 )
    return 0;

  if ( dclInit(&cl_self_o) == 0 )
    return dclDestroy(cl_self), 0;

  node_id = -1;
  while( fsm_LoopNodes(fsm, &node_id) != 0 )
  {
    dclClear(cl_self);
    dclClear(cl_self_o);

    l1 = -1;
    e1 = -1;
    while( fsm_LoopNodeInEdges(fsm, node_id, &l1, &e1) != 0 )
    {
      cl_in = fsm_GetEdgeCondition(fsm, e1);
      cl_in_o = fsm_GetEdgeOutput(fsm, e1);
      l2 = -1;
      e2 = -1;
      while( fsm_LoopNodeOutEdges(fsm, node_id, &l2, &e2) != 0 )
      {
        if ( e1 != e2 )
        {
          cl_out = fsm_GetEdgeCondition(fsm, e2);
          cl_out_o = fsm_GetEdgeOutput(fsm, e2);
          dcOutSetAll(fsm_GetConditionPINFO(fsm), cc, CUBE_OUT_MASK);
          dcOutSetAll(fsm_GetOutputPINFO(fsm), oc, CUBE_OUT_MASK);
          
          for( i1 = 0; i1 < dclCnt(cl_in); i1++ )
          {
            for( i2 = 0; i2 < dclCnt(cl_out); i2++ )
            {
              dcOrIn(fsm_GetConditionPINFO(fsm), cc, dclGet(cl_in, i1), dclGet(cl_out, i2));
              if ( dclAdd(fsm_GetConditionPINFO(fsm), cl_self, cc) < 0 )
              {
                fsm_Log(fsm, "BMS read: Out of memory (dclAdd).");
                return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;
              }
            }
          }

          for( i1 = 0; i1 < dclCnt(cl_in_o); i1++ )
          {
            for( i2 = 0; i2 < dclCnt(cl_out_o); i2++ )
            {
              dcOrIn(fsm_GetOutputPINFO(fsm), oc, dclGet(cl_in_o, i1), dclGet(cl_out_o, i2));
              if ( dclAdd(fsm_GetOutputPINFO(fsm), cl_self_o, oc) < 0 )
              {
                fsm_Log(fsm, "BMS read: Out of memory (dclAdd).");
                return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;
              }
            }
          }
  
        } /* if */
      } /* out edges */
    } /* in edges */
    
    dclSCC(fsm_GetOutputPINFO(fsm), cl_self_o);
    dclSCC(fsm_GetConditionPINFO(fsm), cl_self);
  
    l2 = -1;
    e2 = -1;
    while( fsm_LoopNodeOutEdges(fsm, node_id, &l2, &e2) != 0 )
    {
      if ( fsm_GetEdgeSrcNode(fsm, e2) !=  fsm_GetEdgeDestNode(fsm, e2) )
      {
        cl_out = fsm_GetEdgeCondition(fsm, e2);
        for( i2 = 0; i2 < dclCnt(cl_out); i2++ )
        {
          dcCopyIn(fsm_GetConditionPINFO(fsm), cc, dclGet(cl_out, i2));
          dcOutSetAll(fsm_GetConditionPINFO(fsm), cc, CUBE_OUT_MASK);
          if ( dclSCCSubtractCube(fsm_GetConditionPINFO(fsm), cl_self, cc) == 0 )
          {
            fsm_Log(fsm, "BMS read: Out of memory (dclSCCSubtractCube).");
            return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;
          }
        }

        cl_out_o = fsm_GetEdgeOutput(fsm, e2);
        for( i2 = 0; i2 < dclCnt(cl_out_o); i2++ )
        {
          dcCopyIn(fsm_GetOutputPINFO(fsm), oc, dclGet(cl_out_o, i2));
          dcOutSetAll(fsm_GetOutputPINFO(fsm), oc, CUBE_OUT_MASK);
          if ( dclSCCSubtractCube(fsm_GetOutputPINFO(fsm), cl_self_o, oc) == 0 )
          {
            fsm_Log(fsm, "BMS read: Out of memory (dclSCCSubtractCube).");
            return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;
          }
        }        
      } /* if */
    } /* while out edges */
    

    /* calculate the common output of all incoming edges */
    
    dcOutSetAll(fsm_GetOutputPINFO(fsm), oc, 0);
    dcSetOut(oc, fsm_GetOutputPINFO(fsm)->out_cnt-1, 1);
    l1 = -1;
    e1 = -1;
    while( fsm_LoopNodeInEdges(fsm, node_id, &l1, &e1) != 0 )
    {
      if ( fsm_GetEdgeSrcNode(fsm, e1) !=  fsm_GetEdgeDestNode(fsm, e1) )
      {
        cl_in_o = fsm_GetEdgeOutput(fsm, e1);
        for( i1 = 0; i1 < dclCnt(cl_in_o); i1++ )
        {
          dcOrOut(fsm_GetOutputPINFO(fsm), oc, oc, dclGet(cl_in_o, i1));
        }
      }
    }
    /* puts(dcToStr(fsm_GetOutputPINFO(fsm), dclGet(cl_in_o, i1), " ", "")); */
    
    /* assign the calculated output */
    
    for( i2 = 0; i2 < dclCnt(cl_self_o); i2++ )
      dcCopyOut(fsm_GetOutputPINFO(fsm), dclGet(cl_self_o, i2), oc);

    
    edge_id = fsm_Connect(fsm, node_id, node_id);
    if ( edge_id < 0 )
    {
      fsm_Log(fsm, "BMS read: Out of memory (fsm_Connect).");
      return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;
    }

    dclRealClear(fsm_GetEdgeCondition(fsm, edge_id));
    if ( dclCopy(fsm_GetConditionPINFO(fsm), fsm_GetEdgeCondition(fsm, edge_id), cl_self) == 0 )
    {
      fsm_Log(fsm, "BMS read: Out of memory (dclCopy).");
      return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;
    }

    dclRealClear(fsm_GetEdgeOutput(fsm, edge_id));
    if ( dclCopy(fsm_GetOutputPINFO(fsm), fsm_GetEdgeOutput(fsm, edge_id), cl_self_o) == 0 )
    {
      fsm_Log(fsm, "BMS read: Out of memory (dclCopy).");
      return dclDestroy(cl_self), dclDestroy(cl_self_o), 0;
    }
    
  } /* while nodes */

  return dclDestroy(cl_self), dclDestroy(cl_self_o), 1;
}

int fsm_bms_reset(fsm_type fsm)
{
  int edge_id, node_id;
  dclist cl;

  dcube *oc = &(fsm_GetOutputPINFO(fsm)->tmp[11]);

  int i;
  int cnt = b_sl_GetCnt(fsm->input_sl)+b_sl_GetCnt(fsm->output_sl);

  if ( fsm->reset_node_id >= 0 )
    return 1;

  cnt = b_sl_GetCnt(fsm->input_sl);
  for( i = 0; i < cnt; i++ )
    dcSetIn(oc, i, bms_initial_input_values[i]+1);

  cnt = b_sl_GetCnt(fsm->output_sl);
  for( i = 0; i < cnt; i++ )
    dcSetOut(oc, i, (int)(unsigned char)bms_initial_output_values[i]);
  
  fsm->reset_node_id = -1;
  node_id = -1;
  while( fsm_LoopNodes(fsm, &node_id) != 0 )
  {
    edge_id = fsm_FindEdge(fsm, node_id, node_id);
    assert(edge_id >= 0);

    if ( dclIsSubSet(fsm_GetOutputPINFO(fsm), fsm_GetEdgeOutput(fsm, edge_id), oc) != 0 )
    {
      fsm->reset_node_id = node_id;
      break;
    }
  }
  if ( fsm->reset_node_id < 0 )
  {
    fsm_Log(fsm, "BMS read: No valid reset found.");
    fsm_Log(fsm, "BMS read: Reset is: '%s'.", 
      dcToStr(fsm_GetOutputPINFO(fsm), oc, " ", ""));
    node_id = -1;
    while( fsm_LoopNodes(fsm, &node_id) != 0 )
    {
      edge_id = fsm_FindEdge(fsm, node_id, node_id);
      cl = fsm_GetEdgeOutput(fsm, edge_id);
      for( i = 0; i < dclCnt(cl); i++ )
        fsm_Log(fsm, "BMS read: Node '%s': '%s'.", 
          fsm_GetNodeNameStr(fsm, node_id),
          dcToStr(fsm_GetOutputPINFO(fsm), dclGet(cl, i), " ", ""));
    }
    
  }
  return 1;
}

int fsm_bms_read(fsm_type fsm, FILE *fp)
{
  char *line = fsm->linebuf;
  char *s;
  pinfo pi;
  int is_xbm;

  fsm_bms_clear(fsm);

  if ( pinfoInit(&pi) == 0 )
  {
    fsm_Log(fsm, "BMS read: Out of memory.");
    return 0;
  }
  
  bms_state = 0;
  
  bms_reset_name[0] = '\0';

  /* read signal transitions into the tmp_cube */
  /* this will also allocate memory for the tmp_cubes */
  
  is_xbm = 0;
  for(;;)
  {
    s = fgets(line, FSM_LINE_LEN, fp);
    if ( s == NULL )
      break;
    if ( fsm_bms_line(fsm, &pi, s, &is_xbm) == 0 )
      return pinfoDestroy(&pi), fsm_bms_destroy_cube(fsm), 0;
  }

  /* try to apply a user defined reset node */

  if ( bms_reset_name[0] != '\0' )
    if ( fsm_bms_reset_name(fsm, bms_reset_name) == 0 )
      return 0;


  /* propagate all signals through the graph */
  
  if ( fsm_bms_fill(fsm, &pi, is_xbm) == 0 )
    return pinfoDestroy(&pi), fsm_bms_destroy_cube(fsm), 0;


  /* copy the tmp_cube to it's normal location */
    
  if ( fsm_bms_copy(fsm, &pi) == 0 )
    return pinfoDestroy(&pi), fsm_bms_destroy_cube(fsm), 0;


  /* free allocate memory of the tmp_cubes and its pinfo structure */

  fsm_bms_destroy_cube(fsm);
  pinfoDestroy(&pi);


  /* create the self/stable loop conditions */
  
  if ( fsm_bms_io_stable(fsm) == 0 )
    return 0;

    
  /* find and assigns reset state */
  
  if ( fsm_bms_reset(fsm) == 0 )
    return 0;

  
  fsm_Log(fsm, "BMS read: Burst mode specification successfully read.");
  
  return 1;
}

int fsm_ReadBMS(fsm_type fsm, const char *name)
{
  int ret = 0;
  FILE *fp;
  fp = b_fopen(name, NULL, ".bms", "r");
  if ( fp != NULL )
  {
    ret = fsm_bms_read(fsm, fp);
    fclose(fp);
  }
  return ret;
}

/*---------------------------------------------------------------------------*/

static int bms_have_name = 0;
static int bms_have_input = 0;
static int bms_have_output = 0;

static int is_bms_line(char *s)
{
  char *t;
  //int v;
  char from[BMS_STATE_NAME];
  char to[BMS_STATE_NAME];
  
  fsm_bms_skipspace(&s);
  if ( *s == '\0' )
    return 1;
    
  if ( *s == '#' )
    return 1;

  if ( *s == ';' )
    return 1;
    
  if ( bms_state == 0 )
  {

    if ( strncasecmp(s, "name", 4) == 0 )
    {
      s += 4;
      fsm_bms_skipspace(&s);
      t = fsm_bms_getid(&s);
      bms_have_name = 1;
      return 1;
    }
    if ( strncasecmp(s, "input", 5) == 0 )
    {
      s += 5;
      fsm_bms_skipspace(&s);
      t = fsm_bms_getid(&s);
      fsm_bms_skipspace(&s);
      //v = 0;
      if ( *s != '\0' )
        fsm_bms_getval(&s);
      bms_have_input = 1;
      return 1;
    }
    if ( strncasecmp(s, "output", 6) == 0 )
    {
      s += 6;
      fsm_bms_skipspace(&s);
      t = fsm_bms_getid(&s);
      fsm_bms_skipspace(&s);
      //v = 0;
      if ( *s != '\0' )
        fsm_bms_getval(&s);
      bms_have_output = 1;
      return 1;
    }
    if ( strncasecmp(s, "reset", 5) == 0 )
    {
      fsm_bms_skipspace(&s);
      t = fsm_bms_getid(&s);
      return 1;
    }
    
    if ( bms_have_input == 0 )
      return 0;
    
    bms_state = 1;

  }

  t = fsm_bms_getid(&s);
  if ( strlen(t)+1 >= BMS_STATE_NAME )
    return 0;
  strcpy(from, t);
  fsm_bms_skipspace(&s);
  if ( *s == '\0' )
    return 0;
  t = fsm_bms_getid(&s);
  if ( strlen(t)+1 >= BMS_STATE_NAME )
    return 0;
  strcpy(to, t);
  fsm_bms_skipspace(&s);
  if ( *s == '\0' )
    return 0;
  
  while( *s != '\0' )
  {
    t = fsm_bms_getid(&s);
    fsm_bms_skipspace(&s);
      
    if ( *s != '+' && *s != '-' && *s != '#' && *s != '~' && *s != '*' )
    {
      return 0;
    }
    
    /*
    v = 0;
    if ( *s == '+' )
      v = 1;
    if ( *s == '*' )
      v = 2;
    */
    s++;
      
    fsm_bms_skipspace(&s);
    while ( *s == '|' )
    {
      s++;
      fsm_bms_skipspace(&s);
    }
  }
  return 1;
  
}

int is_bms_read(FILE *fp)
{
  char line[FSM_LINE_LEN];
  char *s;

  bms_state = 0;
  bms_have_name = 0;
  bms_have_input = 0;
  bms_have_output = 0;
  
  /* read signal transitions into the tmp_cube */
  /* this will also allocate memory for the tmp_cubes */
  
  for(;;)
  {
    s = fgets(line, FSM_LINE_LEN, fp);
    if ( s == NULL )
      break;
    if ( is_bms_line(s) == 0 )
      return 0;
  }
  
  if ( bms_state == 0 )
    return 0;
  return 1;
}

int IsValidBMSFile(const char *name)
{
  int ret = 0;
  FILE *fp;
  fp = b_fopen(name, NULL, ".bms", "r");
  if ( fp != NULL )
  {
    ret = is_bms_read(fp);
    fclose(fp);
  }
  return ret;
}

