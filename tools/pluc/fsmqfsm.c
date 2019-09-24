/*

  fsmqfsm.c
  
  read qfsm (xml) files

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

//#include "config.h"
#include <string.h>
#ifdef HAVE_LIBXML  
#include <libxml/parser.h>
#include <libxml/xpath.h>
#endif
#include "fsm.h"

#ifdef HAVE_LIBXML  

void my_empty_xml_error_func(void *ctx, const char *msg, ...)
{
  /* do nothing */
}

static int copy_content(char *buf, int max, xmlDocPtr doc, xmlNodePtr n, char *name)
{
  xmlXPathObjectPtr res;
  xmlXPathContextPtr ctxt;
  xmlNodePtr nn;
  int ret = 0;

  ctxt = xmlXPathNewContext(doc);
  if ( ctxt == NULL )
    return 0;
  ctxt->node = n;
  res = xmlXPathEvalExpression(name, ctxt);
  if ( res != NULL )
  {
    if ( res->type == XPATH_NODESET )
    {
      if ( xmlXPathNodeSetGetLength(res->nodesetval) >= 1 )
      {
        nn = xmlXPathNodeSetItem(res->nodesetval, 0);
        strncpy(buf, xmlNodeListGetString(doc, nn->children, 1), max);
        buf[max-1] = '\0';
        ret = 1;
      }
    }
    xmlXPathFreeObject(res);
  }
  xmlXPathFreeContext(ctxt);
  return ret;
}

/* xmlGetProp(n, "code"); */

static int fsm_qfsm_reset_state(fsm_type fsm, xmlDocPtr doc)
{
  xmlXPathObjectPtr res;
  xmlXPathContextPtr ctxt;
  int i;
  xmlNodePtr n;
  char *name;
  int ret = 0;

  ctxt = xmlXPathNewContext(doc);
  if ( ctxt == NULL )
    return 0;
  ctxt->node = xmlDocGetRootElement(doc);
  res = xmlXPathEvalExpression("/qfsmproject/machine/state", ctxt);
  if ( res != NULL )
  {
    if ( res->type == XPATH_NODESET )
    {
      if ( xmlXPathNodeSetGetLength(res->nodesetval) >= 1 )
      {
        n = xmlXPathNodeSetItem(res->nodesetval, 0);
        if ( n != NULL )
        {
          name = xmlGetProp(n, "code");
          if ( name != NULL )
          {
            if( fsm_SetResetNodeIdByName(fsm, name) != 0 )
            {
              ret = 1;
              fsm_LogLev(fsm, 2, "QFSM import: Machine will use state '%s' as reset state.", name);
            }
          }
        }
      }
    }
    xmlXPathFreeObject(res);
  }
  xmlXPathFreeContext(ctxt);
  return ret;
}

#define STR_LEN 512
static int fsm_qfsm_do_transition(fsm_type fsm, xmlDocPtr doc, xmlNodePtr n)
{
  char src_name[STR_LEN];
  char dest_name[STR_LEN];
  char inputs[STR_LEN*2+2];
  char outputs[STR_LEN];
  int i, cnt;
  if ( copy_content(src_name, STR_LEN, doc, n, "from") == 0 ) return 0;
  if ( copy_content(dest_name, STR_LEN, doc, n, "to") == 0 ) return 0;
  if ( copy_content(inputs, STR_LEN, doc, n, "inputs") == 0 ) return 0;
  if ( copy_content(outputs, STR_LEN, doc, n, "outputs") == 0 ) return 0;
  
  if ( strlen(inputs) != fsm_GetInCnt(fsm) )
  {
    if ( fsm_SetInCnt(fsm, strlen(inputs)) == 0 )
      return 0;
    fsm_LogLev(fsm, 2, "QFSM import: Number of inputs is '%d'.", strlen(inputs));
  }
  
  if ( strlen(outputs) != fsm_GetOutCnt(fsm) )
  {
    if ( fsm_SetOutCnt(fsm, strlen(outputs)) == 0 )
      return 0;
    fsm_LogLev(fsm, 2, "QFSM import: Number of outputs is '%d'.", strlen(outputs));
  }

  cnt = strlen(inputs);
  for( i = 0; i < cnt; i++ )
    if ( inputs[i] != '0' && inputs[i] != '1' )
      inputs[i] = '-';

  strcat(inputs, " ");
  strcat(inputs, outputs);
  
  {
    /* from fsmkiss.c */
    dcube *c_cond_on = &(fsm->pi_cond->tmp[3]);
    dcube *c_cond_dc = &(fsm->pi_cond->tmp[4]);
    dcube *c_output_on = &(fsm->pi_output->tmp[3]);
    dcube *c_output_dc = &(fsm->pi_output->tmp[4]);
    int edge_id;
    if ( dcSetAllByStr(fsm->pi_cond, fsm->pi_cond->in_cnt, 0, 
            c_cond_on, c_cond_dc, inputs) == NULL )
      return 0;
    if ( dcSetAllByStr(fsm->pi_output, 0, fsm->out_cnt, 
            c_output_on, c_output_dc, outputs) == NULL )
      return 0;
    dcCopyIn(fsm->pi_output, c_output_on, c_cond_on);
    
    /* last bit is always 1, because we need to know all conditions, including */
    /* those conditions with zero output */
    dcSetOut(c_output_on, fsm->out_cnt, 1);
    
    edge_id = fsm_ConnectByName(fsm, src_name, dest_name, 1);
    if ( edge_id < 0 )
      return 0;
    if ( fsm_AddEdgeOutputCube(fsm, edge_id, c_output_on) < 0 )
      return 0;
    if ( fsm_AddEdgeConditionCube(fsm, edge_id, c_cond_on) < 0 )
      return 0;
  }
  
  return 1;
}

static int fsm_qfsm_loop_transitions(fsm_type fsm, xmlDocPtr doc)
{
  xmlXPathObjectPtr res;
  xmlXPathContextPtr ctxt;
  int i;
  xmlNodePtr n;

  ctxt = xmlXPathNewContext(doc);
  if ( ctxt == NULL )
    return 0;
  ctxt->node = xmlDocGetRootElement(doc);
  res = xmlXPathEvalExpression("/qfsmproject/machine/transition", ctxt);
  if ( res != NULL )
  {
    if ( res->type == XPATH_NODESET )
    {
      for( i = 0; i < xmlXPathNodeSetGetLength(res->nodesetval); i++ )
      {
        n = xmlXPathNodeSetItem(res->nodesetval, i);
        if ( fsm_qfsm_do_transition(fsm, doc, n) == 0 )
          return 0;
      }
    }
    xmlXPathFreeObject(res);
  }
  xmlXPathFreeContext(ctxt);
  
  return 1;
}

int fsm_IsValidQFSMFile(const char *name)
{
  xmlDocPtr doc;
  xmlXPathObjectPtr res;
  xmlXPathContextPtr ctxt;
  int ret = 0;
  
  xmlSetGenericErrorFunc(NULL, my_empty_xml_error_func);
  
  doc = xmlParseFile(name);
  if ( doc != NULL )
  {
    ctxt = xmlXPathNewContext(doc);
    if ( ctxt != NULL )
    {
      ctxt->node = xmlDocGetRootElement(doc);
      if ( ctxt->node != NULL )
      {
        res = xmlXPathEvalExpression("/qfsmproject/machine", ctxt);
        if ( res != NULL )
        {
          if ( res->type == XPATH_NODESET )
          {
            ret = 1;
          }
          xmlXPathFreeObject(res);
        }
      }
      xmlXPathFreeContext(ctxt);
    }
    xmlFreeDoc(doc);
  }
  return ret;
}

int fsm_ReadQFSM(fsm_type fsm, const char *name)
{
  xmlDocPtr doc;
  int ret = 1;
  
  fsm_Clear(fsm);
  doc = xmlParseFile(name);
  if ( doc == NULL )
    return 0;
  ret &= fsm_qfsm_loop_transitions(fsm, doc);
  ret &= fsm_qfsm_reset_state(fsm, doc);
  xmlFreeDoc(doc);
  return ret;
}

#else

int fsm_IsValidQFSMFile(const char *name)
{
  return 0;
}

int fsm_ReadQFSM(fsm_type fsm, const char *name)
{
  return 0;
}

#endif
