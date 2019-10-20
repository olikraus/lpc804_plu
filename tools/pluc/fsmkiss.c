/*

  fsmkiss.c
  
  read and write KISS files

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

#include "fsm.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "mwc.h"
#include "b_ff.h"
#include "b_sp.h"


static void fsm_skipspace(char **s) 
{  
   b_skipspace(s);
   /*
   while(**s <= ' ' && **s > '\0')
      (*s)++;
   */
}

static char *fsm_getid(char **s)
{
   return b_getid(s);
/*   
   static char t[256];
   int i = 0;
   while( **s > ' ' )
   {
      if ( i < 255 )
        t[i] = **s;
      (*s)++;
      i++;
   }
   t[i] = '\0';
   return t;
*/
}


static int fsm_ReadKISSLine(fsm_type fsm, char *s)
{
  dcube *c_cond_on = &(fsm->pi_cond->tmp[3]);
  dcube *c_cond_dc = &(fsm->pi_cond->tmp[4]);
  dcube *c_output_on = &(fsm->pi_output->tmp[3]);
  dcube *c_output_dc = &(fsm->pi_output->tmp[4]);
  char src_name[256];
  char dest_name[256];
  int edge_id;

  fsm_skipspace(&s);
  if ( *s == '\0' )
    return 1;
  if ( *s == '#' )
    return 1;
  
  s = dcSetAllByStr(fsm->pi_cond, fsm->pi_cond->in_cnt, 0, c_cond_on, c_cond_dc, s);
  if ( s == NULL )
    return 0;
  fsm_skipspace(&s);
  strcpy(src_name, fsm_getid(&s));
  fsm_skipspace(&s);
  strcpy(dest_name, fsm_getid(&s));
  fsm_skipspace(&s);
  s = dcSetAllByStr(fsm->pi_output, 0, fsm->out_cnt, c_output_on, c_output_dc, s);
  dcCopyIn(fsm->pi_output, c_output_on, c_cond_on);

  if ( c_output_on->out == NULL )
  {
    fsm_Log(fsm, "FSM: Output specification not availanle, line ignored");
    return 0;
  }
  
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
  
  return 1;
}

static int fsm_ReadKISSFP(fsm_type fsm, FILE *fp)
{
  char *t = fsm->linebuf;
  char *s;
  int is_pinfo_init = 0;
  
  fsm_Clear(fsm);
  pinfoSetOutCnt(fsm->pi_cond, 0);
  for(;;)
  {
    if ( fgets(t, FSM_LINE_LEN, fp) == NULL )
      break;
    s = t;
    fsm_skipspace(&s);

    if ( s[0] == '#' || s[0] == '\0' )
    {
      /* nothing */
    }
    else if ( s[0] == '.' && is_pinfo_init == 0 )
    {
      if ( strncmp(s, ".i ", 3) == 0 )
      {
        if ( fsm_SetInCnt(fsm, atoi(s+3)) == 0 )
          return 0;
      }
      else if ( strncmp(s, ".o ", 3) == 0 )
      {
        if ( fsm_SetOutCnt(fsm, atoi(s+3)) == 0 )
          return 0;
      }
      if ( strncmp(s, ".ilb ", 5) == 0 )
      {
        /* input label names */
        if ( b_sl_ImportByStr(fsm->input_sl, s+5, " \t\n\r", "") == 0 )
          return 0;
	if ( fsm->in_cnt < b_sl_GetCnt(fsm->input_sl) )
	{
	  if ( fsm_SetInCnt(fsm, b_sl_GetCnt(fsm->input_sl)) == 0 )
	    return 0;	  
	}
	
      }  
      else if ( strncmp(s, ".olb ", 5) == 0 )
      {
        /* this is not a valid espresso command... */
        /* output label names */
        if ( b_sl_ImportByStr(fsm->output_sl, s+5, " \t\n\r", "") == 0 )
          return 0;
	if ( fsm->out_cnt <  b_sl_GetCnt(fsm->output_sl) )
	{
	  if ( fsm_SetOutCnt(fsm, b_sl_GetCnt(fsm->output_sl)) == 0 )
	    return 0;
	}
      }
      else if ( strncmp(s, ".ob ", 4) == 0 )
      {
        /* output label names */
        if ( b_sl_ImportByStr(fsm->output_sl, s+4, " \t\n\r", "") == 0 )
          return 0;
	if ( fsm->out_cnt <  b_sl_GetCnt(fsm->output_sl) )
	{
	  if ( fsm_SetOutCnt(fsm, b_sl_GetCnt(fsm->output_sl)) == 0 )
	    return 0;
	}
      }
      else if ( strncmp(s, ".reset ", 7) == 0 )
      {
        char *t = s + 7;
        fsm_skipspace(&t);
        fsm->reset_node_id = fsm_AddNodeByName(fsm, fsm_getid(&t));
        if ( fsm->reset_node_id < 0 )
          return 0;
      }
      else if ( strncmp(s, ".r ", 3) == 0 )
      {
        char *t = s + 3;
        fsm_skipspace(&t);
        fsm->reset_node_id = fsm_AddNodeByName(fsm, fsm_getid(&t));
        if ( fsm->reset_node_id < 0 )
          return 0;
      }
    }
    else if ( s[0] == '.' )
    {
      fsm_Log(fsm, "IMPORT: KISS command ignored '%s'.", s);
      /* ignore */
    }
    else
    {
      is_pinfo_init = 1;
      if ( fsm_ReadKISSLine(fsm, s) == 0 )
      {
	fsm_Log(fsm, "IMPORT: Failed for KISS line '%s'.", s);
      }
      else
      {
	//fsm_Log(fsm, "IMPORT: Line '%s'.", s);
      }
    }
  }
  return fsm_ExpandUniversalNodeByName(fsm, "*");
}

int fsm_ReadKISS(fsm_type fsm, const char *name)
{
  int ret = 0;
  FILE *fp;
  fp = b_fopen(name, NULL, ".kiss", "r");
  if ( fp != NULL )
  {
    ret = fsm_ReadKISSFP(fsm, fp);
    fclose(fp);
  }
  return ret;
}

static int fsm_WriteKISSFP(fsm_type fsm, FILE *fp)
{
  int edge_id;
  int i, j;
  dclist cl;
  
  fprintf(fp, ".i %d\n", fsm_GetInCnt(fsm));
  fprintf(fp, ".o %d\n", fsm_GetOutCnt(fsm));

  if ( fsm->input_sl != NULL )
  {
    if ( fprintf(fp, ".ilb ") < 0 ) return 0;
    if ( b_sl_ExportToFP(fsm->input_sl, fp, " ", "\n") == 0 ) return 0;
  }
  if ( fsm->output_sl != NULL )
  {
    if ( fprintf(fp, ".ob ") < 0 ) return 0;
    if ( b_sl_ExportToFP(fsm->output_sl, fp, " ", "\n") == 0 ) return 0;
  }
  
  if ( fsm->reset_node_id >= 0 )
    fprintf(fp, ".reset %s\n", fsm_GetNodeNameStr(fsm, fsm->reset_node_id));
  
  edge_id = -1;
  while(fsm_LoopEdges(fsm, &edge_id))
  {
    /*
    cl = fsm_GetEdgeCondition(fsm, edge_id);
    for( i = 0; i < dclCnt(cl); i++ )
    {
      fprintf(fp, "%s ", dcToStr(fsm_GetConditionPINFO(fsm), dclGet(cl, i), "", ""));
      fprintf(fp, "%s ", fsm_GetNodeNameStr(fsm, fsm_GetEdgeSrcNode(fsm, edge_id)));
      fprintf(fp, "%s", fsm_GetNodeNameStr(fsm, fsm_GetEdgeDestNode(fsm, edge_id)));
      if ( fsm->out_cnt > 0 )
        fprintf(fp, " ");
      for( j = 0; j < fsm->out_cnt; j++ )
        fprintf(fp, "0");
      fprintf(fp, "\n");
    }
    */
    cl = fsm_GetEdgeOutput(fsm, edge_id);
    for( i = 0; i < dclCnt(cl); i++ )
    {
      for( j = 0; j < fsm->pi_output->in_cnt; j++ )
        fprintf(fp, "%c", "x01-"[dcGetIn(dclGet(cl, i), j)]);
      fprintf(fp, " %s ", fsm_GetNodeNameStr(fsm, fsm_GetEdgeSrcNode(fsm, edge_id)));
      fprintf(fp, "%s", fsm_GetNodeNameStr(fsm, fsm_GetEdgeDestNode(fsm, edge_id)));
      if ( fsm->out_cnt > 0 )
        fprintf(fp, " ");
      for( j = 0; j < fsm->out_cnt; j++ )
        fprintf(fp, "%c", "01"[dcGetOut(dclGet(cl, i), j)]);
      fprintf(fp, "\n");
    }
  }
  return 1;
}

int fsm_WriteKISS(fsm_type fsm, const char *name)
{
  int ret = 0;
  FILE *fp;
  fp = fopen(name, "w");
  if ( fp != NULL )
  {
    ret = fsm_WriteKISSFP(fsm, fp);
    fclose(fp);
  }
  return ret;
}

/*---------------------------------------------------------------------------*/

static int is_kiss_line(char *s)
{
  char src_name[256];
  char dest_name[256];

  fsm_skipspace(&s);
  if ( *s == '\0' )
    return 1;
  if ( *s == '#' )
    return 1;
    
  while( *s=='1' || *s=='0' || *s=='-' )
    s++;
  
  fsm_skipspace(&s);
  if ( *s == '\0' )
    return 0;
  strcpy(src_name, fsm_getid(&s));
  if ( strlen(src_name) == 0 )
    return 0;
  fsm_skipspace(&s);
  if ( *s == '\0' )
    return 0;
  strcpy(dest_name, fsm_getid(&s));
  if ( strlen(dest_name) == 0 )
    return 0;
  fsm_skipspace(&s);
  if ( *s == '\0' )
    return 1;       /* 25 okt 2001 ok: kiss file has no output... */

  while( *s=='1' || *s=='0' || *s=='-' ) /* 25 okt 2001 ok: consider dc */
    s++;
  fsm_skipspace(&s);
  if ( *s != '\0' )
    return 0;
    
  return 1;
}

static int is_kiss_read(FILE *fp)
{
  static char t[FSM_LINE_LEN];
  char *s;
  int is_pinfo_init = 0;
  
  for(;;)
  {
    if ( fgets(t, FSM_LINE_LEN, fp) == NULL )
      break;
    s = t;
    fsm_skipspace(&s);

    if ( s[0] == '#' || s[0] == '\0' )
    {
      /* nothing */
    }
    else if ( s[0] == '.' && is_pinfo_init == 0 )
    {
      if ( strncmp(s, ".i ", 3) == 0 )
      {
      }
      else if ( strncmp(s, ".o ", 3) == 0 )
      {
      }
      else if ( strncmp(s, ".reset ", 7) == 0 )
      {
        char *t = s + 7;
        fsm_skipspace(&t);
      }
      else if ( strncmp(s, ".r ", 3) == 0 )
      {
        char *t = s + 3;
        fsm_skipspace(&t);
      }
    }
    else if ( s[0] == '.' )
    {
      /* ignore */
    }
    else
    {
      is_pinfo_init = 1;
      if ( is_kiss_line(s) == 0 )
        return 0;
    }
  }
  return 1;
}

int IsValidKISSFile(const char *name)
{
  int ret = 0;
  FILE *fp;
  fp = b_fopen(name, NULL, ".kiss", "r");
  if ( fp != NULL )
  {
    ret = is_kiss_read(fp);
    fclose(fp);
  }
  return ret;
}
