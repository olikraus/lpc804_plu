/*

  dcubehf.h

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

#ifndef _DCUBEHF_H
#define _DCUBEHF_H

#include "b_set.h"
#include "cubedefs.h"

/* privileged cube */
struct _pcube_struct
{
  dcube sc;   /* start cube */
  dcube tc;   /* transition cube */
};
typedef struct _pcube_struct pcube;

/* hazard free problem */
struct _hfp_struct
{
  pinfo *pi;
  b_set_type pclist;  /* list of privileged cubes */
  dclist cl_req_on;
  dclist cl_req_off;
  dclist cl_dc;
  
  dclist cl_on;
  
  void (*log_fn)(void *log_data, const char *fmt, va_list va);
  void *log_data;

  void (*err_fn)(void *err_data, const char *fmt, va_list va);
  void *err_data;
  
};
typedef struct _hfp_struct *hfp_type;

pcube *pcOpen(pinfo *pi);
void pcClose(pcube *p);
hfp_type hfp_Open(int in, int out);
void hfp_Close(hfp_type hfp);

void hfp_SetLogCB(hfp_type hfp, void log_cb(void *log_data, const char *fmt, va_list va), void *log_data);
void hfp_Log(hfp_type hfp, const char *fmt, ...);

void hfp_SetErrorCB(hfp_type hfp, void err_cb(void *err_data, const char *fmt, va_list va), void *err_data);
void hfp_Error(hfp_type hfp, const char *fmt, ...);

int hfp_AddPC(hfp_type hfp, pcube *pc);
pcube *hfp_GetPC(hfp_type hfp, int id);
int hfp_AddStartTransitionCube(hfp_type hfp, dcube *sc, dcube *tc);
int hfp_AddStartEndCube(hfp_type hfp, dcube *sc, dcube *ec);

int hfp_AddRequiredOnCube(hfp_type hfp, dcube *c);
int hfp_AddRequiredOffCube(hfp_type hfp, dcube *c);
int hfp_AddRequiredOnCubeList(hfp_type hfp, dclist cl);
int hfp_AddRequiredOffCubeList(hfp_type hfp, dclist cl);

int hfp_AddStartTransitionCubes(hfp_type hfp, dcube *c, dclist cl_all);

int hfp_AddFromToTransition(hfp_type hfp, dcube *from, dcube *to);


int hfp_Minimize(hfp_type hfp, int is_greedy, int is_literal);

int dclIsHazardfreeFunction(pinfo *pi, dclist cl, dcube *start, dcube *end, int out);

#endif /* _DCUBEHF_H */

