/*

  cd_codes.h
  
  Copyright (C) 2019 Oliver Kraus (olikraus@gmail.com)

  This file is part of "pluc".

  "pluc" is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  "pluc" is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with DGC; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA  



*/

#ifndef _codes_H
#define _codes_H

#include <stdio.h>

#include "dcube.h"

/* CODE_INFO ...used for constraint graph */

typedef struct _code_info *CODE_INFO;

struct _code_info
{
  int k;      /* the total number of bits for the code; */
  int level;  /* the number of dc bits */
  int *dc;    /* the vector with the positions of the dc bits */
  int nr;     /* the value of the number given by the not dc bits;
               * nr is between 0..pow(2, k-level) - 1
               */
};

/* OUT_CODE_INFO ... used for my "FanIn" encoding */

typedef struct _out_code_info *OUT_CODE_INFO;

struct _out_code_info
{
  int k;        /* the total number of bits for the code; */
  int nrOnes;   /* the number of 1 bits */
  int *ones;    /* the vector with the positions of the 1 bits */
};

/* DCPositions */

int* cd_OpenDCPositions (int level);
int* cd_InitDCPositions(int* dc, int level);
void cd_CloseDCPositions(int *dc);
void cd_WriteDCPositions(int *dc, int level);
int* cd_GetNextDCPositions(int* dc, int k, int level);

/* CODE_INFO */

CODE_INFO cd_OpenCodeInfo( int k, int level);
void cd_CloseCodeInfo(CODE_INFO cInfo);
void cd_WriteCodeInfo(CODE_INFO cInfo);
CODE_INFO cd_GetNextCodeInfo(CODE_INFO cInfo);
int cd_GetCodeFromCodeInfo(CODE_INFO cInfo, pinfo *pCode, dcube *code);

/* 1Positions */

int* cd_Open1Positions (int nrOnes);
int* cd_Init1Positions(int* ones, int nrOnes);
void cd_Close1Positions(int *ones);
void cd_Write1Positions(int *ones, int nrOnes);
int* cd_GetNext1Positions(int* ones, int k, int nrOnes);

/* OUT_CODE_INFO */

OUT_CODE_INFO cd_OpenOutCodeInfo( int k);
void cd_CloseOutCodeInfo(OUT_CODE_INFO outCInfo);
OUT_CODE_INFO cd_GetNextOutCodeInfo(OUT_CODE_INFO outCInfo);
int cd_GetCodeFromOutCodeInfo(OUT_CODE_INFO outCInfo, pinfo *pCode, dcube *code);


#endif
