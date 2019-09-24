/*

  codes.c
  
  
*/

#include <stdio.h>
#include <stdlib.h>

#include "cd_codes.h"

/* DCPositions */

/*-- cd_OpenDCPositions -------------------------------------------------------------*/

int* cd_OpenDCPositions ( int level)
{
  int* dc;
  
  dc = (int*) malloc (sizeof(int)*level);
  
  return dc;
}

/*-- cd_InitDCPositions -------------------------------------------------------------*/

int* cd_InitDCPositions(int* dc, int level)
{
  int i; 
  
  if(dc==NULL)
    return NULL;
    
  for(i=0; i< level; i++)
    dc[i] = i;
  
  return dc;
}

/*-- cd_CloseDCPositions -------------------------------------------------------------*/

void cd_CloseDCPositions(int *dc)
{
  if(dc!=NULL)
    free(dc);
}

/*-- cd_WriteDCPositions -------------------------------------------------------------*/

void cd_WriteDCPositions(int *dc, int level)
{
  int i;
  for(i=0; i<level; i++)
    printf("%d", dc[i]);
    
  printf("\n");
}


/*-- cd_GetNextDCPositions -------------------------------------------------------------*/

int* cd_GetNextDCPositions(int* dc, int k, int level)
{
  int alpha;
  int j;
  
  for(alpha = level-1; alpha>=0; alpha--)
  {
    if(dc[alpha] < k-level+alpha)
    {
      dc[alpha]++;
      
      for(j=alpha+1; j<=level-1; j++)
        dc[j] = dc[j-1]+1;
        
      return dc;
    }
  }
  free(dc);
  return NULL;
}

/* CODE_INFO */

/*-- cd_OpenCodeInfo -------------------------------------------------------------*/

CODE_INFO cd_OpenCodeInfo( int k, int level)
{
  CODE_INFO cInfo;
  
  if((cInfo = (CODE_INFO) malloc(sizeof(struct _code_info))) == NULL)
    return NULL;
    
  cInfo->k = k;
  cInfo->level = level;
  cInfo->nr = 0;
  
  cInfo->dc = cd_OpenDCPositions ( level);
  cInfo->dc = cd_InitDCPositions(cInfo->dc, level);
  
  return cInfo;
}

/*-- cd_CloseCodeInfo -------------------------------------------------------------*/

void cd_CloseCodeInfo(CODE_INFO cInfo)
{
  if(cInfo == NULL)
    return;
    
  cd_CloseDCPositions(cInfo->dc);
  
  free(cInfo);
  
  return;
}

/*-- cd_WriteCodeInfo -------------------------------------------------------------*/

void cd_WriteCodeInfo(CODE_INFO cInfo)
{
  int i;
  
  if(cInfo == NULL)
    return;
 
  printf("Nr of bits %d\n", cInfo->k);
  printf("Nr of DC bits %d\nDC positions", cInfo->level);
  for(i=0; i<cInfo->level; i++)
    printf("%d ", cInfo->dc[i]);
  printf("\nNumber %d\n\n", cInfo->nr);
}

/*-- cd_GetNextCodeInfo -------------------------------------------------------------*/

CODE_INFO cd_GetNextCodeInfo(CODE_INFO cInfo)
{
  if(cInfo == NULL)
    return NULL;
 
  if(cInfo->nr == (1<<(cInfo->k - cInfo->level)) - 1)
  {
    cInfo->nr = 0;
    cInfo->dc = cd_GetNextDCPositions(cInfo->dc, cInfo->k, cInfo->level);
    
    if(cInfo->dc == NULL)
    {
      cd_CloseCodeInfo(cInfo);
      return NULL;
    }
  }
  else
    cInfo->nr++;
    
  return cInfo;
}

/*-- cd_GetCodeFromCodeInfo ----------------------------------------------------------*/

int  cd_GetCodeFromCodeInfo(CODE_INFO cInfo, pinfo *pCode, dcube *code)
{
  int i, mask, s;
  
  if(cInfo == NULL)
    return 0;
  
  
  dcInSetAll(pCode, code, CUBE_IN_MASK_ZERO);
  
  for(i=0; i<cInfo->level; i++)
    dcSetIn(code, cInfo->dc[i], 3);
    
  mask = 1<<(cInfo->k - cInfo->level - 1);
  
  for(i=0; i< cInfo->k ; i++)
  {
    if(dcGetIn(code, i) == 3)
      continue;
    else
    {
      s = cInfo->nr & mask;
      mask = mask >> 1;
      
      if(s==0)
        dcSetIn(code, i, 1);
      else
        dcSetIn(code, i, 2);
        
    }
  }
  
  return 1;
}

/* 1Positions */

/*-- cd_Open1Positions -------------------------------------------------------------*/

int* cd_Open1Positions ( int nrOnes)
{
  int* ones;
  
  ones = (int*) malloc (sizeof(int)*nrOnes);
  
  return ones;
}

/*-- cd_Init1Positions -------------------------------------------------------------*/

int* cd_Init1Positions(int* ones, int nrOnes)
{
  int i; 
  
  if(ones==NULL)
    return NULL;
    
  for(i=0; i< nrOnes; i++)
    ones[i] = i;
  
  return ones;
}

/*-- cd_Close1Positions -------------------------------------------------------------*/

void cd_Close1Positions(int *ones)
{
  if(ones!=NULL)
    free(ones);
}

/*-- cd_Write1Positions -------------------------------------------------------------*/

void cd_Write1Positions(int *ones, int nrOnes)
{
  int i;
  for(i=0; i<nrOnes; i++)
    printf("%d", ones[i]);
    
  printf("\n");
}


/*-- cd_GetNext1Positions -------------------------------------------------------------*/

int* cd_GetNext1Positions(int* ones, int k, int nrOnes)
{
  int alpha;
  int j;
  
  for(alpha = nrOnes-1; alpha>=0; alpha--)
  {
    if(ones[alpha] < k-nrOnes+alpha)
    {
      ones[alpha]++;
      
      for(j=alpha+1; j<=nrOnes-1; j++)
        ones[j] = ones[j-1]+1;
        
      return ones;
    }
  }
  free(ones);
  return NULL;
}

/* OUT_CODE_INFO */

/*-- cd_OpenOutCodeInfo -------------------------------------------------------------*/

OUT_CODE_INFO cd_OpenOutCodeInfo( int k)
{
  /* returns always 0000...00 */
  
  OUT_CODE_INFO outCInfo;
  
  if((outCInfo = (OUT_CODE_INFO) malloc(sizeof(struct _out_code_info))) == NULL)
    return NULL;
    
  outCInfo->k = k;
  outCInfo->nrOnes = 0;
  
  outCInfo->ones = NULL;
    
  return outCInfo;
}

/*-- cd_CloseOutCodeInfo -------------------------------------------------------------*/

void cd_CloseOutCodeInfo(OUT_CODE_INFO outCInfo)
{
  if(outCInfo == NULL)
    return ;
    
  cd_Close1Positions(outCInfo->ones);
  
  free(outCInfo);
  
  return;
}


/*-- cd_GetNextOutCodeInfo -------------------------------------------------------------*/

OUT_CODE_INFO cd_GetNextOutCodeInfo(OUT_CODE_INFO outCInfo)
{
  if(outCInfo == NULL)
    return NULL;

  outCInfo->ones = cd_GetNext1Positions(outCInfo->ones, outCInfo->k, outCInfo->nrOnes);
  
  if(outCInfo->ones == NULL)
  {
    if(outCInfo->nrOnes < outCInfo->k)
    {
      outCInfo->nrOnes++;
      outCInfo->ones = cd_Open1Positions(outCInfo->nrOnes);
      outCInfo->ones = cd_Init1Positions(outCInfo->ones, outCInfo->nrOnes);
      
      return outCInfo;
    }
    else
    {
      cd_CloseOutCodeInfo(outCInfo);
      return NULL;
    }
  }
  else
    return outCInfo;

}

/*-- cd_GetCodeFromOutCodeInfo ----------------------------------------------------------*/

int cd_GetCodeFromOutCodeInfo(OUT_CODE_INFO outCInfo, pinfo *pCode, dcube *code)
{
  int i;
  
  if(outCInfo == NULL)
    return 0;
  
  dcOutSetAll(pCode, code, 0);
  
  for(i=0; i < outCInfo->nrOnes; i++)
    dcSetOut(code, outCInfo->ones[i], 1);
    
  return 1;
}

/*-- main ---------------------------------------------------------------*/

int main22()
{
  OUT_CODE_INFO cInfo;
  dcube code;
  pinfo pCode;

  int cnt;

  pinfoInit(&pCode);
  pinfoSetOutCnt(&pCode, 7);
  pinfoSetInCnt(&pCode, 0);

  dcInit(&pCode, &code);

  cInfo = cd_OpenOutCodeInfo(7);
  cd_GetCodeFromOutCodeInfo(cInfo, &pCode, &code );
  puts(dcToStr(&pCode, &code, "", ""));

  cnt=1;
  while(cInfo != NULL)
  {
    cInfo = cd_GetNextOutCodeInfo(cInfo);
    if(cInfo == NULL)
      break;

    cd_GetCodeFromOutCodeInfo(cInfo, &pCode , &code);
    puts(dcToStr(&pCode, &code, "", ""));
    cnt++;

  }  
  printf("There were %d codes\n", cnt); 

  dcDestroy(&code);
  pinfoDestroy(&pCode);
  return 1;
}

