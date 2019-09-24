/*

  code_list.c
  
  Description of the list of constraints and assigned codes
  
*/

#include <stdio.h>
#include <stdlib.h>

#ifdef WITH_GETRUSAGE
#include <sys/types.h>
#include <sys/time.h>
#include <sys/resource.h>
#endif

#include <math.h>

#include "fsm.h"

#include "code_list.h"
#include "extra.h"
#include "cd_codes.h"

/*-- code_OpenCodeList -------------------------------------------------------------*/

CODE_LIST code_OpenCodeList()
{
  return NULL;
}

/*-- code_AddPairToList ------------------------------------------------------------*/

CODE_LIST code_AddPairToList(CODE_LIST codeList, dcube *constr, dcube *code, pinfo *pConstr, pinfo *pCode)

{
  /* adds an element at the beginning of the list and returns the new list 
   */
  CODE_LIST tmp;

  if((tmp = (CODE_LIST) malloc(sizeof(struct _code_list))) == NULL)
    return NULL;
    
  if((tmp->constr = (dcube *) malloc(sizeof(dcube))) == NULL)
    return NULL;
    
  if((tmp->code = (dcube *) malloc(sizeof(dcube))) == NULL)
    return NULL;
  
  if(!dcInit(pConstr, tmp->constr))
    return NULL;
    
   if(!dcInit(pCode, tmp->code))
    return NULL;
 
  dcCopy(pConstr, tmp->constr, constr);
  dcCopy(pCode, tmp->code, code);
  
  tmp->next = codeList;
  
  return (CODE_LIST) tmp;
  
}

/*-- code_DeleteTopPairFromList ------------------------------------------------------------*/

CODE_LIST code_DeleteTopPairFromList(CODE_LIST codeList)
{
  /* deletes the first element from the list and returns the resulting list 
   */
   
  CODE_LIST tmp;
  if(codeList == NULL) return NULL;
  
  tmp = codeList->next;
  
  dcDestroy(codeList->constr);
  dcDestroy(codeList->code);
  
  free(codeList->constr);
  free(codeList->code);
  
  free(codeList);
  
  return tmp;
}

/*-- code_DeleteConstrFromList ------------------------------------------------------------*/

CODE_LIST code_DeleteConstrFromList (CODE_LIST codeList, dcube *constr, pinfo *pConstr)
{
  /* deletes the element with the given constraint from the list and returns the 
   * resulting list; if the element is not in the list, returns the unmodified list
   */
   
  CODE_LIST tmp, prev;
  
  if(codeList == NULL)
    return NULL;
    
  /* if it is the first in the list */
  if(dcIsEqual(pConstr, constr, codeList->constr))
  {
    tmp = codeList->next;
    
    dcDestroy(codeList->constr);
    dcDestroy(codeList->code);
    
    free(codeList->constr);
    free(codeList->code);
    
    free (codeList);
    return tmp;
  }
  
  /* it is sure not the first one */
  prev = codeList;
  for(tmp = codeList->next; tmp != NULL; tmp = tmp->next)
  {
    if(dcIsEqual(pConstr, constr, tmp->constr))
    {
      prev->next = tmp->next;
      dcDestroy(tmp->constr);
      dcDestroy(tmp->code);
      
      free(tmp->constr);
      free(tmp->code);
      
      free (tmp);
      return codeList;
    }
    else
      prev = prev->next;
  }
  
  /* if I am here, constraint was not in the list */
  return codeList;
}

/*-- code_CloseCodeList ----------------------------------------------------------*/

void code_CloseCodeList(CODE_LIST codeList)
{
  while(codeList != NULL)
    codeList = code_DeleteTopPairFromList(codeList);
}

/*-- code_WriteCodeList ---------------------------------------------------------------*/

void code_WriteCodeList(CODE_LIST codeList, pinfo *pConstr, pinfo *pCode)
{
  CODE_LIST tmp;
  
  for(tmp = codeList; tmp != NULL; tmp = tmp->next)
  {
    printf("( ");
    printf("%s", dcToStr(pConstr, tmp->constr, "", ""));
    printf(", ");
    printf("%s", dcToStr(pCode, tmp->code, "", ""));
    printf(")\n");
  }
}


/*-- code_FindConstrInList ---------------------------------------------------------------*/

int code_FindConstrInList(CODE_LIST codeList, dcube *constr, dcube *code, pinfo *pConstr, pinfo *pCode)
{
  /* searches for a given constraint and if it finds it gives back the corresponding code, 
   * returning 1;
   * returns 0 if the constraint is not in the list
   */
  CODE_LIST tmp;
  
  if(codeList == NULL)
    return 0;
    
  for(tmp = codeList; tmp != NULL; tmp = tmp->next)
  {
    if(dcIsEqual(pConstr, constr, tmp->constr))
    {
      dcCopy(pCode, code, tmp->code);
      return 1;  
    }
  }
  return 0;

}

/*-- code_FindCodeInList ---------------------------------------------------------------*/

int code_FindCodeInList(CODE_LIST codeList, dcube *code, pinfo *pCode)
{
  /* searches for a given code and if it finds it returns 1;
   * returns 0 if the code is not in the list
   */

  CODE_LIST tmp;
  
  if(codeList == NULL)
    return 0;

  

  for(tmp = codeList; tmp != NULL; tmp = tmp->next)
  {
    if(dcIsEqual(pCode, code, tmp->code))
      return 1;
  }

  return 0;
}


int code_CopyCodesAux(CODE_LIST cList, fsm_type fsm, INDEX_TABLE tab, pinfo * pConstr, pinfo *pCode, int k)
{
  CODE_LIST tmp;
  int node_id;
  pinfo *pi = fsm_GetCodePINFO(fsm);
  
  for(tmp = cList; tmp != NULL; tmp = tmp->next)
  {
    if(extra_dcOutOneCnt(tmp->constr, pConstr) == 1)
    {
      node_id = index_GetNodeId(tab, extra_dcOutGetOnePos(tmp->constr, pConstr));
      
      dcCopyInToOut(pi, fsm_GetNodeCode(fsm, node_id), 0, pCode, tmp->code);


    }
  }
  return 1;
}

CODE_LIST code_CopyCodeList(CODE_LIST codeList, pinfo *pConstr, pinfo *pCode)
{
  /* makes a copy */
  CODE_LIST tmp;
  CODE_LIST newList;
  
  newList = code_OpenCodeList();
  
  for(tmp = codeList; tmp; tmp = tmp->next)
  {
    newList = code_AddPairToList(newList, tmp->constr, tmp->code, pConstr, pCode);
  }
  
  return newList;
}


/* CODE_LIST_VECTOR */

/*-- code_OpenCodeListVector ----------------------------------------------------------*/

CODE_LIST_VECTOR code_OpenCodeListVector(int n)
{
  CODE_LIST_VECTOR clv;
  
  int i;
  
  if((clv = (CODE_LIST_VECTOR) malloc(sizeof(struct _code_list_vector))) == NULL)
    return NULL;

  clv->nr_levels = n;
  
  if ( (clv->lists = (CODE_LIST*) malloc(n*sizeof(CODE_LIST)) ) == NULL)
    return NULL;
  
  for(i=0; i< n; i++)
    clv->lists[i] = code_OpenCodeList();
    
  return clv;
}

/*-- code_CloseCodeListVector ---------------------------------------------------------*/

void code_CloseCodeListVector(CODE_LIST_VECTOR clv)
{
  int i;
  
  if(clv == NULL)
    return;
    
  for(i=0; i< clv->nr_levels; i++)
    code_CloseCodeList(clv->lists[i]);
    
  free(clv->lists);
  free(clv);
  
  return;
}

/*-- code_AddPairToListVector ---------------------------------------------------------*/

CODE_LIST_VECTOR code_AddPairToListVector(CODE_LIST_VECTOR clv, int level, dcube *constr, dcube *code, 
    pinfo *pConstr, pinfo *pCode)
{
  if(level >= clv->nr_levels)
    clv = code_ReallocCodeListVector(clv, level+1);
    
  if( (clv->lists[level] = code_AddPairToList(clv->lists[level], constr, code, pConstr, pCode)) == NULL)
    return NULL;
  
  return (CODE_LIST_VECTOR) clv;    
}

/*-- code_WriteCodeListVector ----------------------------------------------------------*/

void code_WriteCodeListVector(CODE_LIST_VECTOR clv, pinfo *pConstr, pinfo *pCode)
{
  int i;
  if(clv == NULL)
    return;
 
  printf("\n----BEGIN CODE LIST VECTOR\n");
  for(i=0; i< clv->nr_levels; i++)
  {
    printf("Level %d\n", i);
    code_WriteCodeList(clv->lists[i], pConstr, pCode);
  }
  printf("\n----END CODE LIST VECTOR\n");
 
}


/*-- code_ReallocCodeListVector --------------------------------------------------------*/

CODE_LIST_VECTOR code_ReallocCodeListVector(CODE_LIST_VECTOR clv, int new_n)
{
  int n, i;
  n = clv->nr_levels;
  clv->nr_levels = new_n;
  if (( clv->lists = (CODE_LIST*) realloc(clv->lists, new_n* sizeof(CODE_LIST)) ) == NULL)
    return NULL;
  
  for(i=n; i< new_n; i++)
    clv->lists[i] = code_OpenCodeList();
  
  return clv;
}

/*-- code_FindConstrOnLevelInListVector --------------------------------------------------------*/

int code_FindConstrOnLevelInListVector(CODE_LIST_VECTOR clv, int level, dcube *constr, pinfo *pConstr, dcube *code, pinfo *pCode)
{
  /* searches for a constraint on a specified level in the list vector
   * returns 1 if it has found and gives back the code and 0 if it has not found 
   */
  
  if(clv == NULL)
    return 0;


  if(level >= clv->nr_levels)
    return 0;

  return code_FindConstrInList(clv->lists[level], constr, code, pConstr, pCode);
}


/*-- code_FindConstrInListVector --------------------------------------------------------*/

int code_FindConstrInListVector(CODE_LIST_VECTOR clv,  dcube *constr, pinfo *pConstr, dcube *code, pinfo *pCode)
{
  /* searches for a constraint in all lists of the list vector
   * returns 1 if it has found and gives back the code and 0 if it has not found 
   */
  int i;
  
  if(clv == NULL)
    return 0;

  for(i=0; i<clv->nr_levels; i++)
    if(code_FindConstrInList(clv->lists[i], constr, code, pConstr, pCode))
      return 1;
      
  return 0;
}

/*-- code_FindCodeInListVector --------------------------------------------------------*/

int code_FindCodeInListVector(CODE_LIST_VECTOR clv,  dcube *code, pinfo *pCode)
{
  /* searches a given code and returns 1 if the code exists in the list vector
   * and 0 otherwise
   */
  int i;
   
  if(clv == NULL)
    return 0;
  
  for(i=0; i < clv->nr_levels; i++)
    if(code_FindCodeInList(clv->lists[i], code, pCode))
      return 1;
      
  return 0;
}  


/*-- code_DeleteNodeFromListVector --------------------------------------------------------*/

CODE_LIST_VECTOR code_DeleteNodeFromListVector(CODE_LIST_VECTOR clv, int level, CONSTR_GRAPH cGraph, 
  int cNode, pinfo *pConstr)
{
  KID_LIST tmp;
  CODE_LIST_VECTOR aux;
  
  if(clv == NULL)
    return NULL;
    
  if(level >= clv->nr_levels) 
    return clv;
    
  clv->lists[level] = code_DeleteConstrFromList(clv->lists[level], &(cGraph->nodes[cNode]->constr), pConstr);
  
  aux = clv;
  for(tmp = cGraph->nodes[cNode]->kids; tmp != NULL; tmp = tmp->next)
  {
    aux =  code_DeleteNodeFromListVector(aux, level+1, cGraph, tmp->id, pConstr);
  }
  return (CODE_LIST_VECTOR) aux;
}

/*-- code_IsCompatible --------------------------------------------------------*/

int code_IsCompatible(CODE_LIST_VECTOR clv,  dcube *constr1, dcube* constr2, dcube *code1, dcube * code2, pinfo * pConstr, pinfo *pCode)
{
  /* returns 1 if the two codes from the two constraints are compatible */
  
  dcube intersConstr;
  dcube intersCode;
  dcube listCode;
  
/*  printf("constr1 = %s\n", dcToStr(pConstr, constr1, "", ""));
  printf("constr2 = %s\n", dcToStr(pConstr, constr2, "", ""));
  printf("code1 = %s\n", dcToStr(pCode, code1, "", ""));
  printf("code2 = %s\n", dcToStr(pCode, code2, "", ""));
  
  */
  dcInit(pConstr, &intersConstr);
  dcInit(pCode, &intersCode);
  dcInit(pCode, &listCode);
  
      
  if(dcIntersection(pConstr, &intersConstr, constr1, constr2) == 0)
  {
    /* constraints don't intersect; the codes musn't intersect either */
    if(dcIntersection(pCode, &intersCode, code1, code2) == 0)
    {
      dcDestroy(&intersConstr);
      dcDestroy(&intersCode);
      dcDestroy(&listCode);

      return 1;
    }
    else
    {
      /*printf("Not compatible because constraints don't intersect and codes intersect\n");*/
      dcDestroy(&intersConstr);
      dcDestroy(&intersCode);
      dcDestroy(&listCode);

      return 0;
    }
  }
  else
  {
    /* constraints intersect; codes must either intersect */
    /*printf("intersConstr %s\n", dcToStr(pConstr, &intersConstr, "", ""));*/
    
    if(dcIntersection(pCode, &intersCode, code1, code2) == 0)
    {
      /*printf("Not compatible because constraints intersect and codes don't\n");*/
      dcDestroy(&intersConstr);
      dcDestroy(&intersCode);
      dcDestroy(&listCode);

      return 0;
    }
    else
    {
      /*printf("intersCode %s\n", dcToStr(pCode, &intersCode, "", ""));*/

     /* find the code of the intersesction - this already exists in codeList */
     if (code_FindConstrInListVector(clv,  &intersConstr, pConstr, &listCode, pCode)==0)
      {
        dcDestroy(&intersConstr);
        dcDestroy(&intersCode);
        dcDestroy(&listCode);
        
        return 1;
      }
      else 
      {
        /*printf("listCode %s\n", dcToStr(pCode, &listCode, "", ""));*/

        if(dcIsSubSet(pCode, &intersCode, &listCode)) /* !!! */
        /*if(dcIsEqual(pCode, &intersCode, &listCode))*/
        {
          dcDestroy(&intersConstr);
          dcDestroy(&intersCode);
          dcDestroy(&listCode);

          return 1;
        }

        else
        {
          /*printf("Not compatible because intersCode is not a subset of the code already assigned\n");*/
          dcDestroy(&intersConstr);
          dcDestroy(&intersCode);
          dcDestroy(&listCode);
          return 0;
        }
      }
    }
  }
}

/* --------- code_CopyCodes --------------------------------------*/

int code_CopyCodes(CODE_LIST_VECTOR codeList, fsm_type fsm, INDEX_TABLE tab, pinfo *pConstr, pinfo *pCode)
{
  int i, k;
  k = pinfoGetInCnt(pCode);
  
  if(fsm_SetCodeWidth(fsm, k, FSM_CODE_DFF_EXTRA_OUTPUT)== 0)
    return 0;
    
  for(i=0; i< codeList->nr_levels; i++)
    if(code_CopyCodesAux(codeList->lists[i], fsm, tab, pConstr, pCode, k) == 0)
      return 0;
    
  return 1;
}

/* --------- code_SetSatisfiedConstr --------------------------------------*/

int code_SetSatisfiedConstr(MIN_SYMB_LIST minList, CODE_LIST_VECTOR codeList, pinfo *pConstr)
{
  /* searches the constraints list and for each term sets is_valid=1 if the constraint was satisfied 
   * or is_valid = 0 if not;
   * returns the number of satisfied constraints
   */
   
  CODE_LIST tmp;
  int cnt = 0, i;

  min_InvalidateAll(minList);
    
  for(i=0; i< codeList->nr_levels; i++)
    for(tmp = codeList->lists[i]; tmp; tmp = tmp->next)
    {
      if(extra_dcOutOneCnt(tmp->constr, pConstr) > 1)
      {
        cnt++;
        min_SetValid(minList, tmp->constr, pConstr);
      }
    }
  
  return cnt;
}


CODE_LIST_VECTOR code_CopyCodeListVector(CODE_LIST_VECTOR clv, pinfo *pConstr, pinfo *pCode)
{
  CODE_LIST_VECTOR newClv;
  int i;
  
  if(!clv) return NULL;
  
  newClv = code_OpenCodeListVector(clv->nr_levels);
  
  for(i=0; i < newClv->nr_levels; i++)
  {
    newClv->lists[i] = code_CopyCodeList(clv->lists[i], pConstr, pCode);
  }
  
  return newClv;
}

/* assigning codes to a constraint graph */
/*-- code_AssignCodes -----------------------------------------------------------*/

int code_AssignCodes(CONSTR_GRAPH cGraph, CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode, KID_LIST *bad)
{
  KID_LIST kids;
  int OK;
  
  OK = 1;
  for(kids = cGraph->nodes[0]->kids; kids != NULL && OK; kids = kids->next)
  {
    if( (code_AssignCodeToPrimaryConstr(cGraph, kids->id, 0, codeList, pConstr, pCode) ) == 0)
    {
      OK = 0;
      *bad = kids;
    }
  }
  
  return OK;
}

/*-- code_AssignCodeToPrimaryConstr -----------------------------------------------------------*/

int code_AssignCodeToPrimaryConstr(CONSTR_GRAPH cGraph, int primNode, int listLevel, CODE_LIST_VECTOR codeList,
  pinfo *pConstr, pinfo *pCode)
{
  int OK, exist;
  dcube primConstr;
  dcube code;
  CODE_INFO cInfo;
  int k;      /* number of bits for primConstr; */
  int level, kidLevel;  /* number of DC bits for primConstr */
  int cardConstr, kidCard; /* constraint cardinality */   
  //int nrStates;
  CODE_LIST tmp;
  KID_LIST kids, kids1;
  int *user_data;
#ifdef WITH_GETRUSAGE
  int start, end, time;
  struct rusage r_time;
#endif

  
#ifdef WITH_GETRUSAGE
  /* take the beginning time */
  if( getrusage(RUSAGE_SELF, &r_time) != 0)
  {
    return 0;
  }
  start = (r_time.ru_utime.tv_sec * 1000000) + (r_time.ru_utime.tv_usec);
#endif

  
  user_data = (int*) malloc(sizeof(int));
  *user_data = 0;
  
  //nrStates = pinfoGetOutCnt(pConstr);
  
  dcInit(pConstr, &primConstr);
  dcInit(pCode, &code);
  
  dcCopy(pConstr, &primConstr,   &(cGraph->nodes[primNode]->constr));
  /*fprintf(f, "The current primary constraint %s\n", dcToStr(pConstr, &primConstr, "", ""));*/
  
  if (code_FindConstrInListVector(codeList, &primConstr, pConstr, &code, pCode))
  {
    dcDestroy(&primConstr);
    dcDestroy(&code);
    return 1;
  }

  
  k = pinfoGetInCnt(pCode);
  cardConstr = extra_dcOutOneCnt(&primConstr, pConstr);
  level = (int) ceil( log((double)cardConstr)/log(2.0));

  if(k == level)
  {
     dcDestroy(&primConstr);
     dcDestroy(&code);
     free(user_data);
     return 0;
  }
   
  exist = 0;
  for(kids = cGraph->nodes[primNode]->kids; kids != NULL && !exist; kids = kids->next)
  {
    kidCard = extra_dcOutOneCnt(&(cGraph->nodes[kids->id]->constr), pConstr);
    kidLevel = (int) ceil( log((double)kidCard)/log(2.0));
    if(kidLevel == level)
      exist = 1;
  }
  if( exist)
  {
    if(level < k-1)
      level++;
    else
    {
       dcDestroy(&primConstr);
       dcDestroy(&code);
       free(user_data);
       return 0;
    }
  }
   
  cInfo = cd_OpenCodeInfo(k,level);
  
  
  do
  {
   
    OK = 1;
    
    /* generate a new code */
    
#ifdef WITH_GETRUSAGE
    /* take the ending time */
    if( getrusage(RUSAGE_SELF, &r_time) != 0)
    {
      return 0;
    }
    end = (r_time.ru_utime.tv_sec * 1000000) + (r_time.ru_utime.tv_usec);
    time = end - start;
#endif
/*
 *     if(time > 2000) 
 *     {
 * 
 *       dcDestroy(&primConstr);
 *       dcDestroy(&code);
 *       free(user_data);
 *       return 0;
 *     }
 * 
 */
    cd_GetCodeFromCodeInfo(cInfo, pCode,   &code );
    /*fprintf(f, "The code: %s\n", dcToStr(pCode, &code, "", ""));*/
    
    /* see if it wasn't already assigned */
    
    if(code_FindCodeInListVector(codeList, &code, pCode))
    {
      /*printf("Not compatible because it already exists\n");*/
      OK = 0;
    }
    
    /* see if it is compatible with all other codes from all the levels 
     * perhaps easier for a primary constraint
     */
    if (OK)
    {
      for(tmp = codeList->lists[0]; tmp != NULL && OK; tmp = tmp->next)
        if(!code_IsCompatible(codeList, &primConstr, tmp->constr, &code, tmp->code, pConstr, pCode))
        {
          OK = 0;
        }
    }
    
    if(OK)
    {
      for(kids = cGraph->nodes[primNode]->kids; kids != NULL && OK; kids = kids->next)
      {
        *user_data = 0;
        if(code_AssignCodeToKidConstr(cGraph, kids->id, primNode, code, listLevel+1, codeList, pConstr, pCode, user_data) == 0)
        {
          OK = 0;       

          for(kids1 = cGraph->nodes[primNode]->kids; kids1 != NULL; kids1 = kids1->next)
            codeList = (CODE_LIST_VECTOR) code_DeleteNodeFromListVector(codeList, listLevel+1, cGraph, kids1->id, pConstr);
         }
      }
    }

     
    if(!OK)
    {
      if(*user_data == 1)
      {
        /* must increase level */
        if( level < k-1)
        {
          level++;
          if(cInfo != NULL)
            cd_CloseCodeInfo(cInfo);
          cInfo = cd_OpenCodeInfo(k, level);
        }
        else
        {
          dcDestroy(&primConstr);
          dcDestroy(&code);
          if(cInfo != NULL)
            cd_CloseCodeInfo(cInfo);
          free(user_data);
          return 0;
        }
        
      }
      else
      {
        /* try to get next code on the same level */
    
        cInfo = cd_GetNextCodeInfo(cInfo);
        if(cInfo == NULL)
        {
          if(level < k-1)
          {
            level++;
            cInfo = cd_OpenCodeInfo(k, level);
          }
          else
          {
            dcDestroy(&primConstr);
            dcDestroy(&code);
            if(cInfo != NULL)
              cd_CloseCodeInfo(cInfo);
            free(user_data);
            return 0;
          }
        }
      }
    }
  }
  while(!OK);
  
  /* we have found a compatible code */
  /*printf("Code assigned %s\n", dcToStr(pCode, &code, "", ""));*/
  /*printf("Code assigned %s", dcToStr(pCode, &code, "", ""));
  printf(" to constraint %s\n", dcToStr(pConstr, &primConstr, "", ""));*/
  codeList = code_AddPairToListVector(codeList, listLevel, &primConstr, &code, pConstr, pCode);
 
  if(cInfo!=NULL)
    cd_CloseCodeInfo(cInfo);   
    
  dcDestroy(&primConstr); 
  dcDestroy(&code);
  
  free(user_data);
  
  return 1;

}

/*-- code_AssignCodeToKidConstr -----------------------------------------------------------*/

int code_AssignCodeToKidConstr(CONSTR_GRAPH cGraph, int kidNode, int fatherNode, dcube fatherCode, int listLevel, CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode, int* user_data)
{
  int OK;
  dcube code;
  dcube kidConstr;
  CODE_INFO cInfo;
  int k;      /* number of bits for primConstr; */
  int level;  /* number of DC bits for primConstr */
  int cardConstr; /* constraint cardinality */   
  CODE_LIST tmp;
  KID_LIST kids, kids1;
  int kidCard, kidLevel, fatherLevel;
  int i, exist;
  
  dcInit(pConstr, &kidConstr);
  dcInit(pCode, &code);

  dcCopy(pConstr, &kidConstr, &(cGraph->nodes[kidNode]->constr));
  /*printf("The current kid constraint %s\n", dcToStr(pConstr, &kidConstr, "", ""));*/
  
  /* if the listlevel is too big, codeList must be realocated */
  if(listLevel == codeList->nr_levels)
    codeList = code_ReallocCodeListVector(codeList, 10+codeList->nr_levels);
    
  /* if the kid is already assigned, do nothong else */
  if (code_FindConstrInListVector(codeList, &kidConstr, pConstr, &code, pCode))
  {
    dcDestroy(&kidConstr);
    dcDestroy(&code);
    *user_data = 0;
    return 1;
  }
  
  k = pinfoGetInCnt(pCode);
  cardConstr = extra_dcOutOneCnt(&kidConstr, pConstr);
  level = (int) ceil( log((double)cardConstr)/log(2.0));
  
  fatherLevel = extra_dcInGetDCCnt( &fatherCode, pCode);

  exist = 0;
  for(kids = cGraph->nodes[kidNode]->kids; kids != NULL && !exist; kids = kids->next)
  {
    kidCard = extra_dcOutOneCnt(&(cGraph->nodes[kids->id]->constr), pConstr);
    kidLevel = (int) ceil( log((double)kidCard)/log(2.0));
    if(kidLevel == level)
      exist = 1;
  }
  if( exist)
  {
    if(level < fatherLevel-1)
      level++;
    else
    {
       dcDestroy(&kidConstr);
       dcDestroy(&code);
       *user_data = 1;
       return 0;
    }
  }
  
  cInfo = cd_OpenCodeInfo(k,level);
  
  
  do
  {
    /* generate a code and try to see if it is compatible with the other codes  at level
     * level in the list codeList
     */
    OK = 1;
    
    cd_GetCodeFromCodeInfo(cInfo, pCode, &code );
    /*printf("The code: %s\n", dcToStr(pCode, &code, "", ""));*/
    
    /* the code exist - see if it is a subset of father's code */
    if(!dcIsSubSet(pCode, &fatherCode, &code))
    {
      /*printf("Not compatible because it is not a subset of father's code\n");*/
      OK = 0;
    }
    
    /* see if it is compatible with the codes in codeList */
    if(OK)
    {    
      if(code_FindCodeInListVector(codeList, &code, pCode))
      {
        /*printf("Not compatible because it already exists\n");*/
        OK = 0;
      }
    }
    
    if(OK)
    {
      for(i = 0; i<= listLevel; i++)
      {
      for(tmp = codeList->lists[i]; tmp != NULL && OK; tmp = tmp->next)
        if(!code_IsCompatible(codeList,  &kidConstr, tmp->constr, &code, tmp->code, pConstr, pCode))
          OK = 0;
     
     }
    }

   /* try to assign codes to the kids of kidConstr constraint */
    /* kids must be in reverse order of cardinality */
  
    if(OK)
    {
      for(kids = cGraph->nodes[kidNode]->kids; kids != NULL && OK; kids = kids->next)
      {
        if(code_AssignCodeToKidConstr(cGraph, kids->id,kidNode,  code, listLevel+1, codeList, pConstr, pCode, user_data) == 0)
        {
          OK = 0;       

          for(kids1 = cGraph->nodes[kidNode]->kids; kids1 != NULL; kids1 = kids1->next)
            codeList = (CODE_LIST_VECTOR) code_DeleteNodeFromListVector(codeList, listLevel+1, cGraph, kids1->id, pConstr);
          }
      }
    }
    
    if(!OK)
    {
      if(*user_data == 1)
      {
        /* must increase level - if can */
        if(level < fatherLevel-1)
        {
          level++;
          if(cInfo != NULL)
            cd_CloseCodeInfo(cInfo);
          cInfo = cd_OpenCodeInfo(k, level);
        }
        else
        {
          dcDestroy(&kidConstr);
          dcDestroy(&code);
          *user_data = 1;
          if(cInfo != NULL)
            cd_CloseCodeInfo(cInfo);
          return 0;
        }
      }
      else
      {
        cInfo = cd_GetNextCodeInfo(cInfo);
        if(cInfo == NULL)
        {
          if(level < fatherLevel-1)
          {
            level++;
            cInfo = cd_OpenCodeInfo(k, level);
          }
          else
          {
            dcDestroy(&kidConstr);
            dcDestroy(&code);
            *user_data = 0;
            if(cInfo != NULL)
              cd_CloseCodeInfo(cInfo);
            return 0;
          }
        }
      }
    }
  }
  while(!OK);
  
  /* we have found a compatible code */
  /*printf("Code assigned %s", dcToStr(pCode, &code, "", ""));
  printf(" to constraint %s\n", dcToStr(pConstr, &kidConstr, "", ""));*/

  codeList = code_AddPairToListVector(codeList, listLevel, &kidConstr, &code, pConstr, pCode);
 
  if(cInfo!=NULL)
    cd_CloseCodeInfo(cInfo);    
    
  dcDestroy(&kidConstr);
  dcDestroy(&code);
  *user_data = 0;
  
  return 1;

}

