/*

  dcube.h

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


#ifndef _DCUBE_H
#define _DCUBE_H

#include "cubedefs.h"
#include "pinfo.h"
#include <stdio.h>
#include <stdarg.h>

struct _dclist_struct
{
  dcube *list;
  int max;
  int cnt;
  char *flag_list;
};


void  dcInSetAll           (pinfo *pi, dcube *c, c_int v);
void  dcOutSetAll          (pinfo *pi, dcube *c, c_int v);
void  dcCopy               (pinfo *pi, dcube *dest, dcube *src);
void  dcCopyOut            (pinfo *pi, dcube *dest, dcube *src);
void  dcCopyIn             (pinfo *pi, dcube *dest, dcube *src);
void  dcCopyInToIn         (pinfo *pi_dest, dcube *dest, int dest_offset, pinfo *pi_src, dcube *src);
void  dcCopyInToInRange    (pinfo *pi_dest, dcube *dest, int dest_offset, pinfo *pi_src, dcube *src, int src_offset, int src_cnt);
void  dcCopyInToOut        (pinfo *pi_dest, dcube *dest, int dest_offset, pinfo *pi_src, dcube *src);
void  dcCopyInToOutRange   (pinfo *pi_dest, dcube *dest, int dest_offset, pinfo *pi_src, dcube *src, int src_offset, int src_cnt);
void  dcCopyOutToIn        (pinfo *pi_dest, dcube *dest, int dest_offset, pinfo *pi_src, dcube *src);
void  dcCopyOutToInRange   (pinfo *pi_dest, dcube *dest, int dest_offset, pinfo *pi_src, dcube *src, int src_offset, int src_cnt);
void  dcCopyOutToOut       (pinfo *pi_dest, dcube *dest, int dest_offset, pinfo *pi_src, dcube *src);
void  dcCopyOutToOutRange  (pinfo *pi_dest, dcube *dest, int dest_offset, pinfo *pi_src, dcube *src, int src_offset, int src_cnt);
void dcDeleteIn		(pinfo *pi, dcube *c, int pos);
void  dcOutSetAll          (pinfo *pi, dcube *c, c_int v);
void  dcAllClear           (pinfo *pi, dcube *c);
void  dcSetTautology       (pinfo *pi, dcube *c);
void  dcSetOutTautology    (pinfo *pi, dcube *c);
int   dcInit               (pinfo *pi, dcube *c);
int   dcInitVA             (pinfo *pi, int n, ...);
void  dcDestroy            (dcube *c);
void  dcDestroyVA          (int n, ...);
int   dcAdjust             (dcube *c, int in_words, int out_words);
int   dcAdjustByPinfo      (pinfo *pi, dcube *c);
int   dcAdjustByPinfoVA    (pinfo *pi, int n, ...);
void  dcSetIn              (dcube *c, int pos, int code);
int   dcGetIn              (dcube *c, int pos);
void  dcSetOut             (dcube *c, int pos, int code);
int   dcGetOut             (dcube *c, int pos);
int   dcSetByStr           (pinfo *pi, dcube *c, char *str);
int   dcSetInByStr         (pinfo *pi, dcube *c, char *str);
int   dcSetOutByStr        (pinfo *pi, dcube *c, char *str);
char *dcSetAllByStr        (pinfo *pi, int in_cnt, int out_cnt, dcube *c_on, dcube *c_dc, char *str);
char *dcToStr              (pinfo *pi, dcube *c, char *sep, char *post);
char *dcToStr2             (pinfo *pi, dcube *c, char *sep, char *post);
char *dcToStr3             (pinfo *pi, dcube *c, char *sep, char *post);
char *dcInToStr            (pinfo *pi, dcube *c, char *post);
char *dcOutToStr           (pinfo *pi, dcube *c, char *post);
int   dcInc                (pinfo *pi, dcube *c);
int   dcIncOut             (pinfo *pi, dcube *c);
int   dcIsEqualIn          (pinfo *pi, dcube *a, dcube *b);
int   dcIsEqualOut         (pinfo *pi, dcube *a, dcube *b);
int   dcIsEqualOutCnt      (dcube *a, dcube *b, int off, int cnt);
int   dcIsEqualOutRange    (dcube *a, int off_a, dcube *b, int off_b, int cnt);
int   dcIsEqual            (pinfo *pi, dcube *a, dcube *b);
int   dcIntersection       (pinfo *pi, dcube *r, dcube *a, dcube *b);
int   dcIsInSubSet         (pinfo *pi, dcube *a, dcube *b); /* Is b subset of a? */
int   dcIsSubSet           (pinfo *pi, dcube *a, dcube *b); /* Ist b Teilmenge von a? */
int   dcInDCCnt            (pinfo *pi, dcube *cube);
void  dcInDCMask           (pinfo *pi, dcube *cube);
int   dcOutCnt             (pinfo *pi, dcube *cube);
int   dcInZeroCnt          (pinfo *pi, dcube *cube);
int   dcInOneCnt           (pinfo *pi, dcube *cube);
int   dcDeltaIn            (pinfo *pi, dcube *a, dcube *b);
int   dcDeltaOut           (pinfo *pi, dcube *a, dcube *b);
int   dcDelta              (pinfo *pi, dcube *a, dcube *b);
int   dcInvIn              (pinfo *pi, dcube *cube);
void  dcInvOut             (pinfo *pi, dcube *c);
int   dcIsTautology        (pinfo *pi, dcube *c);
int   dcIsDeltaInNoneZero  (pinfo *pi, dcube *a, dcube *b);
int   dcIsDeltaNoneZero    (pinfo *pi, dcube *a, dcube *b);
int   dcIsOutIllegal       (pinfo *pi, dcube *c);
int   dcIsInIllegal        (pinfo *pi, dcube *c);
int   dcIsIllegal          (pinfo *pi, dcube *c);
int   dcCofactor           (pinfo *pi, dcube *r, dcube *a, dcube *b);
int   dcConsensus          (pinfo *pi, dcube *r, dcube *a, dcube *b);
int   dcSharpIn            (pinfo *pi, dcube *r, dcube *a, dcube *b, int k);
int   dcSharpOut           (pinfo *pi, dcube *r, dcube *a, dcube *b);
int   dcD1SharpIn          (pinfo *pi, dcube *r, dcube *a, dcube *b, int k);
int   dcD1SharpOut         (pinfo *pi, dcube *r, dcube *a, dcube *b);
int dcGetBinateInVarCofactor(pinfo *pi, dcube *r, dcube *rinv, dclist cl, dcube *cof);
int dcGetNoneDCInVarCofactor(pinfo *pi, dcube *r, dcube *rinv, dclist cl, dcube *cof);

int   dcGetCofactorForSplit(pinfo *pi, dcube *l, dcube *r, dclist cl, dcube *cof);
int   dcIsTautology        (pinfo *pi, dcube *c);
void  dcOrIn               (pinfo *pi, dcube *r, dcube *a, dcube *b);
void  dcOrOut              (pinfo *pi, dcube *r, dcube *a, dcube *b);
void  dcOr                 (pinfo *pi, dcube *r, dcube *a, dcube *b);
void  dcAnd                (pinfo *pi, dcube *r, dcube *a, dcube *b);
void  dcAndIn              (pinfo *pi, dcube *r, dcube *a, dcube *b);
void  dcNotAndOut          (pinfo *pi, dcube *r, dcube *a, dcube *b);
void  dcXorOut             (pinfo *pi, dcube *r, dcube *a, dcube *b);
int   dcGetLiteralCnt      (pinfo *pi, dcube *c);
int   dcGetDichotomyWeight (pinfo *pi, dcube *c);
int   dcWriteBin           (pinfo *pi, dcube *c, FILE *fp);
int   dcReadBin            (pinfo *pi, dcube *c, FILE *fp);

   int   dclInit               (dclist *cl);
   int   dclInitVA             (int n, ...);
   void  dclDestroy            (dclist cl);
   void  dclDestroyVA          (int n, ...);
   int   dclInitCached         (pinfo *pi, dclist *cl);
   void  dclDestroyCached      (pinfo *pi, dclist cl);
   int   dclInitCachedVA       (pinfo *pi, int n, ...);
   void  dclDestroyCachedVA    (pinfo *pi, int n, ...);
   int   dclAddEmpty           (pinfo *pi, dclist cl);
   dcube*dclAddEmptyCube       (pinfo *pi, dclist cl);
   int   dclAdd                (pinfo *pi, dclist cl, dcube *c);
   int   dclAddUnique          (pinfo *pi, dclist cl, dcube *c);
   int   dclJoin               (pinfo *pi, dclist dest, dclist src);
   int dclJoinByOut(pinfo *pi, dclist dest, dclist src, int out_pos);
   int   dclCopy               (pinfo *pi, dclist dest, dclist src);
   int dclCopyByOut(pinfo *pi, dclist dest, dclist src, int out_pos);
   int   dclClearFlags         (dclist cl);
/* void  dclSetFlag            (dclist cl, int pos); */
#define  dclSetFlag(cl,pos)            ((cl)->flag_list[(pos)] = 1)
/* int   dclIsFlag             (dclist cl, int pos); */
#define  dclIsFlag(cl,pos)            ((int)(cl)->flag_list[(pos)])
   void  dclDeleteCubesWithFlag(pinfo *pi, dclist cl);
   int   dclCopyCubesWithFlag  (pinfo *pi, dclist dest, dclist src);
   void  dclDeleteCube         (pinfo *pi, dclist cl, int pos);
   int   dclDeleteByCube       (pinfo *pi, dclist cl, dcube *c);
   int   dclDeleteByCubeList   (pinfo *pi, dclist cl, dclist del);
   void  dclClear              (dclist cl);
   void  dclRealClear          (dclist cl);
/* int   dclCnt                (dclist cl);          */
#define  dclCnt(cl)            ((cl)->cnt)
/* dcube*dclGet                (dclist cl, int pos); */
#define  dclGet(cl,pos)        ((cl)->list + (pos))
   int   dclSetPinfoByLength   (pinfo *pi, dclist cl);
   int   dclInvertOutMatrix    (pinfo *dest_pi, dclist dest_cl, pinfo *src_pi, dclist src_cl);
   int   dclReadCNF            (pinfo *pi, dclist cl, const char *filename);
   int   dclReadPLAStr         (pinfo *pi, dclist cl_on, dclist cl_dc, const char **t);
   int   dclReadFP             (pinfo *pi, dclist cl, FILE *fp);
   int   dclReadFile           (pinfo *pi, dclist cl, const char *filename);
   int   dclReadDichotomyFP    (pinfo *pi, dclist cl_on, FILE *fp);
   int   dclReadDichotomy      (pinfo *pi, dclist cl_on, const char *filename);
   int   dclReadPLA            (pinfo *pi, dclist cl_on, dclist cl_dc, const char *filename);
   int   IsValidPLAFile        (const char *filename);
   int   IsValidDichotomyFile  (const char *filename);
   int   IsValidDCLFile        (const char *filename);
   int   dclImport             (pinfo *pi, dclist cl_on, dclist cl_dc, const char *filename);
   int   dclWritePLA           (pinfo *pi, dclist cl, const char *filename);
   void  dclShow               (pinfo *pi, dclist cl);
   void  dclSetOutAll          (pinfo *pi, dclist cl, c_int v);
   void dclDeleteIn(pinfo *pi, dclist cl, int pos);
   int   dclDontCareExpand     (pinfo *pi, dclist cl);
   int   dclIsSingleSubSet     (pinfo *pi, dclist cl, dcube *c);
   void  dclGetOutput          (pinfo *pi, dclist cl, dcube *c);
   int   dclSharp              (pinfo *pi, dclist cl, dcube *a, dcube *b);
   int   dclD1Sharp            (pinfo *pi, dclist cl, dcube *a, dcube *b);
   int   dclAddDistance1       (pinfo *pi, dclist dest, dclist src);
   int   dclDistance1          (pinfo *pi, dclist dest, dclist src);
   int   dclDistance1Cube      (pinfo *pi, dclist dest, dcube *c);
   int   dclRestrictByDistance1(pinfo *pi, dclist a, dclist b);
   int   dclSCCSharpAndSetFlag (pinfo *pi, dclist cl, dcube *a, dcube *b);
   int   dclComplementCube     (pinfo *pi, dclist cl, dcube *c);
   int   dclIntersection       (pinfo *pi, dclist cl, dcube *c);
   int   dclIntersectionCube   (pinfo *pi, dclist dest, dclist src, dcube *c);
   int   dclSCCIntersectionCube(pinfo *pi, dclist dest, dclist src, dcube *c);
   int   dclIntersectionList   (pinfo *pi, dclist dest, dclist a, dclist b);
   int   dclConvertFromPOS     (pinfo *pi, dclist cl);
   int   dclIsIntersection     (pinfo *pi, dclist a, dclist b);
   int   dclIsIntersectionCube (pinfo *pi, dclist cl, dcube *c);
   int   dclIntersectionListInv(pinfo *pi, dclist dest, dclist a, dclist b);
   void  dclAndElements        (pinfo *pi, dcube *r, dclist cl);
   void  dclOrElements         (pinfo *pi, dcube *r, dclist cl);
   void  dclResult             (pinfo *pi, dcube *r, dclist cl);
   int   dclResultList         (pinfo *pi, dclist r, dclist cl);
#define  dclSuper(pi,r,cl)     dclOrElements((pi),(r),(cl))
   void  dclSuper2             (pinfo *pi, dcube *r, dclist cl1, dclist cl2);
   int   dclAndLocalElements   (pinfo *pi, dcube *r, dcube *m, dclist cl);
   int   dclSCCCofactor        (pinfo *pi, dclist dest, dclist src, dcube *cofactor);
   int   dclCofactor           (pinfo *pi, dclist dest, dclist src, dcube *cofactor);
   int   dclGetCofactorCnt     (pinfo *pi, dclist src, dcube *cofactor);
   int   dclSCCAddAndSetFlag   (pinfo *pi, dclist cl, dcube *c);
   int   dclSCCInvAddAndSetFlag(pinfo *pi, dclist cl, dcube *c);
   int   dclSCCConsensusCube   (pinfo *pi, dclist a, dcube *b);
   int   dclConsensusCube      (pinfo *pi, dclist a, dcube *b);
   int   dclConsensus          (pinfo *pi, dclist dest, dclist a, dclist b);
   int   dclIsBinateInVar      (dclist cl, int var);
   int   dclIsDCInVar          (pinfo *pi, dclist cl, int var);
   int   dclTautology          (pinfo *pi, dclist cl);
   int   dclIsSubSet           (pinfo *pi, dclist cl, dcube *cube);
   int   dclRemoveSubSet       (pinfo *pi, dclist cl, dclist cover, dclist removed);
   void  dclSortOutput         (pinfo *pi, dclist cl);
   int   dclSCC                (pinfo *pi, dclist cl);
   int   dclSCCInv             (pinfo *pi, dclist cl);
   int   dclRemoveEqual        (pinfo *pi, dclist cl);
   int   dclSCCUnion           (pinfo *pi, dclist dest, dclist src);
   int   dclSCCSubtractCube    (pinfo *pi, dclist a, dcube *b);
   int   dclSubtractCube       (pinfo *pi, dclist a, dcube *b);
   int   dclSubtract           (pinfo *pi, dclist a, dclist b);
   int   dclComplementOut      (pinfo *pi, int o, dclist cl);
   int   dclIsSubsetList       (pinfo *pi, dclist a, dclist b); /* Ist b Teilmenge von a? */
   int   dclIsEquivalent       (pinfo *pi, dclist a, dclist b);
   int   dclIsEquivalentDC     (pinfo *pi, dclist cl, dclist cl_on, dclist cl_dc);
   int   dclAreaIsMaxLarge     (pinfo *pi, dclist cl_s, dclist cl_area, int *is_max_large);
   int   dclArea               (pinfo *pi, dclist cl_s, dclist cl_area);
   int   dclIsRelated          (pinfo *pi, dclist cl);
   int   dclComplementWithSharp(pinfo *pi, dclist cl);
   int   dclComplementWithURP  (pinfo *pi, dclist cl);
   int   dclComplement         (pinfo *pi, dclist cl);
   int   dclPrimes             (pinfo *pi, dclist cl);
   int   dclPrimesDC           (pinfo *pi, dclist cl, dclist cl_dc);
   int   dclPrimesInv          (pinfo *pi, dclist cl);
   int   dclIsEssential        (pinfo *pi, dclist cl_on, dclist cl_dc, dcube *a);
   int   dclEssential          (pinfo *pi, dclist cl_es, dclist cl_nes, dclist cl, dclist cl_on, dclist cl_dc);
   int   dclGetEssential       (pinfo *pi, dclist cl, dclist cl_on, dclist cl_dc);
   int   dclSplitEssential     (pinfo *pi, dclist cl_es, dclist cl_fr, dclist cl_pr, dclist cl, dclist cl_on, dclist cl_dc);
   int   dclSplitRelativeEssential(pinfo *pi, dclist cl_es, dclist cl_fr, dclist cl_pr, dclist cl, dclist cl_dc, dclist cl_rc);
   int   dclIrredundantMark    (pinfo *pi_m, dclist cl_m, pinfo *pi, dclist cl_pr, int pos, dclist cl_es, dclist cl_dc);
   int   dclIrredundantMatrix  (pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc, int (*set_fn)(void *data, int x, int y), void *data);
   int   dclIrredundantDCubeMatrix(pinfo *pi_m, dclist cl_m, pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc);
   int   dclIrredundantDCubeTMatrix(pinfo *pi_m, dclist cl_m, pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc);
   int   dclIrredundantDCubeTMatrixRC(pinfo *pi_m, dclist cl_m, pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc, dclist cl_rc);
   int   dclMarkRequiredCube   (pinfo *pi_m, dclist cl_m, pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_rc);
   int   dclReduceDCubeTMatrix (pinfo *pi_m, dclist cl_m, pinfo *pi_pr, dclist cl_pr);
   int   dclIrredundant        (pinfo *pi, dclist cl, dclist cl_dc);
   void  dclRestrictOutput     (pinfo *pi, dclist cl);
   int   dclMinimize           (pinfo *pi, dclist cl);
   int   dclMinimizeDC         (pinfo *pi, dclist cl, dclist cl_dc, int greedy, int is_literal);
   int   dclWriteBin           (pinfo *pi, dclist cl, FILE *fp);
   int   dclReadBin            (pinfo *pi, dclist *cl, FILE *fp);
   int   dclGetLiteralCnt      (pinfo *pi, dclist cl);


/* pinfo.c */
int pinfoCntDCList(pinfo *pi, dclist cl, dcube *cof);
int pinfoGetNoneDCInVarCnt(pinfo *pi);

int pinfoGetInVarDCubeCofactor(pinfo *pi, dcube *r, dcube *rinv, dclist cl, dcube *cof);
int pinfoGetOutVarDCubeCofactor(pinfo *pi, dcube *r, dcube *rinv, dclist cl, dcube *cof);
int pinfoGetDCubeCofactorForSplitting(pinfo *pi, dcube *r, dcube *rinv, dclist cl, dcube *cof);

int pinfoMerge(pinfo *pi_dest, dclist cl_dest, pinfo *pi_src, dclist cl_src);



/* dcubevhdl.c */

int dclWriteVHDLEntityFP(pinfo *pi, const char *entity, FILE *fp);
int dclWriteVHDLArchitectureFP(pinfo *pi, dclist cl, const char *entity, FILE *fp);
int dclWriteVHDLVecTBFP(pinfo *pi, dclist cl, const char *entity, int wait_ns, FILE *fp);
int dclWriteVHDLSOPTBFP(pinfo *pi, dclist cl, const char *entity, int wait_ns, FILE *fp);

/* writes a testbench if wait_ns >= 0 */
int dclWriteVHDL(pinfo *pi, dclist cl, const char *entity, int wait_ns, char *name);

/* dcubehf.c */

/* one of f_out(start) and f_out(end) is 1                      */
#define DCH_MSG_ERROR_NOT_UNATE         50

/* f_out(start) = 1, f_out(end) = 1                             */
/* super ist not a subset of any elements of cl                 */
#define DCH_MSG_ERROR_SUPER_NOT_SUBSET  100

/* f_out(start) = 1, f_out(end) = 1                             */
/* super is subset of one or more elements of cl                */
/* a valid solution must include at least one of these elements */
#define DCH_MSG_OK_SUPER_IS_SUBSET      101

#define DCH_MSG_ERROR_START_NOT_COVERED 200
#define DCH_MSG_OK_START_IS_COVERED     201

#define DCH_MSG_ERROR_END_NOT_COVERED   300
#define DCH_MSG_OK_END_IS_COVERED       301

#define DCH_MSG_ERROR_SUPER_HAS_INTERSECTION 400
#define DCH_MSG_OK_SUPER_HAS_NO_INTERSECTION 401

struct dchazard_struct
{
  int msg;
  dclist cl;     /* cover */
  dcube *start;  /* start cube */
  dcube *end;    /* end cube */
  int out;       /* considered function f_out() */
  int f_start;   /* functionvalue f_out(start) */
  int f_end;     /* functionvalue f_out(end) */
  dcube *super;  /* = super(start, end) = or(start, end) */
  dcube *inter;  /* start: inter = taut */
                 /* c in and(cl,super) and c!={}: inter = intersect(inter, c) */
  dclist cl_local; /* = SCC(interect(cl, super)) */
  int type;      /* hazard type 0, 1, 2, 3  (dch.f_start<<1)|dch.f_end */
  int is_ok;
};
typedef struct dchazard_struct dchazard;

int dclHazardAnalysisOut(pinfo *pi, dclist cl, dcube *start, dcube *end, int out, int (*fn)(void *data, dchazard *dch), void *data);
int dclHazardAnalysis(pinfo *pi, dclist cl, dcube *start, dcube *end, int out, int (*fn)(void *data, dchazard *dch), void *data);

int dclIsHazardfreeTransition(pinfo *pi, dclist cl, dcube *start, dcube *end);

/* dcex.c */
int dclReadBEXStr(pinfo *pi, dclist cl_on, dclist cl_dc, const char *content);
int dclReadBEX(pinfo *pi, dclist cl_on, dclist cl_dc, const char *filename);
int IsValidBEXStr(const char *content);
int IsValidBEXFile(const char *filename);
int dclWriteBEX(pinfo *pi, dclist cl, const char *filename);
int dclWrite3DEQN(pinfo *pi, dclist cl, const char *filename);

/* nex.c */
int dclReadNEX(pinfo *pi, dclist cl_on, dclist cl_dc, const char *filename);
int IsValidNEXFile(const char *filename);

/* dcubebcp.h */
int dclBCP(pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc);
int dclMinimizeDCWithBCP(pinfo *pi, dclist cl, dclist cl_dc);

/* dcubeustt.h */
int dclPrimesUSTT(pinfo *pi, dclist cl);
int dclMinimizeUSTT(pinfo *pi, dclist cl, void (*msg)(void *data, char *fmt, va_list va), void *data, const char *pre, const char *primes_file);

#endif /* _DCUBE_H */
