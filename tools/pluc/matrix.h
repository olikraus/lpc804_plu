/*

  matrix.h
  
  Copyright (C) 2001 Tobias Dichtl (tobias@dichtl.org)

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
#ifndef _matrix_h
#define _matrix_h

/*
#define NOGIMPEL - wenn die Reduktionstechnik des James F. Gimpel (1965) nicht angewendet werden soll
#define NOCOUDERT - Wenn die Reduktionstechnik des Olivier Coudert (1995) nicht angewendet werden soll
*/

/* --------------------- Globals -------------------- */
/* extern FILE *fp_out; */

/* ---------------------------------- ma_ptr2field -- */
typedef struct _ma_field_struct ma_typ_field, *ma_ptr2field;
struct _ma_field_struct
{
  ma_ptr2field fieldptr_nextrow;
  ma_ptr2field fieldptr_prevrow;
  ma_ptr2field fieldptr_nextcol;
  ma_ptr2field fieldptr_prevcol;
  unsigned int row_no;
  unsigned int col_no;
};

/* ------------------------------------ ma_ptr2col -- */
typedef struct _ma_col_struct ma_typ_col, *ma_ptr2col;
struct _ma_col_struct
{
  ma_ptr2field fieldptr_firstrow;
  ma_ptr2field fieldptr_lastrow;
  ma_ptr2col colptr_next;
  ma_ptr2col colptr_prev;  
  unsigned int visited;
  unsigned int no;
  unsigned int cnt;
  dcube *cubeptr;
};

/* ------------------------------------ ma_ptr2row -- */
typedef struct _ma_row_struct ma_typ_row, *ma_ptr2row;
struct _ma_row_struct
{
  ma_ptr2field fieldptr_firstcol;
  ma_ptr2field fieldptr_lastcol;
  ma_ptr2row rowptr_next;
  ma_ptr2row rowptr_prev;
  unsigned int visited;
  unsigned int no;
  unsigned int cnt;
};

/* --------------------------------- ma_ptr2matrix -- */
typedef struct _matrix_struct ma_typ_matrix, *ma_ptr2matrix;
struct _matrix_struct
{
  ma_ptr2col *cols;
  ma_ptr2row *rows;
  ma_ptr2col colptr_first;
  ma_ptr2col colptr_last;
  ma_ptr2row rowptr_first;
  ma_ptr2row rowptr_last;
  unsigned int col_cnt;
  unsigned int row_cnt;
  unsigned int col_init;
  unsigned int row_init;
};
 
/* ------------------------------ ma_ptr2statistic -- */
typedef struct _ma_statistic_struct ma_typ_statistic, *ma_ptr2statistic;
struct _ma_statistic_struct {
  long time_msec;
  unsigned int gimpel_reduction_cnt;  /* Wie oft wurde durch Gimpel's Technik vereinfacht */
  unsigned int gimpels_above;         /* Wieviele z.Z. hoeren Rek.ebenen durch Gimpel red. worden */
  unsigned int greedy_cost_min;       /* Untere Kostenschranke bei heuristischer Loesung */
  unsigned int literals;              /* Wieviele Literale sind in der Loesung (prop. Chipflaeche) */
  unsigned int partition_build_cnt;   /* Wieviel Teilmatrizen konnten gebildet werden */
  unsigned int partitions_above;      /* Wieviel z.Z. hoeren Partitionierungen gibt es ? */
  unsigned int partitions_level;      /* Auf welcher Ebene wurde partitioniert ? */
  float partitions_offset;            /* Die Anteil der folgenden Partitionierungen ? */
  float partitions_threshold;         /* Der "Sockel", darueberliegender Teilpart. */
  unsigned int recursion_cnt;         /* Anzahl der durchlaufenen Rekusionen */
  unsigned int recursion_depth_max;   /* Maximal erreichte Rekursiontiefe */
  unsigned int use_greedy;            /* Soll rekusiv verzweigt werden */
  unsigned int progress;              /* Fortschrittsanzeige */
  unsigned int debug;                 /* Anzeige welcher Infos ??? */
};

/* ------------------------------- ma_ptr2solution -- */
typedef struct _ma_solution_struct ma_typ_solution, *ma_ptr2solution;
struct _ma_solution_struct {
  ma_ptr2row row;
  int cost;
};

/* --------------------------------- ma_ptr2cover -- */
typedef struct _cover_struct ma_typ_cover, *ma_ptr2cover;
struct _cover_struct
{
  ma_ptr2matrix matrix;
  ma_ptr2solution solution;
  int *weight;
  int weight_init;
  int do_debug;
  int do_heuristic;
  int do_weighted;
  int cnt_literals;
  int cnt_productterms;
};

/* ----------Funktionen (Deklarationsteil) ---------- */

/* ===== maCol... (Spalten-Fkt.) ==================== */
extern int  maColLogAnd (ma_ptr2col, ma_ptr2col, ma_ptr2col);   
extern int  maColCopy (ma_ptr2col, ma_ptr2col);
extern int  maColContainment (ma_ptr2col, ma_ptr2col);
extern void maColDebug (ma_ptr2col);
extern void maColDeleteFieldByPointer (ma_ptr2col, ma_ptr2field);
extern void maColDeleteFieldByRow (ma_ptr2col, int);
extern void maColDestroy (ma_ptr2col);
extern void maColFindField (ma_ptr2col, ma_ptr2field *, int);
extern int  maColInit (ma_ptr2col *);
extern int  maColInsertField (ma_ptr2col, int);
extern int  maColIntersection (ma_ptr2col, ma_ptr2col);
extern void maColLinkField (ma_ptr2col, int, ma_ptr2field *);
extern void maColShow (ma_ptr2col);

/* ===== maCover... (Schnittstellen-Fkt.) =========== */
extern int  maCoverCompute (ma_ptr2cover);
extern void maCoverDestroy (ma_ptr2cover);
extern int  maCoverGet (ma_ptr2cover, int);
extern int  maCoverInit (ma_ptr2cover *);
extern int  maCoverSetField (ma_ptr2cover, int, int);
extern int  maCoverSetWeight (ma_ptr2cover, int, int);

/* ===== maField... (Feld-Fkt.) ===================== */
extern void maFieldDestroy (ma_ptr2field);
extern int  maFieldInit (ma_ptr2field *);

/* ===== maMatrix... (Matrix-Fkt.) ================== */
extern int  maMatrixCopy (ma_ptr2matrix, ma_ptr2matrix);
extern int  maMatrixCopyCol (ma_ptr2matrix, ma_ptr2col);
extern int  maMatrixCopyRow (ma_ptr2matrix, ma_ptr2row);
extern int  maMatrixCountFields (ma_ptr2matrix);
extern int  maMatrixCoverInit (ma_ptr2matrix, ma_ptr2solution, int *, int, int);
extern int  maMatrixCoverRek (ma_ptr2matrix, ma_ptr2solution, ma_ptr2solution, int *, int, int, int, ma_ptr2statistic);
extern void maMatrixDebug1 (ma_ptr2matrix);
extern void maMatrixDebug2 (ma_ptr2matrix);
extern void maMatrixDebugSol1 (ma_ptr2matrix, ma_ptr2solution);
extern void maMatrixDebugSol2 (ma_ptr2matrix, ma_ptr2solution);
extern void maMatrixDeleteCol (ma_ptr2matrix, int);
extern void maMatrixDeleteFieldByColAndRow (ma_ptr2matrix, int, int);
extern void maMatrixDeleteFieldByPointer (ma_ptr2matrix, ma_ptr2field);
extern void maMatrixDeleteRow (ma_ptr2matrix, int);
extern void maMatrixDestroy (ma_ptr2matrix);
extern int  maMatrixExpand (ma_ptr2matrix, int, int);
extern void maMatrixFindField (ma_ptr2matrix, ma_ptr2field *, int, int);
extern int  maMatrixFindIndepCols (ma_ptr2matrix, ma_ptr2matrix, int *, ma_ptr2solution);
extern int  maMatrixInit (ma_ptr2matrix *);
extern int  maMatrixInitSized (ma_ptr2matrix *, int, int);
/*extern int  maMatrixIrredundant (pinfo *, dclist, dclist, dclist, int);*/
#define MA_LIT_NONE 0
#define MA_LIT_SOP 1
#define MA_LIT_DICHOTOMY 2
extern int  maMatrixDCL (pinfo *pi, dclist cl, int greedy, int lit_opt);
extern int  maMatrixIrredundant (pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc, dclist cl_rc, int greedy, int lit_opt);
extern int  maMatrixInsertField (ma_ptr2matrix, int, int);
extern void maMatrixLinkCol (ma_ptr2matrix, int, ma_ptr2col);
extern void maMatrixLinkRow (ma_ptr2matrix, int, ma_ptr2row);
extern int  maMatrixPartitionBlocks (ma_ptr2matrix, ma_ptr2matrix *, ma_ptr2matrix *);
extern int  maMatrixPartitionCheck(ma_ptr2matrix, ma_ptr2matrix);
extern void maMatrixProgressOut (float);
extern int  maMatrixReduceAndSolutionAccept (ma_ptr2matrix, ma_ptr2solution, int, int);
extern void maMatrixReduceAndSolutionReject (ma_ptr2matrix, int);
extern int  maMatrixReduceClassic (ma_ptr2matrix, ma_ptr2solution, int *, int);
extern void maMatrixReduceCoudert (ma_ptr2matrix, int *, int, int);
extern int  maMatrixReduceDominatedCols (ma_ptr2matrix, int *);
extern int  maMatrixReduceDominatingRows (ma_ptr2matrix);
extern int  maMatrixReduceEssentials (ma_ptr2matrix, ma_ptr2solution, int *, int);
extern int  maMatrixReduceGimpel (ma_ptr2matrix, ma_ptr2solution, ma_ptr2solution, int *, int, int, int, ma_ptr2statistic);
extern int  maMatrixSelectBranching (ma_ptr2matrix, int *, int *);
extern int  maMatrixSelectCol(ma_ptr2matrix, int *, ma_ptr2solution);
extern void maMatrixShow1 (ma_ptr2matrix);
extern void maMatrixShow2 (ma_ptr2matrix);
extern void maMatrixShowSol1 (ma_ptr2matrix, ma_ptr2solution);
extern void maMatrixShowSol2 (ma_ptr2matrix, ma_ptr2solution);
extern void maMatrixVADestroy (int, ...);
extern int  maMatrixVisitCol(ma_ptr2matrix, ma_ptr2col, int *, int *);
extern int  maMatrixVisitRow(ma_ptr2matrix, ma_ptr2row, int *, int *);
     
/* ===== maRow... (Zeilen-Fkt.) ===================== */
extern int  maRowLogAnd (ma_ptr2row, ma_ptr2row, ma_ptr2row);   
extern int  maRowCopy (ma_ptr2row, ma_ptr2row);
extern int  maRowContainment (ma_ptr2row, ma_ptr2row);
extern void maRowDebug (ma_ptr2row);
extern void maRowDeleteFieldByPointer (ma_ptr2row, ma_ptr2field);
extern void maRowDeleteFieldByCol (ma_ptr2row, int);
extern void maRowDestroy (ma_ptr2row);
extern void maRowFindField (ma_ptr2row, ma_ptr2field *, int);
extern int  maRowInit (ma_ptr2row *);
extern int  maRowInsertField (ma_ptr2row, int);
extern int  maRowIntersection (ma_ptr2row, ma_ptr2row);
extern void maRowLinkField (ma_ptr2row, int, ma_ptr2field *);
extern void maRowShow (ma_ptr2row);

/* ===== maSolution... (Loesungs-Fkt.) ============== */
extern int  maSolutionAddCol (ma_ptr2solution, int, int);
extern int  maSolutionCheckCol (ma_ptr2solution, int);
extern void maSolutionClear (ma_ptr2solution);
extern int  maSolutionCopy (ma_ptr2solution, ma_ptr2solution);
extern void maSolutionDestroy (ma_ptr2solution);
extern int  maSolutionInit (ma_ptr2solution *);
extern void maSolutionVADestroy (int, ...);
extern int  maSolutionVAInit (int, ...);
extern int  maSolutionVerify (ma_ptr2matrix, ma_ptr2solution);

/* ===== tools ====================================== */
extern void tprintf(char *fmt, ...);
extern long t_process_msec(void);


#endif
 
