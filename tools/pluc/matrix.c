/*

  matrix.c
  
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

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdarg.h>
/* #include <sys/resource.h> */
#include "dcube.h"
#include "matrix.h"

/* not always allowed... */
/* FILE *fp_out = stderr; */
/* changed to: */
static FILE *fp_out = NULL;


/* ===== maCol... (Spalten-Fkt.) ==================== */

/* ----- maColContainment --------------------------- */
int maColContainment (ma_ptr2col h_colptr_src1, ma_ptr2col h_colptr_src2)
{
  /* Rueckgabe = Enthaelt die src2-Spalte die src1-Spalte ?  1 : 0 */
  int rtc = 1;
  ma_ptr2field field1, field2;
  
  field1 = h_colptr_src1->fieldptr_firstrow;
  field2 = h_colptr_src2->fieldptr_firstrow;
  
  /* Nur ueberpruefen, wenn h_colptr_src1 ueberhaupt Felder enthaelt, sonst "containt" ihn der h_colptr_src2 in jedem Fall */
  while (field1 != NULL)
  {
    if (field2 == NULL || field1->row_no < field2->row_no)
    {
      rtc = 0;
      break;
    }
    else
    {
      if (field1->row_no == field2->row_no)
      {
        field1 = field1->fieldptr_nextrow;
        field2 = field2->fieldptr_nextrow;
      }
      else
      {
        field2 = field2->fieldptr_nextrow;
      }
    }
  }
  
  return rtc;
}

/* ----- maColCopy ---------------------------------- */
int maColCopy (ma_ptr2col h_colptr_dest, ma_ptr2col h_colptr_src)
{
  ma_ptr2field field, prevfield;
  int rtc;
  
  /* h_colptr_dest von evtl. Altlasten befreien */
  for (field = h_colptr_dest->fieldptr_lastrow; field != NULL; field = prevfield)
  {
    prevfield = field->fieldptr_prevrow;
    (void) maColDeleteFieldByPointer(h_colptr_dest, field);
  }

  for (field = h_colptr_src->fieldptr_firstrow; field!=NULL; field = field->fieldptr_nextrow)
    if (!maColInsertField(h_colptr_dest, field->row_no))
    {
      rtc = 0;
      break;
    }
    
  return rtc;
}

/* ----- maColDebug --------------------------------- */
void maColDebug (ma_ptr2col h_colptr)
{
  FILE *fp_hlp;

  if ( fp_out == NULL )
    fp_out = stderr;

  fp_hlp = fp_out;
  fp_out = stderr;

  (void) maColShow(h_colptr);

  fp_out = fp_hlp;
}

/* ----- maColDeleteFieldByRow ---------------------- */
void maColDeleteFieldByRow (ma_ptr2col h_colptr, int h_row)
{
  ma_ptr2field field;

  if (h_row <= ((h_colptr->fieldptr_lastrow->row_no + h_colptr->fieldptr_firstrow->row_no) / 2))
    for (field = h_colptr->fieldptr_firstrow; field != NULL && field->row_no < h_row; field = field->fieldptr_nextrow);
  else
    for (field = h_colptr->fieldptr_lastrow; field != NULL && field->row_no > h_row; field = field->fieldptr_prevrow);
  
  /* Existiert das Feld in der Spalte ueberhaupt ? */
  if (field != NULL && field->row_no == h_row)
      (void) maColDeleteFieldByPointer(h_colptr, field);
}

/* ----- maColDeleteFieldByPointer ------------------ */
void maColDeleteFieldByPointer (ma_ptr2col h_colptr, ma_ptr2field h_field)
{
    /* Das Feld muss aus seinem Spaltenbezug geloest werden */

    /* Vorderen Bezug des loesen */
    if (h_field->fieldptr_prevrow == 0)
      h_colptr->fieldptr_firstrow = h_field->fieldptr_nextrow;
    else
      h_field->fieldptr_prevrow->fieldptr_nextrow = h_field->fieldptr_nextrow;

    /* Hinteren Bezug des loesen */
    if (h_field->fieldptr_nextrow == 0)
      h_colptr->fieldptr_lastrow = h_field->fieldptr_prevrow;
    else
      h_field->fieldptr_nextrow->fieldptr_prevrow = h_field->fieldptr_prevrow;

    h_colptr->cnt--;
    (void) maFieldDestroy(h_field);
}

/* ----- maColDestroy ------------------------------- */
void maColDestroy (ma_ptr2col h_col)
{
  ma_ptr2field thisfield, nextfield; 

  if (h_col != NULL)
  {
    /* Alle enthaltenen Felder entfernen */
    for (thisfield = h_col->fieldptr_firstrow; thisfield != NULL; thisfield = nextfield)
    {
      nextfield = thisfield->fieldptr_nextrow;
      (void) maFieldDestroy(thisfield);
    }
  
    /* die Felder der Struktur zuruecksetzen ... */
    h_col->fieldptr_firstrow = NULL;
    h_col->fieldptr_lastrow = NULL;
    h_col->colptr_next = NULL;
    h_col->colptr_prev = NULL;
    h_col->cubeptr = NULL;
    h_col->visited = 0;
    h_col->no = 0;
    h_col->cnt = 0;
  
    /* und freigeben. */
    free(h_col);
  }
}

/* ----- maColFindField ----------------------------- */
void maColFindField (ma_ptr2col h_colptr, ma_ptr2field *h_field, int h_row)
{
  /* Anfahren der Feldnnummer im Feldkontext */
  if (h_row <= ((h_colptr->fieldptr_lastrow->row_no + h_colptr->fieldptr_firstrow->row_no) / 2))
    /* Pointer auf betreffendes Feld stellen (je nachdem was schneller zu erreichen ist: von vorne) */
    for ((*h_field) = h_colptr->fieldptr_firstrow; (*h_field) != NULL && (*h_field)->row_no < h_row; (*h_field) = (*h_field)->fieldptr_nextrow);
  else
    /* Pointer auf betreffendes Feld stellen (je nachdem was schneller zu erreichen ist: von hinten) */
    for ((*h_field) = h_colptr->fieldptr_lastrow; (*h_field) != NULL && (*h_field)->row_no > h_row; (*h_field) = (*h_field)->fieldptr_prevrow);

 /* Gibt es ueberhaupt Felder in dieser Col und existiert das gesuchte Feld No: # h_row ?? */
  if (*h_field != NULL)
    if ((*h_field)->row_no != h_row)
      *h_field = NULL;
}

/* ----- maColInit ---------------------------------- */
int maColInit (ma_ptr2col *newcol)
{
  int rtc = 0;

  *newcol = (ma_ptr2col)malloc(sizeof(ma_typ_col));
  if (*newcol != NULL)                                
  {
    (*newcol)->fieldptr_firstrow = NULL;
    (*newcol)->fieldptr_lastrow = NULL;
    (*newcol)->colptr_next = NULL;
    (*newcol)->colptr_prev = NULL;
    (*newcol)->cubeptr = NULL;
    (*newcol)->visited = 0;
    (*newcol)->no = 0;
    (*newcol)->cnt = 0;
    rtc++;
  }
  return rtc;
}  

/* ----- maColInsertField --------------------------- */
int maColInsertField (ma_ptr2col h_colptr, int h_row)
{
  ma_ptr2field newfield, nf_copy;
  int rtc = 0;

  /* Das neue Feld der Spalte aufbauen ... */
  if (maFieldInit(&newfield))
  {  
    nf_copy = newfield;
  
    /* Das evtl. neue Feld in die Spalten einfuegen */
    (void) maColLinkField(h_colptr, h_row, &newfield);

    /* Wenn beim Linken klar wurde, dass das Feld nicht neu war wieder loeschen */
    if (newfield != nf_copy)
      (void) maFieldDestroy(nf_copy);

    rtc++;
  }
  
  return rtc;
}

/* ----- maColIntersection -------------------------- */
int maColIntersection (ma_ptr2col h_colptr_src1, ma_ptr2col h_colptr_src2)
{
  int rtc = 0;
  ma_ptr2field field1, field2;
  
  field1 = h_colptr_src1->fieldptr_firstrow;
  field2 = h_colptr_src2->fieldptr_firstrow;
  
  if (field1 != NULL && field2 != NULL)
    for (; ; )
    {
      if (field1->row_no < field2->row_no)
      {
        field1 = field1->fieldptr_nextrow;
        if (field1 == NULL)
          break;
      }
      else
      {
        if (field1->row_no > field2->row_no)
        {
          field2 = field2->fieldptr_nextrow;
          if (field2 == NULL)
            break;
        }
        else
        {
          rtc++;
          break;
        }
      }
    }
  
  return rtc;
}

/* ----- maColLinkField ----------------------------- */
void maColLinkField (ma_ptr2col h_colptr, int h_rowno, ma_ptr2field *h_fieldptr)
{                   
  ma_ptr2field lo_fieldptr;
  
  (*h_fieldptr)->row_no = h_rowno;
  h_colptr->cnt++;

  /* Fall 1.) Die Row enthaelt noch gar keine Felder ... */
  if (h_colptr->fieldptr_lastrow == NULL)
  {
    (*h_fieldptr)->fieldptr_prevrow = NULL;
    (*h_fieldptr)->fieldptr_nextrow = NULL;

    h_colptr->fieldptr_firstrow = (*h_fieldptr);
    h_colptr->fieldptr_lastrow = (*h_fieldptr);
  }
  else 
  {
    /* Fall 2.) Das spezifizierte Feld ist letztes Feld der Spalte und muss linksseitig in den Feldkontext der Matrix einsortiert werden ... */
    if (h_colptr->fieldptr_lastrow->row_no < h_rowno)
    {
      (*h_fieldptr)->fieldptr_prevrow = h_colptr->fieldptr_lastrow;
      (*h_fieldptr)->fieldptr_nextrow = NULL;

      h_colptr->fieldptr_lastrow->fieldptr_nextrow = (*h_fieldptr);
      h_colptr->fieldptr_lastrow = (*h_fieldptr);
    }
    else 
    {
      /* Fall 3.) Das spezifizierte Feld ist erstes Feld der Spalte und muss rechtsseitig in den Feldkontext der Matrix einsortiert werden ... */
      if (h_colptr->fieldptr_firstrow->row_no > h_rowno)
      {
        (*h_fieldptr)->fieldptr_prevrow = NULL;
        (*h_fieldptr)->fieldptr_nextrow = h_colptr->fieldptr_firstrow;

        h_colptr->fieldptr_firstrow->fieldptr_prevrow = (*h_fieldptr);
        h_colptr->fieldptr_firstrow = (*h_fieldptr);
      } 
      else
      {
        /* Anfahren der Feldnnummer im Feldkontext "von oben" */
        if (h_rowno <= ((h_colptr->fieldptr_lastrow->row_no + h_colptr->fieldptr_firstrow->row_no) / 2))
          for(lo_fieldptr = h_colptr->fieldptr_firstrow; lo_fieldptr->row_no < h_rowno; lo_fieldptr = lo_fieldptr->fieldptr_nextrow);
        else
        {
          /* Anfahren des Feldes "von unten" */
          lo_fieldptr = h_colptr->fieldptr_lastrow;
          if (h_rowno != lo_fieldptr->row_no)
          {
            for(; lo_fieldptr->row_no >= h_rowno; lo_fieldptr = lo_fieldptr->fieldptr_prevrow);

            lo_fieldptr = lo_fieldptr->fieldptr_nextrow;
          }
        }
        
        /* Fall 4.) Das spezifizierte Feld muss beidseitig in den Feldkontext der Matrix einsortiert werden ... */
        if (lo_fieldptr->row_no > h_rowno)
        {
          /* zu weit -> wieder eine Position nach voerne ruecken (auf alle Faelle ist jetzt bekannt, dass das Feld noch nicht exisitierte) */
          lo_fieldptr = lo_fieldptr->fieldptr_prevrow;

          (*h_fieldptr)->fieldptr_prevrow = lo_fieldptr;
          (*h_fieldptr)->fieldptr_nextrow = lo_fieldptr->fieldptr_nextrow;

          lo_fieldptr->fieldptr_nextrow->fieldptr_prevrow = (*h_fieldptr);
          lo_fieldptr->fieldptr_nextrow = (*h_fieldptr);
        }
        else
        {
          /* Fall 5.) Das spezifizierte Feld exisitert bereits (dann war die Erheohung des Spalten-Counters unzulaessig)... */
          h_colptr->cnt--;
          (*h_fieldptr) = lo_fieldptr;
        }
      }
    }
  }
}

/* ----- maColLogAnd -------------------------------- */
int maColLogAnd (ma_ptr2col h_colptr_dest, ma_ptr2col h_colptr_src1, ma_ptr2col h_colptr_src2)
{
  ma_ptr2field field1, field2;
  int rtc = 0;
  
  /* Wenn die Destination nicht leer ist */
  if (h_colptr_dest->cnt = 0)
    rtc = 1;

  field1 = h_colptr_src1->fieldptr_firstrow;
  field2 = h_colptr_src2->fieldptr_firstrow;

  if (field1 != NULL && field2 != NULL && rtc != 0)  
    for (; ; )
    {
      if (field1->row_no < field2->row_no)
      {
        field1 = field1->fieldptr_nextrow;
        if (field1 == NULL)
          break;
      }
      else
      {
        if (field1->row_no > field2->row_no)
        {
          field2 = field2->fieldptr_nextrow;
          if (field2 == NULL)
            break;
        }
        else
        {
          if (!maColInsertField(h_colptr_dest, field1->row_no))
          {
            rtc = 0;
            break;
          }
          field1 = field1->fieldptr_nextrow;
          field2 = field2->fieldptr_nextrow;
          if (field1 == NULL || field2 == NULL)
            break;
        }
      }
    }    
  return rtc;
}

/* ----- maColShow ---------------------------------- */
void maColShow (ma_ptr2col h_colptr)
{
  ma_ptr2field field;

  tprintf("col no %3d: ", h_colptr->no);

  for (field = h_colptr->fieldptr_firstrow; field != NULL; field->fieldptr_nextrow)
    tprintf("%d ", field->row_no);

  tprintf("/n");
}


/* ===== maCover... (Schnittstellen-Fkt.) =========== */

/* ----- maCoverCompute ----------------------------- */
int maCoverCompute (ma_ptr2cover cover)
{
  ma_ptr2field field;
  ma_ptr2col colptr;
  
  int rtc = 0;
  
  if (maMatrixCoverInit(cover->matrix, cover->solution, (cover->do_weighted ? cover->weight : NULL), cover->do_heuristic, (12 * cover->do_debug)))
  {
    cover->cnt_productterms = cover->solution->row->cnt;

    for (field = cover->solution->row->fieldptr_firstcol; field != NULL; field = field->fieldptr_nextcol)
    {
      colptr = cover->matrix->cols[field->col_no];
      cover->cnt_literals += colptr->cnt;
    }
    
    rtc++;
  }

  return rtc;
}

/* ----- maCoverDestroy ----------------------------- */
void maCoverDestroy (ma_ptr2cover cover)
{
  if (cover->matrix != NULL)
    (void) maMatrixDestroy(cover->matrix);

  if (cover->solution != NULL)
    (void) maSolutionDestroy(cover->solution);

  if (cover->weight != NULL)
    free(cover->weight);
  
  cover->cnt_literals = 0;
  cover->cnt_productterms = 0;
  cover->do_debug = 0;
  cover->do_heuristic = 0;
  cover->do_weighted = 0;
  cover->matrix = NULL;
  cover->solution = NULL;
  cover->weight = NULL;

  if (cover != NULL)
    free(cover);
}

/* ----- maCoverGet --------------------------------- */
int maCoverGet (ma_ptr2cover cover, int col)
{
  return maSolutionCheckCol(cover->solution, col);
}

/* ----- maCoverInit -------------------------------- */
int maCoverInit (ma_ptr2cover *cover)
{
  ma_ptr2matrix matrix;
  ma_ptr2solution solution;
  int rtc = 0, *weight_ar;
  
  weight_ar = (int*) malloc(sizeof(int) * 1);
  
  if ((weight_ar != NULL) && maMatrixInit(&matrix) && maSolutionInit(&solution))
  {
    weight_ar[0] = 0;
  
    *cover = (ma_ptr2cover)malloc(sizeof(struct _cover_struct));
    if (*cover != NULL)
    {
      (*cover)->matrix = matrix;
      (*cover)->solution = solution;
      (*cover)->weight = weight_ar;
      (*cover)->weight_init = 1;

      (*cover)->cnt_literals = 0;
      (*cover)->cnt_productterms = 0;

      (*cover)->do_debug = 0;
      (*cover)->do_heuristic = 0;
      (*cover)->do_weighted = 0;
      rtc++;
    }
  }

  return rtc;
}

/* ----- maCoverSetField ---------------------------- */
int maCoverSetField (ma_ptr2cover cover, int col, int row)
{
  return maMatrixInsertField(cover->matrix, col, row);
}

/* ----- maCoverSetWeight --------------------------- */
int maCoverSetWeight (ma_ptr2cover cover, int col, int val)
{
  int i, rtc = 0;

  cover->do_weighted = 1;
  
  if (col >= cover->weight_init)
  {
    if ( cover->weight == NULL )
      cover->weight = (int *) malloc((col + 1) * sizeof(int));
    else
      cover->weight = (int *) realloc(cover->weight, (col + 1) * sizeof(int));
  
    if (cover->weight != NULL)
    {
      for (i = cover->weight_init; i < col; i++)
          cover->weight[i] = 0;
    
      cover->weight_init = (col + 1);
      rtc++;
    }
  }
  else
    rtc++;
    
  cover->weight[col] = val;
  
  return rtc;
}


/* ===== maField... (Feld-Fkt.) ===================== */

/* ----- maFieldDestroy ----------------------------- */
void maFieldDestroy (ma_ptr2field field)
{
  if (field != NULL)
  {
    /* die Felder der Struktur zuruecksetzen ... */
    field->fieldptr_nextrow = NULL;
    field->fieldptr_prevrow = NULL;
    field->fieldptr_nextcol = NULL;
    field->fieldptr_prevcol = NULL;
    field->row_no = 0;
    field->col_no = 0;

    /* und freigeben. */
    free(field);
  }
}

/* ----- maFieldInit -------------------------------- */
int maFieldInit (ma_ptr2field *newfield)
{
  int rtc = 0;
  
  *newfield = (ma_ptr2field)malloc(sizeof(struct _ma_field_struct));
  if (*newfield != NULL)
  {
    (*newfield)->fieldptr_nextrow = NULL;
    (*newfield)->fieldptr_prevrow = NULL;
    (*newfield)->fieldptr_nextcol = NULL;
    (*newfield)->fieldptr_prevcol = NULL;
    (*newfield)->row_no = 0;
    (*newfield)->col_no = 0;
    rtc++;
  }
  return rtc;
}

/* ===== maMatrix... (Matrix-Fkt.) ================== */

/* ----- maMatrixBuildRowIntersection --------------- */
int maMatrixBuildRowIntersection(ma_ptr2matrix sparse, ma_ptr2matrix intersection)
{
  int rtc = 1;
  ma_ptr2col f_col;
  ma_ptr2row f_row, f_crossrow;
  ma_ptr2field field, f_inrow;
  
  /* Schleife ueber alle Zeilen der Matrix, darin jeweils: Spalte fuer Spalte alle Ueberdeckungen mit anderen Zeilen protokollieren */
  for (f_row = sparse->rowptr_first; f_row != NULL && rtc != 0; f_row = f_row->rowptr_next)
  {  
    /* Damit das mehrfache Anfahren eines Feldes nicht immer wieder zum erneuten Insert fuehrt, 
       wird zuerst das Flag "visited" auf Null gesetzt ... */
    for (f_inrow = f_row->fieldptr_firstcol; f_inrow != NULL && rtc != 0; f_inrow = f_inrow->fieldptr_nextcol)
    {
      f_col = sparse->cols[f_inrow->col_no];
      
      for (field = f_col->fieldptr_firstrow; field != NULL; field = field->fieldptr_nextrow)
      {
        f_crossrow = sparse->rows[field->row_no];
        f_crossrow->visited = 0;
      }
    }

    /* ... und nur dann wird ein Feld inserted, wenn dieses Flag immer noch auf Null steht (sonst existiert es ja bereits) */
    for (f_inrow = f_row->fieldptr_firstcol; f_inrow != NULL && rtc != 0; f_inrow = f_inrow->fieldptr_nextcol)
    {
      f_col = sparse->cols[f_inrow->col_no];
      
      for (field = f_col->fieldptr_firstrow; field != NULL; field = field->fieldptr_nextrow)
      {
        f_crossrow = sparse->rows[field->row_no];
        if (f_crossrow->visited == 0)
          if (!maMatrixInsertField(intersection, f_row->no, f_crossrow->no))
          {
            rtc--;
            break;
          }
      }
    }
  }  
  return rtc;
}

/* ----- maMatrixCopy ------------------------------- */
int maMatrixCopy (ma_ptr2matrix sparse_dest, ma_ptr2matrix sparse_src)
{
  int rtc = 0;
  ma_ptr2col f_col; 
  ma_ptr2field field;

  /* bei Bedarf kann diese Kopierroutine auch den Cubeptr mitkopieren, s.u. (fuer Mincov aber nicht noetig) */
  if (sparse_src->rowptr_last != NULL) /* wir arbeiten ja nicht umsonst */
  {  
    /* Groesse von dest anpassen */
    if (maMatrixExpand(sparse_dest, sparse_src->colptr_last->no, sparse_src->rowptr_last->no))
    {
      rtc++;
      
      /* Spalte fuer Spalte, Feld fuer Feld ... */
      for (f_col = sparse_src->colptr_first; f_col != NULL; f_col = f_col->colptr_next)
      {
        for (field = f_col->fieldptr_firstrow; field != NULL; field = field->fieldptr_nextrow)
          if (!maMatrixInsertField(sparse_dest, f_col->no, field->row_no))
          {
            rtc = 0;
            break;          
          }
      }
    }  
  }
  else
    rtc = 1;
    
  return rtc;
}

/* ----- maMatrixCopyCol ---------------------------- */
int maMatrixCopyCol (ma_ptr2matrix sparse, ma_ptr2col h_srcptr)
{
  ma_ptr2field field;
  int rtc = 1;

  for (field = h_srcptr->fieldptr_firstrow; field != NULL && rtc != 0; field = field->fieldptr_nextrow)
    if (!maMatrixInsertField(sparse, field->col_no, field->row_no))
      rtc = 0;
    
  return rtc;
}

/* ----- maMatrixCopyRow ---------------------------- */
int maMatrixCopyRow (ma_ptr2matrix sparse, ma_ptr2row h_srcptr)
{
  ma_ptr2field field;
  int rtc = 1;
  
  for (field = h_srcptr->fieldptr_firstcol; field != NULL && rtc != 0; field = field->fieldptr_nextcol)
    if (!maMatrixInsertField(sparse, field->col_no, field->row_no))
      rtc = 0;

  return rtc;
}

/* ----- maMatrixCountFields ------------------------ */
int maMatrixCountFields (ma_ptr2matrix sparse)
{
  int rtc = 0;
  ma_ptr2col f_col;
  
  for (f_col = sparse->colptr_first; f_col != NULL; f_col = f_col->colptr_next)
    rtc += f_col->cnt;
    
  return rtc;
}

/* ----- maMatrixCoverInit ---------------------------- */
int maMatrixCoverInit (ma_ptr2matrix sparse, ma_ptr2solution result, int *weight, int branching, int debug)
{
  ma_ptr2matrix sparse_rek;
  ma_typ_statistic stat_daten;
  ma_ptr2statistic statistic = &stat_daten;
  ma_ptr2solution sol_forchild;
  ma_ptr2col f_col;
  ma_ptr2field f_field;
  
  int rtc = 0, cost_max = 0;

  if (sparse->row_cnt < 1)
    return (1);
  
  if (result->row->cnt > 0)
    return (0);
  
  /*  Initialisieren der Statistik-Struktur */
  statistic->gimpel_reduction_cnt = 0;  
  statistic->gimpels_above = 0;         
  statistic->greedy_cost_min = 0;      
  statistic->literals = 0;
  statistic->partition_build_cnt = 0;   
  statistic->partitions_level = 0;
  statistic->partitions_above = 0;     
  statistic->partitions_offset = 1;     
  statistic->partitions_threshold = 0;
  statistic->recursion_cnt = 0;             
  statistic->recursion_depth_max = 0;   
  statistic->use_greedy = branching;           
  statistic->progress = 0;           
  statistic->debug = debug;           
  
  /* Die obere Grenze festlegen */
  for (f_col = sparse->colptr_first; f_col !=NULL; f_col = f_col->colptr_next)
    cost_max += ((weight == NULL) ? 1 : weight[f_col->no] );
  
  if (maMatrixInitSized(&sparse_rek, sparse->col_cnt, sparse->row_cnt) && maSolutionInit(&sol_forchild))
    if (maMatrixCopy(sparse_rek, sparse))
      rtc++;
      
  statistic->time_msec = t_process_msec();
  if (debug)
    (void) maMatrixProgressOut(0.0);

  if (rtc != 0)
    rtc = maMatrixCoverRek(sparse_rek, sol_forchild, result, weight, 0, cost_max, 1, statistic);
  
  statistic->time_msec = (t_process_msec() - statistic->time_msec);

  (void) maMatrixDestroy(sparse_rek);
  (void) maSolutionDestroy(sol_forchild);

  if (debug)
  {
    tprintf("\n# rec. cnt. = %d\n# rec. depth = %d", statistic->recursion_cnt, statistic->recursion_depth_max);
    tprintf("\n# greedy    = %d\n# partitions = %d", statistic->use_greedy, statistic->partition_build_cnt);
    tprintf("\n# gimpels   = %d", statistic->gimpel_reduction_cnt);
  }
  
  if (rtc == 0)
  {
    /*
    fprintf(stderr, "\nSpeicherfehler oder Ueberdeckung der Matrix (selbst trivial) nicht moeglich!");
    */
  }
  else
    if (!maSolutionVerify(sparse, result))
    {
      /*
      fprintf(stderr, "\nUeberdeckung ist nicht gegeben!");
      */
      rtc = 0;
    }
      
  if (debug)
  {
    for (f_field = result->row->fieldptr_firstcol; f_field != NULL; f_field = f_field->fieldptr_nextcol)
    {
      f_col = sparse->cols[f_field->col_no];
      statistic->literals += f_col->cnt;
    }
    tprintf("\n# literals  = %d", statistic->literals);

    if ((statistic->time_msec /1000) > 2)
      tprintf("\n# elapsed time solving part.-red. cover => %02d:%02d", (statistic->time_msec/60000), ((statistic->time_msec%60000)/1000));
    else
      tprintf("\n# elapsed time solving part.-red. cover => %d msec.", statistic->time_msec);
  }
  
  return rtc;
}

/* ----- maMatrixCoverRek ----------------------------- */
int maMatrixCoverRek (ma_ptr2matrix sparse, ma_ptr2solution sol_fromparent, ma_ptr2solution sol_toparent, int *weight, int cost_min, int cost_max, int recursion_depth, ma_ptr2statistic statistic)
{
  ma_ptr2solution sol_forchild, sol_fromchild;
  ma_ptr2matrix sparse_forchild;
  int rtc, branching_col, curr_mincost, indep_cost;

  /* Die Statistik-Counter hochzaehlen und ggf. die Statistik-Max.Rek.-Tiefe aktualisieren */
  statistic->recursion_cnt++;
  if (recursion_depth > statistic->recursion_depth_max)
    statistic->recursion_depth_max = recursion_depth;

  /* Die Matrix reduzieren (also auf Essentialitaet, Dominanzen ueberpruefen) und wenn die Loesung inzw. zu teuer geworden 
    (oder eine fehlerhafte Speicheranforderung stattgefunden hat) ist Rekursion abbrechen */

  /* maReduceClassic gibt die den Code zurueck: 
     1 = ok, 0 = bound erreicht, bzw. zu teuer geworden durch ess.select., -1 Speicherfehler */
  rtc = maMatrixReduceClassic(sparse, sol_fromparent, weight, cost_max);
  if (rtc <= 0)
  {
    if (rtc < 0)
      return (0);
    else
      return (1);
  }

  /* Ist die Matrix "vollstaendig reduziert worden ? Rekursionabbruch */ 
  if (sparse->row_cnt == 0)  
  {
    rtc = maSolutionCopy(sol_toparent, sol_fromparent);
    return rtc;
  }
  
/* Definition in Header-Datei: matrix.h */
#ifndef NOGIMPEL
  /* Die Matrix reduzieren (also auf Gimpelreduktionen ueberpruefen) */ 
  rtc = maMatrixReduceGimpel(sparse, sol_fromparent, sol_toparent, weight, cost_min, cost_max, recursion_depth, statistic);
  if (rtc != 0)
  {
    if (rtc < 0)
    {
      (void) maSolutionClear(sol_toparent);
      return (rtc + 2);
    }
    else
      return (1);
  }
#endif

  /* Bestimmen der Verzweigungsspalte (in ganz erheblichen Masse entscheident fuer kleine Rekursionsbaeume */
  branching_col = maMatrixSelectBranching(sparse, weight, &indep_cost);
  if (branching_col < 0)
    return (0);
    
  /* Neue Kostenabschaetzung curr_mincost (wenn Grenze nicht ueberschritten werden diese Kosten an Kinder weitergegeben */
  curr_mincost = (((sol_fromparent->cost + indep_cost) > cost_min) ? (sol_fromparent->cost + indep_cost) : cost_min);

  /* Bisherige Loesung schon zu teuer ? */
  if (curr_mincost >= cost_max)
    return (1);

  /* Ist ein Leaf erreicht ? Rekursion - Endbedingung! */
  if (sparse->row_cnt == 0)  
    return maSolutionCopy(sol_toparent, sol_fromparent);
    
  { /* Inline-Anweisung zur Reduzierung der Stackgroesse */
    ma_ptr2matrix sparse_left, sparse_right;
    ma_ptr2field field;
    float offset_parent, threshold_parent;
    unsigned int level_parent, progress_parent;
            
    /* Laesst sich die Matrix in zwei Teilmatritzen zerlegen? */
    rtc = maMatrixPartitionBlocks(sparse, &sparse_left, &sparse_right);
    
    if (rtc != 0)
    {
      /* Ist bei der Partitionierung ein Speicherfehler aufgetreten? Ja: Abbruch!*/
      if (rtc < 0)
      {
        (void) maMatrixVADestroy(2, sparse_left, sparse_right);
        return (0);
      }    
    
      statistic->partition_build_cnt++;

      offset_parent = statistic->partitions_offset;
      threshold_parent = statistic->partitions_threshold;
      level_parent = statistic->partitions_level;
      progress_parent = statistic->progress;
      
      statistic->partitions_level = recursion_depth - statistic->gimpels_above - 1;
      
      /* Duplikate der Loesung fuer Rekursion anfertigen */
      rtc = 0;  
      if (maSolutionVAInit(2, &sol_forchild, &sol_fromchild))
        if (maSolutionCopy(sol_forchild, sol_fromparent))
          rtc++;    
        
      /* Gab es Probleme mit dem Speicher ? */
      if (rtc == 0)
      {
        /* Speicherfehler und Abbruch */
        (void) maMatrixVADestroy(2, sparse_left, sparse_right);
        (void) maSolutionVADestroy(2, sol_forchild, sol_fromchild);
        return (0);
      }

      if (statistic->partitions_above == 0)
      {
        statistic->partitions_threshold = ((float)(statistic->progress) / (float)(1 << statistic->debug));
        statistic->partitions_offset = ((float)sparse_left->row_cnt / (float)(sparse->row_cnt)); 

        if (statistic->partitions_level > 0) 
          statistic->partitions_offset /= (float)(1 << statistic->partitions_level);
      }
      else
      {
       if (statistic->partitions_level > 0)
          statistic->partitions_threshold += (offset_parent * ((float)((statistic->progress << level_parent) & ((1 << statistic->debug) - 1)) / (float)(1 << statistic->debug)));
      
        statistic->partitions_offset = offset_parent * ((float)sparse_left->row_cnt / (float)(sparse->row_cnt * (1 << (statistic->partitions_level - level_parent))));
      }
        
      statistic->partitions_above++;   
  
      /* I.) Linken Teil der eigentlichen Matrix in die Rekursion schicken */
      rtc = maMatrixCoverRek(sparse_left, sol_forchild, sol_fromchild, weight, sol_fromparent->cost, cost_max, (recursion_depth), statistic);

      (void) maSolutionDestroy(sol_forchild);

      if (!rtc)
      {
        (void) maMatrixVADestroy(2, sparse_left, sparse_right);
        (void) maSolutionDestroy(sol_fromchild);
        return (0);
      }  

      for (field = sol_fromchild->row->fieldptr_firstcol; field != NULL; field = field->fieldptr_nextcol)
        if (!maSolutionAddCol(sol_toparent, ((weight == NULL) ? 1 : weight[field->col_no]), field->col_no))   
        {
          (void) maMatrixVADestroy(2, sparse_left, sparse_right);
          (void) maSolutionDestroy(sol_fromchild);
          return (0);
        }  
        
      (void) maSolutionClear(sol_fromchild);

      /* Hier wird der Fortschrittsanzeiger auf die Position vor Eintritt in die linke Teilmatritze zurueckgestellt */
      statistic->progress = progress_parent;
      statistic->partitions_threshold += statistic->partitions_offset;

      /* Gleiche Anweisung wie oben (... == 0), hier wurde der Counter aber schon um eins hochgezaehlt */
      if (statistic->partitions_above == 1)  
      {
        statistic->partitions_offset = ((float)sparse_right->row_cnt / (float)(sparse->row_cnt)); 
        if (statistic->partitions_level > 0)
          statistic->partitions_offset /= (float)(1 << statistic->partitions_level);
      }
      else
        statistic->partitions_offset = offset_parent * ((float)sparse_right->row_cnt / (float)(sparse->row_cnt * (1 << (statistic->partitions_level - level_parent))));
        
      /* II.) Rechten Teil der eigentlichen Matrix in die Rekursion schicken */
      rtc = maMatrixCoverRek(sparse_right, sol_toparent, sol_fromchild, weight, (sol_toparent->cost > curr_mincost ? sol_toparent->cost : curr_mincost), cost_max, (recursion_depth), statistic);

      statistic->partitions_above--;     

      statistic->partitions_threshold = threshold_parent; 
      statistic->partitions_offset = offset_parent;
      statistic->partitions_level = level_parent;

      if (!rtc)
      {
        (void) maMatrixVADestroy(2, sparse_left, sparse_right);
        (void) maSolutionDestroy(sol_fromchild);
        return (0);
      }  

      rtc = maSolutionCopy(sol_toparent, ((sol_fromchild->cost > 0 && sol_fromchild->cost < cost_max) ? sol_fromchild : sol_fromparent));
      (void) maSolutionDestroy(sol_fromchild);
      (void) maMatrixVADestroy(2, sparse_left, sparse_right);
      return rtc;  
    }
  }

  /* Duplikate der Loesung und der eigentlichen Matrix fuer Rekursion anfertigen */
  rtc = 0;  
  if (maMatrixInit(&sparse_forchild) && maSolutionVAInit(2, &sol_forchild, &sol_fromchild))
    if (maMatrixCopy(sparse_forchild, sparse) && maSolutionCopy(sol_forchild, sol_fromparent))
      if (maMatrixReduceAndSolutionAccept(sparse_forchild, sol_forchild, ((weight == NULL) ? 1 : weight[branching_col]), branching_col))
        rtc++;    

  /* Gab es Probleme mit dem Speicher ? */
  if (rtc == 0)
  {
    /* Speicherfehler und Abbruch */
    (void) maSolutionVADestroy(2, sol_forchild, sol_fromchild);
    (void) maMatrixDestroy(sparse_forchild);
    return (0);
  }

  /* Rekursion I  */
  rtc = maMatrixCoverRek(sparse_forchild, sol_forchild, sol_fromchild, weight, curr_mincost, cost_max, (recursion_depth+1), statistic);

  /* Wenn keine gueltige Loesung zustande kam, dann werden der Ansatz verworfen (und fuehrt nicht zu keinen Problemen) */
  if (!maSolutionVerify(sparse, sol_fromchild))
    (void) maSolutionClear(sol_fromchild);

  (void) maSolutionDestroy(sol_forchild);
  (void) maMatrixDestroy(sparse_forchild);
  if (!rtc)
  {
    (void) maSolutionDestroy(sol_fromchild);
    return (0);
  }  

  /* Ist eine biligere "<cost_max" Loesung zustande gekommen ? */
  if (sol_fromchild->cost > 0 && sol_fromchild->cost < cost_max)
  {
    if (!maSolutionCopy(sol_toparent, sol_fromchild))
      return (0);  
    (void) maSolutionDestroy(sol_fromchild);
    cost_max = sol_toparent->cost;

    /* Bevor die zweite Rekursion gestartet wird: Ist theoretisch minimale Loesung bereits gefunden ? */   
    if (cost_max == curr_mincost)
      return (1);
  }
  else
    (void) maSolutionDestroy(sol_fromchild);
  
  /* Wenn heuristische Greedy Loesung gewuenscht wurde wird hier wieder returniert (und der zweite Baum ausgelassen) */
  if (statistic->use_greedy)
    return (1);
  else
  {
    int tmp = recursion_depth - statistic->gimpels_above;     
    if (tmp < statistic->debug)
    { 
      /* Wird gerade in einem Teilzweig einer Partitionierung gearbeitet ?*/
      if (statistic->partitions_above > 0)
      {
        float progress_local;
        statistic->progress |= (1 << (statistic->debug - tmp));
        statistic->progress &= (1 << statistic->debug) - (1 << (statistic->debug - tmp));   
        progress_local = ( (float)((statistic->progress << statistic->partitions_level) & ((1 << statistic->debug) - 1)) / (float)(1 << statistic->debug));
        (void) maMatrixProgressOut(statistic->partitions_threshold + ((float)progress_local * statistic->partitions_offset));
      }
      else
      {
        statistic->progress |= (1 << (statistic->debug - tmp));
        statistic->progress &= (1 << statistic->debug) - (1 << (statistic->debug - tmp));   
        (void) maMatrixProgressOut(( (float)statistic->progress)/ (float)(1 << statistic->debug));
      }
    }
  }
 
  /* Duplikate der Loesung und der eigentlichen Matrix fuer Rekursion anfertigen */
  rtc = 0;  
  if (maMatrixInitSized(&sparse_forchild, sparse->col_cnt, sparse->row_cnt) && maSolutionVAInit(2, &sol_forchild, &sol_fromchild))
    if (maMatrixCopy(sparse_forchild, sparse) && maSolutionCopy(sol_forchild, sol_fromparent))
      rtc++;  

  /* Gab es Probleme mit dem Speicher ? */
  if (rtc == 0)
  {
    /* Speicherfehler und Abbruch */
    (void) maSolutionVADestroy(2, sol_forchild, sol_fromchild);
    (void) maMatrixDestroy(sparse_forchild);
    return (0);
  }

  (void) maMatrixReduceAndSolutionReject(sparse_forchild, branching_col);

  /* Rekursion II  */
  rtc = maMatrixCoverRek(sparse_forchild, sol_forchild, sol_fromchild, weight, curr_mincost, cost_max, (recursion_depth+1), statistic);

  /* Wenn keine gueltige Loesung zustande kam, dann werden der Ansatz verworfen (und fuehrt nicht zu keinen Problemen) */
  if (!maSolutionVerify(sparse, sol_fromchild))
    (void) maSolutionClear(sol_fromchild);

  (void) maSolutionDestroy(sol_forchild);
  (void) maMatrixDestroy(sparse_forchild);

  if (!rtc)
  {
    (void) maSolutionDestroy(sol_fromchild);
    return (0);
  }  
   
  /* Ergebnisse vergleichen ... */  
  if (sol_fromchild->cost == 0)
  {
    (void) maSolutionDestroy(sol_fromchild);
    rtc = 1;
  }
  else
    if (sol_toparent->cost == 0)
    {
      rtc = maSolutionCopy(sol_toparent, sol_fromchild);
      (void) maSolutionDestroy(sol_fromchild);
    }
    else
      if (sol_fromchild->cost > sol_toparent->cost)
        (void) maSolutionDestroy(sol_fromchild);
      else
      {
        rtc = maSolutionCopy(sol_toparent, sol_fromchild);
        (void) maSolutionDestroy(sol_fromchild);
      }

  return rtc;  
}

/* ----- maMatrixDebug1 ----------------------------- */
void maMatrixDebug1 (ma_ptr2matrix sparse)
{
  (void) maMatrixDebugSol1(sparse, NULL); 
}

/* ----- maMatrixDebug2 ----------------------------- */
void maMatrixDebug2 (ma_ptr2matrix sparse)
{
  (void) maMatrixDebugSol2(sparse, NULL); 
}

/* ----- maMatrixDebugSol1 -------------------------- */
void maMatrixDebugSol1 (ma_ptr2matrix sparse, ma_ptr2solution solution)
{
  FILE *fp_hlp;

  if ( fp_out == NULL )
    fp_out = stderr;

  fp_hlp = fp_out;
  fp_out = stderr;

  (void) maMatrixShowSol1(sparse, solution);

  fp_out = fp_hlp;
}

/* ----- maMatrixDebugSol2 -------------------------- */
void maMatrixDebugSol2 (ma_ptr2matrix sparse, ma_ptr2solution solution)
{
  FILE *fp_hlp;

  if ( fp_out == NULL )
    fp_out = stderr;

  fp_hlp = fp_out;
  fp_out = stderr;

  (void) maMatrixShowSol2(sparse, solution);

  fp_out = fp_hlp;
}

/* ----- maMatrixDeleteCol -------------------------- */
void maMatrixDeleteCol (ma_ptr2matrix sparse, int h_col)
{
  ma_ptr2col f_col;
  ma_ptr2row f_row;
  ma_ptr2field field, field_next;
  int f_rowno, letzte = -1;
  
  f_col = sparse->cols[h_col];
  if (f_col != NULL)
  {
    /* Alle Felder in der Column loeschen */
    for (field = f_col->fieldptr_firstrow; field != NULL && f_col->cnt > 0; field = field_next)
    {
      field_next = field->fieldptr_nextrow;

      f_row = sparse->rows[field->row_no];
      
      /* Jedes Feld im Zeilenbezug unlinken und dann loeschen */
      /* Vorderen Bezug des loesen */
      if (field->fieldptr_prevcol == NULL)
        f_row->fieldptr_firstcol = field->fieldptr_nextcol;
      else
        field->fieldptr_prevcol->fieldptr_nextcol = field->fieldptr_nextcol;

      /* Hinteren Bezug des loesen */
      if (field->fieldptr_nextcol == NULL)
        f_row->fieldptr_lastcol = field->fieldptr_prevcol;
      else
        field->fieldptr_nextcol->fieldptr_prevcol = field->fieldptr_prevcol;

      (void) maFieldDestroy(field);
      f_row->cnt--;
      f_col->cnt--;

      /* Wenn die referenzierte Zeile nur ein Feld besass kann sie geloescht werden */ 
      if (f_row->cnt == 0)
      {
        f_rowno = f_row->no;
        
        /* Voerderen Bezug der Zeile loesen */
        if (f_row->rowptr_prev == NULL)
          sparse->rowptr_first = f_row->rowptr_next;
        else
          f_row->rowptr_prev->rowptr_next = f_row->rowptr_next;

        /* Hinteren Bezug der Zeile loesen */
        if (f_row->rowptr_next == NULL)
          sparse->rowptr_last = f_row->rowptr_prev;
        else
          f_row->rowptr_next->rowptr_prev = f_row->rowptr_prev;

        /* Zeile aus Matrix und aus der Summe der Zeilen herausnehmen, dann selbst loeschen */
        sparse->row_cnt--;
        sparse->rows[f_rowno] = NULL;

        f_row->fieldptr_firstcol = NULL;
        f_row->fieldptr_lastcol = NULL;

        free(f_row);
      }
    }
    
    /* Den voerderen Bezug der Spalte unlinken */
    if (f_col->colptr_prev == 0)
      sparse->colptr_first = f_col->colptr_next;
    else
      f_col->colptr_prev->colptr_next = f_col->colptr_next;

    /* Den hinteren Bezug der Spalte unlinken */
    if (f_col->colptr_next == 0)
      sparse->colptr_last = f_col->colptr_prev;
    else
      f_col->colptr_next->colptr_prev = f_col->colptr_prev;

    /* Column aus Matrix und aus der Summe der Spalten herausnehmen, dann selbst loeschen */
    sparse->cols[h_col] = NULL;
    sparse->col_cnt--;

    f_col->fieldptr_firstrow = NULL;
    f_col->fieldptr_lastrow = NULL;
    f_col->cubeptr = NULL;

    free(f_col);
  }
}

/* ----- maMatrixDeleteFieldByColAndRow ------------- */
void maMatrixDeleteFieldByColAndRow (ma_ptr2matrix sparse, int h_col, int h_row)
{
  ma_ptr2field field;
  
  (void) maMatrixFindField(sparse, &field, h_col, h_row);

  if (field != NULL)
    (void) maMatrixDeleteFieldByPointer(sparse, field);
}

/* ----- maMatrixDeleteFieldByPointer --------------- */
void maMatrixDeleteFieldByPointer (ma_ptr2matrix sparse, ma_ptr2field h_fieldptr)
{
  ma_ptr2col f_col;
  ma_ptr2row f_row;

  if (h_fieldptr != NULL)
  {
    f_col = sparse->cols[h_fieldptr->col_no];
    f_row = sparse->rows[h_fieldptr->row_no];

    /* Den Spaltenbezug aktualisieren und wenn kein Feld mehr vorhanden ist die Spalten loeschen */

    /* Vorderen Bezug des loesen */
    if (h_fieldptr->fieldptr_prevrow == 0)
      f_col->fieldptr_firstrow = h_fieldptr->fieldptr_nextrow;
    else
      h_fieldptr->fieldptr_prevrow->fieldptr_nextrow = h_fieldptr->fieldptr_nextrow;

    /* Hinteren Bezug des loesen */
    if (h_fieldptr->fieldptr_nextrow == 0)
      f_col->fieldptr_lastrow = h_fieldptr->fieldptr_prevrow;
    else
      h_fieldptr->fieldptr_nextrow->fieldptr_prevrow = h_fieldptr->fieldptr_prevrow;

    f_col->cnt--;

    if (f_col->cnt == 0)
      (void) maMatrixDeleteCol(sparse, f_col->no);

    /* Den Zeilenbezug aktualisieren und wenn kein Feld mehr vorhanden ist die Zeile loeschen */

    /* Vorderen Bezug des loesen */
    if (h_fieldptr->fieldptr_prevcol == 0)
      f_row->fieldptr_firstcol = h_fieldptr->fieldptr_nextcol;
    else
      h_fieldptr->fieldptr_prevcol->fieldptr_nextcol = h_fieldptr->fieldptr_nextcol;

    /* Hinteren Bezug des loesen */
    if (h_fieldptr->fieldptr_nextcol == 0)
      f_row->fieldptr_lastcol = h_fieldptr->fieldptr_prevcol;
    else
      h_fieldptr->fieldptr_nextcol->fieldptr_prevcol = h_fieldptr->fieldptr_prevcol;

    f_row->cnt--;

    if (f_row->cnt == 0)
      (void) maMatrixDeleteRow(sparse, f_row->no);
      
    /* Das Feld selbst wieder freigeben */      
    (void) maFieldDestroy(h_fieldptr);
  }
}

/* ----- maMatrixDeleteRow -------------------------- */
void maMatrixDeleteRow (ma_ptr2matrix sparse, int h_row)
{
  ma_ptr2field field, field_next;
  ma_ptr2col f_col;
  ma_ptr2row f_row;
  int f_colno, letzte = -1;
  
  f_row = sparse->rows[h_row];
  if (f_row != NULL)
  {
    /* Alle Felder in der Zeile loeschen */
    for (field = f_row->fieldptr_firstcol; field != NULL  && f_row->cnt > 0; field = field_next)
    {
      field_next = field->fieldptr_nextcol;

      f_col = sparse->cols[field->col_no];
      
      /* Den Spaltenbezug aktualisieren und dann das Feld loeschen */
      /* Vorderen Bezug des loesen */
      if (field->fieldptr_prevrow == NULL)
        f_col->fieldptr_firstrow = field->fieldptr_nextrow;
      else
        field->fieldptr_prevrow->fieldptr_nextrow = field->fieldptr_nextrow;

      /* Hinteren Bezug des loesen */
      if (field->fieldptr_nextrow == NULL)
        f_col->fieldptr_lastrow = field->fieldptr_prevrow;
      else
        field->fieldptr_nextrow->fieldptr_prevrow = field->fieldptr_prevrow;

      (void) maFieldDestroy(field);
      f_col->cnt--;
      f_row->cnt--;

      /* Wenn die referenzierte Spalte nur ein Feld besass kann sie geloescht werden */ 
      if (f_col->cnt == 0)
      {
        f_colno = f_col->no;

        /* Den voerderen Bezug der Spalte unlinken */
        if (f_col->colptr_prev == 0)
          sparse->colptr_first = f_col->colptr_next;
        else
          f_col->colptr_prev->colptr_next = f_col->colptr_next;

        /* Den hinteren Bezug der Spalte unlinken */
        if (f_col->colptr_next == 0)
          sparse->colptr_last = f_col->colptr_prev;
        else
          f_col->colptr_next->colptr_prev = f_col->colptr_prev;

        /* Column aus Matrix und aus der Summe der Spalten herausnehmen, dann selbst loeschen */
        sparse->cols[f_colno] = NULL;
        sparse->col_cnt--;

        f_col->fieldptr_firstrow = NULL;
        f_col->fieldptr_lastrow = NULL;
        f_col->cubeptr = NULL;

        free(f_col);
      }
    }
  
    /* Voerderen Bezug der Zeile loesen */
    if (f_row->rowptr_prev == NULL)
      sparse->rowptr_first = f_row->rowptr_next;
    else
      f_row->rowptr_prev->rowptr_next = f_row->rowptr_next;

    /* Hinteren Bezug der Zeile loesen */
    if (f_row->rowptr_next == NULL)
      sparse->rowptr_last = f_row->rowptr_prev;
    else
      f_row->rowptr_next->rowptr_prev = f_row->rowptr_prev;

    /* Zeile aus Matrix und aus der Summe der Zeilen herausnehmen, dann selbst loeschen */
    sparse->row_cnt--;
    sparse->rows[h_row] = NULL;

    f_row->fieldptr_firstcol = NULL;
    f_row->fieldptr_lastcol = NULL;

    free(f_row);
  }
}

/* ----- maMatrixDestroy ---------------------------- */
void maMatrixDestroy (ma_ptr2matrix sparse)
{
  ma_ptr2col f_col, f_col_next;
  ma_ptr2row f_row, f_row_next;
  
  if (sparse != NULL)
  {
    /* Innerhalb der maMAtrixDeleteCol - Fkt. werden die Rows automatisch mitgeloescht, sobald sie keine Felder mehr enthalten */
    for (f_col = sparse->colptr_first; f_col != NULL; f_col = f_col_next)
    {
      f_col_next = f_col->colptr_next;
      (void) maMatrixDeleteCol(sparse, f_col->no);
    }

    /* Die durch maMatrixExpand allokierten Speicherbereiche wieder freigeben */
    free(sparse->cols);
    free(sparse->rows); 
  
    /* Die Struktur der Matrix selbst wieder freigeben */
    free(sparse);
  }
}

/* ----- maMatrixExpand ----------------------------- */
int maMatrixExpand (ma_ptr2matrix sparse, int cols, int rows)
{
  int i, rtc = 1;
  
  /* Muss der Platz fuer neue Spalten angefordert werden ? */
  if (cols >= sparse->col_init)
  {
    if ( sparse->cols == NULL )
      sparse->cols = (ma_ptr2col *)malloc(((cols + 1) * sizeof(ma_typ_col)));
    else
      sparse->cols = (ma_ptr2col *)realloc(sparse->cols, ((cols + 1) * sizeof(ma_typ_col)));
    
    if (sparse->cols != NULL)
    {  
      for (i = sparse->col_init; i <= cols; i++) 
        sparse->cols[i] = NULL;
      sparse->col_init = cols + 1;
    }
    else
      rtc = 0;
  }

  /* Muss der Platz fuer neue Zeilen angefordert werden ? */
  if (rows >= sparse->row_init && rtc != 0)
  {
    if ( sparse->rows == NULL )
      sparse->rows = (ma_ptr2row *)malloc(((rows + 1)* sizeof(ma_typ_row)));
    else
      sparse->rows = (ma_ptr2row *)realloc(sparse->rows, ((rows + 1)* sizeof(ma_typ_row)));
    if (sparse->rows != NULL)
    {  
      for (i = sparse->row_init; i <= rows; i++) 
        sparse->rows[i] = NULL;
      sparse->row_init = rows + 1;
    }
    else 
      rtc = 0;
  }
  
  return rtc;
}

/* ----- maMatrixFindField -------------------------- */
void maMatrixFindField (ma_ptr2matrix sparse, ma_ptr2field *field, int h_col, int h_row)
{
  ma_ptr2col f_col;
  ma_ptr2row f_row;

  f_col = sparse->cols[h_col];
  f_row = sparse->rows[h_row];

  if (f_row == NULL || f_col == NULL)
    *field = NULL;
  else
  {
    /* Wenn die Spalte weniger Elemente besitzt dann in der Spaltenverkettung suchen, sonst ... */
    if (f_col->cnt > f_row->cnt)
      (void) maRowFindField(f_row, field, h_col);
    else
      (void) maColFindField(f_col, field, h_row);
  }
}

/* ----- maMatrixFindIndepCols ---------------------- */
int maMatrixFindIndepCols (ma_ptr2matrix sparse, ma_ptr2matrix intersection, int *weight, ma_ptr2solution solution)
{
  ma_ptr2row f_row, f_bestrow, f_copyrow;
  ma_ptr2col f_col;
  ma_ptr2field field;
  int lo_weight, rtc = 1;
  
  while (intersection->row_cnt > 0)
  {
    /* Diejenige Zeile, die am wenigsten Eintraege (also Ueberschneidungen mit anderen Zeilen) besitzt auswaehlen */
    f_bestrow = intersection->rowptr_first;
    for (f_row = intersection->rowptr_first->rowptr_next; f_row != NULL; f_row = f_row->rowptr_next)
      if (f_row->cnt < f_bestrow->cnt)
        f_bestrow = f_row;
    
    /* Von der besten Zeile (also einem independent clique set) die Spalte mit dem kleinsten Gewicht auswaehlen */
    if (weight == NULL)
      lo_weight = 1;
    else
    {
      f_row = intersection->rows[f_bestrow->no];
      lo_weight = weight[f_row->fieldptr_firstcol->col_no];
      for (field = f_row->fieldptr_firstcol->fieldptr_nextcol; field != NULL; field = field->fieldptr_nextcol)
        if (weight[field->col_no] < lo_weight)
          lo_weight = weight[field->col_no]; 
    }
      
    /* Entgegen der ueblichen Verwendung wird hier in der Rueckgabe: "solution->row" eine Reihe von ZEILEN-NUMMERN! protokolliert */
    solution->cost += lo_weight;
    if (!maRowInsertField(solution->row, f_bestrow->no))
    {
      rtc = 0;
      break;    
    }
    
    /* Loeschen der linear abhaengigen Zeilen aus der intersection-Matrix */
    if (maRowInit(&f_copyrow))
    {
      if (!maRowCopy (f_copyrow, f_bestrow))
      {
        rtc = 0;
        break;
      }
      
      for (field = f_copyrow->fieldptr_firstcol; field != NULL; field = field->fieldptr_nextcol)
      {
        (void) maMatrixDeleteCol(intersection, field->col_no);
        (void) maMatrixDeleteRow(intersection, field->col_no);
      }
      (void) maRowDestroy(f_copyrow);
    }
    else
    {
      rtc = 0;
      break;
    }
  }
  
  return rtc;
}

/* ----- maMatrixInit ------------------------------- */
int maMatrixInit (ma_ptr2matrix *sparse)
{
  int rtc = 0;
  
  *sparse = (ma_ptr2matrix)malloc(sizeof(struct _matrix_struct));
  if (*sparse != NULL)
  {
    (*sparse)->cols = NULL;
    (*sparse)->rows = NULL;
    (*sparse)->colptr_first = NULL;
    (*sparse)->colptr_last = NULL;
    (*sparse)->rowptr_first = NULL;
    (*sparse)->rowptr_last = NULL;
    (*sparse)->col_cnt = 0;
    (*sparse)->row_cnt = 0;
    (*sparse)->col_init = 0;
    (*sparse)->row_init = 0;
    rtc++;
  }
  return rtc;
}

/* ----- maMatrixInitSized -------------------------- */
int maMatrixInitSized (ma_ptr2matrix *sparse, int cols, int rows)
{
  int rtc = 0;
  
  if (maMatrixInit(sparse))
    rtc = maMatrixExpand(*sparse, cols, rows);

  return rtc;
}

/* ----- maMatrixIrredundant ------------------------ */
/* matrix is stored in the output part of cl */
/* marks the selected cubes with a flag */
int maMatrixDCL (pinfo *pi, dclist cl, int greedy, int lit_opt)
{
  ma_ptr2matrix ma_matrix;
  ma_ptr2solution ma_solution;
  ma_ptr2col ma_col;
  ma_ptr2field ma_field;
  int *weight = NULL;

  int i, j, rtc = 0;

  dclClearFlags(cl);
  
  if ( dclCnt(cl) == 0 )
    return 1;

  if ( dclCnt(cl) == 1 )
  {
    dclSetFlag(cl, 0);
    return 1;
  }

  if ( lit_opt != MA_LIT_NONE )
    weight = (int *)malloc(sizeof(int)*dclCnt(cl));


  if (maMatrixInitSized(&ma_matrix, dclCnt(cl), pi->out_cnt) && maSolutionInit(&ma_solution))
  {
    /* Ueberdeckungsmatrix aufbauen und part. red. cubes den einzelnen Spalten zuweisen ... */
    for (i = 0; i < dclCnt(cl); i++)
    {  
      for (j = 0; j < pi->out_cnt; j++)
        if (dcGetOut(dclGet(cl, i), j))
          if (maMatrixInsertField(ma_matrix, i, j) == 0)
            break;

      if (j < pi->out_cnt)
        break;
      (ma_matrix->cols[i])->cubeptr = dclGet(cl, i);

      if ( weight != NULL )
      {
        switch(lit_opt)
        {
          case MA_LIT_NONE:
            weight[i] = 1;
            break;
          case MA_LIT_SOP:
            weight[i] = dcGetLiteralCnt(pi, dclGet(cl, i));
            break;
          case MA_LIT_DICHOTOMY:
            weight[i] = dcGetDichotomyWeight(pi, dclGet(cl, i));
            break;
        }
      }
    }

          
    if (maMatrixCoverInit(ma_matrix, ma_solution, weight, greedy, 0)) /* Test: Debug-Stufen: 0 */
    {
      /* Alle gewaehlten part. red. hinzufuegen */
      for (ma_field = ma_solution->row->fieldptr_firstcol; ma_field != NULL; ma_field = ma_field->fieldptr_nextcol)
      {
        assert(ma_field->col_no <= ma_matrix->col_init);
        dclSetFlag(cl, ma_field->col_no);
        /*
        ma_col = ma_matrix->cols[ma_field->col_no];
        dclAdd(pi, cl_out, ma_col->cubeptr);
        */
      }
      rtc = 1;
    }

    /*
    if ( dclCopy(pi, cl_pr, cl_out) == 0 )
      rtc = 0;
    */
    
    if ( weight != NULL )
      free(weight);

    maMatrixDestroy(ma_matrix);
    maSolutionDestroy(ma_solution);
  }

  return rtc;
}


/* ----- maMatrixIrredundant ------------------------ */
/* cl_rc: additional list with cubes that MUST be covered by a single cube */
/*        use NULL if cl_rc is not used */
int maMatrixIrredundant (pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc, dclist cl_rc, int greedy, int lit_opt)
{
  ma_ptr2matrix ma_matrix;
  ma_ptr2solution ma_solution;
  ma_ptr2col ma_col;
  ma_ptr2field ma_field;
  int *weight = NULL;
  int is_literal;
  
  pinfo local_pi;
  dclist cl_array, cl_out;

  int i, j, rtc = 0;
  
  if ( dclCnt(cl_pr) <= 1 )
    return 1; /* es gibt nix zu tun */
    
    
  is_literal = 1;
  if ( lit_opt == MA_LIT_NONE )
    is_literal = 0;

  if (pinfoInit(&local_pi))
  {  
    if (pinfoInitProgress(&local_pi) && dclInitVA(2, &cl_out, &cl_array))
    {
      /* Bestimmen der Abhaengigkeiten unter den partiell redundanten Implikanten */
          
      if (dclIrredundantDCubeTMatrixRC(&local_pi, cl_array, pi, cl_es, cl_pr, cl_dc, cl_rc))
      {
        if (dclClearFlags(cl_pr) && dclClearFlags(cl_array))
        {          
          /* Hier werden alle illegalen Cubes (d.h. Leerspalten) entfernt */
          for (i = 0; i < dclCnt(cl_array); i++)
            if (dcIsOutIllegal(&local_pi, dclGet(cl_array, i)) != 0)
            {
              dclSetFlag(cl_pr, i);
              dclSetFlag(cl_array, i);
            }
              
          dclDeleteCubesWithFlag(pi, cl_pr);
          dclDeleteCubesWithFlag(&local_pi, cl_array);

          if (maMatrixInitSized(&ma_matrix, dclCnt(cl_array), local_pi.out_cnt) && maSolutionInit(&ma_solution))
          {
            /* Ueberdeckungsmatrix aufbauen und part. red. cubes den einzelnen Spalten zuweisen ... */
            if ( is_literal != 0 )
              weight = (int *)malloc(sizeof(int)*dclCnt(cl_array));
            for (i = 0; i < dclCnt(cl_array); i++)
            {  
              for (j = 0; j < local_pi.out_cnt; j++)
                if (dcGetOut(dclGet(cl_array, i), j))
                  if (maMatrixInsertField(ma_matrix, i, j) == 0)
                    break;

              if (j < local_pi.out_cnt)
                break;
              (ma_matrix->cols[i])->cubeptr = dclGet(cl_pr, i);
              if ( weight != NULL )
              {
                switch(lit_opt)
                {
                  case MA_LIT_NONE:
                    weight[i] = 1;
                    break;
                  case MA_LIT_SOP:
                    weight[i] = dcGetLiteralCnt(pi, dclGet(cl_pr, i));
                    break;
                  case MA_LIT_DICHOTOMY:
                    weight[i] = dcGetDichotomyWeight(pi, dclGet(cl_pr, i));
                    break;
                }
              }
            }
          
            if (maMatrixCoverInit(ma_matrix, ma_solution, weight, greedy, 0)) /* Test: Debug-Stufen: 0 */
            {
              /* Alle gewaehlten part. red. hinzufuegen */
              for (ma_field = ma_solution->row->fieldptr_firstcol; ma_field != NULL; ma_field = ma_field->fieldptr_nextcol)
              {
                assert(ma_field->col_no <= ma_matrix->col_init);
                ma_col = ma_matrix->cols[ma_field->col_no];
                dclAdd(pi, cl_out, ma_col->cubeptr);
              }
              rtc = 1;
            }
            
            if ( dclCopy(pi, cl_pr, cl_out) == 0 )
              rtc = 0;
            
            maMatrixDestroy(ma_matrix);
            maSolutionDestroy(ma_solution);
          }
        }
      }
      dclDestroyVA(2, cl_out, cl_array);
    }     
    pinfoDestroy(&local_pi);
  }
  
  if ( weight != NULL )
    free(weight);

  return rtc;
}


/* ----- maMatrixInsertField ------------------------ */
int maMatrixInsertField (ma_ptr2matrix sparse, int h_col, int h_row)
{
  ma_ptr2col nf_col;
  ma_ptr2row nf_row;
  ma_ptr2field newfield, nf_copy;
  int rtc = 1;

  if (h_col >= sparse->col_init || h_row >= sparse->row_init)
    if (maMatrixExpand(sparse, h_col, h_row) == 0)
      rtc = 0;
  
  nf_row = sparse->rows[h_row];
  nf_col = sparse->cols[h_col];

  /* Wenn die Zeile noch nicht existiert*/
  if (nf_row == NULL && rtc != 0)
  {
    if (maRowInit(sparse->rows + h_row))
    {
      nf_row = sparse->rows[h_row];
      (void) maMatrixLinkRow(sparse, h_row, nf_row);
    }
    else 
      rtc = 0;
  }    

  /* Wenn die Spalte noch nicht existiert*/
  if (nf_col == NULL && rtc != 0)
  {
    if (maColInit(sparse->cols + h_col))
    {
      nf_col = sparse->cols[h_col];
      (void) maMatrixLinkCol(sparse, h_col, nf_col);
    }
    else
      rtc = 0;
  }

  /* Das (evtl.) neue Feld der Matrix aufbauen */
  if (rtc != 0)
    if (maFieldInit(&newfield))
    {  
      nf_copy = newfield;
  
      /* Das Feld in die Zeile einfuegen */
      (void) maRowLinkField(nf_row, h_col, &newfield);

      /* Wenn das Feld neu war, auch noch in den Spaltenbezug integrieren, ansonsten das Feld wieder loeschen */
      if (newfield != nf_copy)
        (void) maFieldDestroy(nf_copy);
      else    
        (void) maColLinkField(nf_col, h_row, &newfield);
    }
    else
      rtc = 0;

  return rtc;
}

/* ----- maMatrixLinkCol ---------------------------- */
void maMatrixLinkCol (ma_ptr2matrix sparse, int h_colno, ma_ptr2col h_colptr)
{                                                                                                             
  ma_ptr2col lo_colptr;  

  h_colptr->no = h_colno;                                                                      
  sparse->col_cnt++;                                                                                         

  /* Fall 1.) Die Matrix enthaelt noch gar keine Spalten ... */
  if (sparse->colptr_last == NULL)                                                                                  
  {                                                                                                         
    h_colptr->colptr_prev = NULL;                                                                              
    h_colptr->colptr_next = NULL;                                                                              

    sparse->colptr_first = h_colptr;                                                                                
    sparse->colptr_last = h_colptr;                                                                                 
  }                                                                                                         
  else 
  {
    /* Fall 2.) Die spezifizierte Spalte ist letzte Spalte der Matrix und muss linksseitig in den Spaltenkontext der Matrix einsortiert werden ... */
    if (sparse->colptr_last->no < h_colno)                                                                  
    {                                                                                                       
      h_colptr->colptr_prev = sparse->colptr_last;                                                                       
      h_colptr->colptr_next = NULL; 

      sparse->colptr_last->colptr_next = h_colptr;                                                                       
      sparse->colptr_last = h_colptr;                                                                                 
    }                                                                                                       
    else 
    {
      /* Fall 3.) Die spezifizierte Spalte ist erste Spalte der Matrix und muss rechtsseitig in den Spaltenkontext der Matrix einsortiert werden ... */
      if (sparse->colptr_first->no > h_colno)                                                               
      {                                                                                                     
        h_colptr->colptr_prev = NULL;
        h_colptr->colptr_next = sparse->colptr_first;                                                                    

        sparse->colptr_first->colptr_prev = h_colptr;                                                                    
        sparse->colptr_first = h_colptr;                                                                              
      } 
      else                                                                                                
      {                                                                                                   
        /* Anfahren der Spalte im Spaltenkontext "von vorne" */
        if (h_colno <= ((sparse->colptr_last->no + sparse->colptr_first->no) / 2))
          for(lo_colptr = sparse->colptr_first; lo_colptr->no < h_colno; lo_colptr = lo_colptr->colptr_next);  
        else
        {
          /* Anfahren der Spalte "von hinten" */
          lo_colptr = sparse->colptr_last;
          if (h_colno != lo_colptr->no)
          {
            for(; lo_colptr->no >= h_colno; lo_colptr = lo_colptr->colptr_prev);  
            
            lo_colptr = lo_colptr->colptr_next;
          }
        }

        /* Fall 4.) Die spezifizierte Spalte muss beidseitig in den Spaltenkontext der Matrix einsortiert werden ... */
        if (lo_colptr->no > h_colno)                                                                       
        {                                                                                                 
          /* zu weit -> wieder eine Position nach voerne ruecken (auf alle Faelle ist jetzt bekannt, dass die Spalte noch nicht exisitierte) */
          lo_colptr = lo_colptr->colptr_prev;                                                                            

          h_colptr->colptr_prev = lo_colptr;                                                                         
          h_colptr->colptr_next = lo_colptr->colptr_next;                                                               

          lo_colptr->colptr_next->colptr_prev = h_colptr;                                                               
          lo_colptr->colptr_next = h_colptr;                                                                         
        }                                                                                                 
        else                                                                                              
          /* Fall 5.) Die spezifizierte Spalte exisitert bereits (dann war die Erheohung des Matrix-Counters unzulaessig)... */
          sparse->col_cnt--; 
          h_colptr = lo_colptr;                                                                                   
      }                                                                                                   
    }
  }
}          

/* ----- maMatrixLinkRow ---------------------------- */
void maMatrixLinkRow (ma_ptr2matrix sparse, int h_rowno, ma_ptr2row h_rowptr)
{                                                                                                             
  ma_ptr2row lo_rowptr;  

  h_rowptr->no = h_rowno;                                                                      
  sparse->row_cnt++;                                                                                         

  /* Fall 1.) Die Matrix enthaelt noch gar keine Zeilen ... */
  if (sparse->rowptr_last == NULL)                                                                                  
  {                                                                                                         
    h_rowptr->rowptr_prev = NULL;                                                                              
    h_rowptr->rowptr_next = NULL;                                                                              

    sparse->rowptr_first = h_rowptr;                                                                                
    sparse->rowptr_last = h_rowptr;                                                                                 
  }                                                                                                         
  else 
  {
    /* Fall 2.) Die spezifizierte Zeile ist letzte Zeile der Matrix und muss linksseitig in den Zeilenkontext der Matrix einsortiert werden ... */
    if (sparse->rowptr_last->no < h_rowno)                                                                  
    {                                                                                                       
      h_rowptr->rowptr_prev = sparse->rowptr_last;                                                                       
      h_rowptr->rowptr_next = NULL; 

      sparse->rowptr_last->rowptr_next = h_rowptr;                                                                       
      sparse->rowptr_last = h_rowptr;                                                                                 
    }                                                                                                       
    else 
    {
      /* Fall 3.) Die spezifizierte Zeile ist erste Zeile der Matrix und muss rechtsseitig in den Zeilenkontext der Matrix einsortiert werden ... */
      if (sparse->rowptr_first->no > h_rowno)                                                               
      {                                                                                                     
        h_rowptr->rowptr_prev = NULL;
        h_rowptr->rowptr_next = sparse->rowptr_first;                                                                    

        sparse->rowptr_first->rowptr_prev = h_rowptr;                                                                    
        sparse->rowptr_first = h_rowptr;                                                                              
      } 
      else                                                                                                
      {                                                                                                   
        /* Anfahren der Zeile im Zeilenkontext "von vorne" */
        if (h_rowno <= ((sparse->rowptr_last->no + sparse->rowptr_first->no) / 2))
          for(lo_rowptr = sparse->rowptr_first; lo_rowptr->no < h_rowno; lo_rowptr = lo_rowptr->rowptr_next);  
        else
        {
          /* Anfahren der Zeile "von hinten" */
          lo_rowptr = sparse->rowptr_last;
          if (h_rowno != lo_rowptr->no)
          {
            for(; lo_rowptr->no >= h_rowno; lo_rowptr = lo_rowptr->rowptr_prev);  
            
            lo_rowptr = lo_rowptr->rowptr_next;
          }
        }

        /* Fall 4.) Die spezifizierte Zeile muss beidseitig in den Zeilenkontext der Matrix einsortiert werden ... */
        if (lo_rowptr->no > h_rowno)                                                                       
        {                                                                                                 
          /* zu weit -> wieder eine Position nach voerne ruecken (auf alle Faelle ist jetzt bekannt, dass die Zeile noch nicht exisitierte) */
          lo_rowptr = lo_rowptr->rowptr_prev;                                                                            

          h_rowptr->rowptr_prev = lo_rowptr;                                                                         
          h_rowptr->rowptr_next = lo_rowptr->rowptr_next;                                                               

          lo_rowptr->rowptr_next->rowptr_prev = h_rowptr;                                                               
          lo_rowptr->rowptr_next = h_rowptr;                                                                         
        }                                                                                                 
        else                                                                                              
          /* Fall 5.) Die spezifizierte Zeile exisitert bereits (dann war die Erheohung des Matrix-Counters unzulaessig)... */
          sparse->row_cnt--; 
          h_rowptr = lo_rowptr;                                                                                   
      }                                                                                                   
    }
  }
}          

/* ----- maMatrixPartitionBlocks --------------------- */
int maMatrixPartitionBlocks (ma_ptr2matrix sparse, ma_ptr2matrix *sparse_left, ma_ptr2matrix *sparse_right)
{
  ma_ptr2matrix sparse_switch;
  ma_ptr2col f_col;
  ma_ptr2row f_row;
  //ma_ptr2field field;
  int no_cols = 0, no_rows = 0, rtc = 0;

  /* Zuruecksetzen der Flags fuer alle Spalten und Zeilen */
  for (f_col = sparse->colptr_first; f_col != NULL; f_col = f_col->colptr_next)
    f_col->visited = 0;

  for (f_row = sparse->rowptr_first; f_row != NULL; f_row = f_row->rowptr_next)
    f_row->visited = 0;

  /* Blieben "unbesuchte" Zeilen, so laesst sich die Matrix zerlegen */
  if (maMatrixVisitRow(sparse, sparse->rowptr_first, &no_cols, &no_rows))
  {
    if (!maMatrixInit(sparse_left) || !maMatrixInit(sparse_right))
      rtc = -2;
  
    for (rtc++, f_row = sparse->rowptr_first; f_row != NULL && rtc > 0; f_row = f_row->rowptr_next)
      if (!maMatrixCopyRow(((f_row->visited == 1) ? *sparse_left : *sparse_right), f_row))
        rtc = -1;
  }

  /* Dafuer sorgen, dass die zuerst bearbeitete, linke Teilmatrize die kleinere von beiden ist */
  if (rtc > 0)
    if ((*sparse_left)->col_cnt > (*sparse_right)->col_cnt && rtc != 0)
    {
      sparse_switch = *sparse_right;
      *sparse_right = *sparse_left;
      *sparse_left = sparse_switch;
    }

/*  if (rtc > 0)
    if (maMatrixPartitionCheck(*sparse_left, *sparse_right) == 1)
    {
      fprintf(stderr, "\n Achtung: Die Teilmatrizen sind nicht linear unabhaengig!!!\n");
      rtc = -1;
    } */

  return rtc;
}

/* ----- maMatrixPartitionCheck --------------------- */
int maMatrixPartitionCheck (ma_ptr2matrix sparse1, ma_ptr2matrix sparse2)
{
  ma_ptr2col f_col, f_col2;
  ma_ptr2row f_row;
  ma_ptr2field field;
  //field2;
  int rtc = 0;

  for (f_col = sparse1->colptr_first; f_col != NULL; f_col = f_col->colptr_next)
    f_col->visited = 0;

  for (f_row = sparse1->rowptr_first; f_row != NULL; f_row = f_row->rowptr_next)
  {
    for (field = f_row->fieldptr_firstcol; field != NULL; field = field->fieldptr_nextcol)
    {
      f_col = sparse1->cols[field->col_no];
      f_col->visited = 1;
    } 
  }

  for (f_col = sparse2->colptr_first; f_col != NULL; f_col = f_col->colptr_next)
    f_col->visited = 0;

  for (f_row = sparse2->rowptr_first; f_row != NULL; f_row = f_row->rowptr_next)
  {
    for (field = f_row->fieldptr_firstcol; field != NULL; field = field->fieldptr_nextcol)
    {
      f_col = sparse2->cols[field->col_no];
      f_col->visited = 1;
    }  
  }

  for (f_col = sparse1->colptr_first; f_col != NULL && rtc != 1; f_col = f_col->colptr_next)
  {
    f_col2 = ((f_col->no < sparse2->col_init) ? sparse2->cols[f_col->no] : NULL);
  
    if (f_col2 != NULL)
      if (f_col->visited == 1 && f_col2->visited == 1)
        rtc = 1;
  }
  
  for (f_col = sparse2->colptr_first; f_col != NULL && rtc != 1; f_col = f_col->colptr_next)
  {
    f_col2 = ((f_col->no < sparse1->col_init) ? sparse1->cols[f_col->no] : NULL);
  
    if (f_col2 != NULL)
      if (f_col->visited == 1 && f_col2->visited == 1)
        rtc = 1;
  }
  
  return rtc;  
}

/* ----- maMatrixProgressOut ------------------------- */
void maMatrixProgressOut (float progress)
{
  fprintf(stderr, "maMatrixCover %5.2f %%", (progress * 100));
  fprintf(stderr, "                      \r"); 
  fflush(stderr);
}

/* ----- maMatrixReduceAndSolutionAccept ------------- */
int maMatrixReduceAndSolutionAccept (ma_ptr2matrix sparse, ma_ptr2solution solution, int h_colweight, int h_colno)
{
  ma_ptr2col f_col;
  ma_ptr2field field, nextfield;
  int rtc = 1;

  f_col = sparse->cols[h_colno];
  
  for (field = f_col->fieldptr_firstrow; field != NULL; field = nextfield)
  {
    nextfield = field->fieldptr_nextrow;
    (void) maMatrixDeleteRow(sparse, field->row_no);
  }
  
  if (!maSolutionAddCol(solution, h_colweight, h_colno))
    rtc = 0;
    
  return rtc;
}

/* ----- maMatrixReduceAndSolutionReject ------------- */
void maMatrixReduceAndSolutionReject (ma_ptr2matrix sparse, int h_colno)
{
  (void) maMatrixDeleteCol(sparse, h_colno);
}

/* ----- maMatrixReduceClassic ----------------------- */
int maMatrixReduceClassic (ma_ptr2matrix sparse, ma_ptr2solution sol_sofar, int *weight, int cost_max)
{
  int red_cols, red_rows, red_ess, rtc = 1;

  do 
  {
    /* (void) maMatrixDebugSol1(sparse, NULL); */
   
    /* 1.) Dominierte Spalten aus der Matrix herausnehmen */
    red_cols = maMatrixReduceDominatedCols(sparse, weight);

    /* 2.) Essentielle Spalten auswaehlen, (incl. der dadurch abgedeckten Zeilen) aus der Matrix herausnehmen 
           und die Spaltennummern zur sol_sofar hinzufuegen (rtc: -2 Speicherfehler, -1 zu teuer, <= 0..n ok) */
    red_ess = maMatrixReduceEssentials(sparse, sol_sofar, weight, cost_max);
    if (red_ess < 0)
    {
      rtc = (red_ess + 1);
      break;
    }
    /* 3.) Dominierende Zeilen aus der Matrix herausnehmen */
    red_rows = maMatrixReduceDominatingRows(sparse);
    
  } while ((red_cols > 0) || (red_ess > 0) || (red_rows > 0));

/* Definition in Header-Datei: matrix.h */
#ifndef NOCOUDERT
  if (weight != NULL)
    (void) maMatrixReduceCoudert(sparse, weight, sol_sofar->cost, cost_max);
#endif

  return rtc;
}

/* ----- maMatrixReduceCoudert ---------------------- */
void maMatrixReduceCoudert (ma_ptr2matrix sparse, int *weight, int cost_curr, int cost_max)
{
  ma_ptr2col f_col, f_nextcol;
  
  /* Schleife ueber alle Spalten: Ist "cost_curr + weight[f_col->no] >= cost_max" ? Dann kann durch Auswaehlen von f_col 
     keine billigere als die bisher bekannte Loesung bestimmt werden und somit f_col aus der Matrix entfernt werden */
  for (f_col = sparse->colptr_first; f_col != NULL; f_col = f_nextcol)
  {
    f_nextcol = f_col->colptr_next;
    if ((cost_curr + weight[f_col->no]) >= cost_max)
      (void) maMatrixDeleteCol(sparse, f_col->no);
  }
}

/* ----- maMatrixReduceDominatedCols ---------------- */
int maMatrixReduceDominatedCols (ma_ptr2matrix sparse, int *weight)
{
  ma_ptr2col f_col, f_nextcol, f_compcol;
  ma_ptr2row f_row, f_smallestrow;
  ma_ptr2field field;
  int rtc = sparse->col_cnt;

  /* Schleife ueber alle Spalten: Wird f_col von einer anderen Spalte dominiert ? Dann kann f_col ggf. 
     (nach Vgl. der Gewichtungen) aus der Matrix entfernt werden */
     
  for (f_col = sparse->colptr_first; f_col != NULL; f_col = f_nextcol)
  {
    f_nextcol = f_col->colptr_next;
    
    /* Um Arbeit zu vermeiden wird die "kleinste" in dieser Spalte referenierte Zeile (= f_smallestrow) herausgesucht */
    f_smallestrow = sparse->rows[f_col->fieldptr_firstrow->row_no]; 
    for (field = f_col->fieldptr_firstrow->fieldptr_nextrow; field != NULL; field = field->fieldptr_nextrow)
    {
      f_row = sparse->rows[field->row_no];
      if (f_row->cnt > f_smallestrow->cnt)
        f_smallestrow = f_row;
    }

    /* Innerhalb der kleinsten Zeile werden nun alle vorhandenen Spalten geprueft */
    for (field = f_smallestrow->fieldptr_firstcol; field != NULL; field = field->fieldptr_nextcol)
    {
      f_compcol = sparse->cols[field->col_no];
      
      /* Sollte eine Gewichtung existieren, so duerfen billigere, bzw. niedriger gew., dominierte Spalten nicht geloescht werden */
      if (weight != NULL)
        if (weight[f_compcol->no] > weight[f_col->no])
          continue;

      /* Ueberpruefung, ob Spalten mehrfach identisch in der Matrix vorhanden sind. Nur wenn das angenommene (wg. cnt) Duplikat weiter rechts 
         liegt wird die betrachtete Spalte auf Identitaet untersucht (sonst haette sie uns ja schon frueher auffallen muessen!) */
      if (f_col->cnt == f_compcol->cnt && f_col->no < f_compcol->no) 
        if (maColContainment(f_col, f_compcol))
        {
          /* printf("col '%d' dominated by '%d', delete '%d'\n", f_col->no, f_compcol->no, f_col->no); */
          (void) maMatrixDeleteCol(sparse, f_col->no);
          break;
        }   
      
      /* Ueberpruefung auf Containment nur bei all den Spalten, die mehr Elemente enthalten (sonst koennten sie wohl kaum dominieren) */
      if (f_col->cnt < f_compcol->cnt) 
        if (maColContainment(f_col, f_compcol))
        {
          /* printf("col '%d' dominated by '%d', delete '%d'\n", f_col->no, f_compcol->no, f_col->no); */
          (void) maMatrixDeleteCol(sparse, f_col->no);
          break;
        }   
    }
  }
  rtc -= sparse->col_cnt;
  return rtc;
}

/* ----- maMatrixReduceDominatingRows --------------- */
int maMatrixReduceDominatingRows (ma_ptr2matrix sparse)
{
  ma_ptr2col f_col, f_smallestcol;
  ma_ptr2row f_row, f_comprow;
  ma_ptr2field field, nextfield;
  int rtc = sparse->row_cnt;

  /* Schleife ueber alle Zeilen: Wird f_row von einer anderen Spalte dominiert ? Dann kann diese aus der Matrix entfernt werden werden */
  for (f_row = sparse->rowptr_first; f_row != NULL; f_row = f_row->rowptr_next)
  {
    /* Um Arbeit zu vermeiden wird die "kleinste" in dieser Zeile referenierte Zeile (= f_smallestcol) herausgesucht */
    f_smallestcol = sparse->cols[f_row->fieldptr_firstcol->col_no]; 
    for (field = f_row->fieldptr_firstcol->fieldptr_nextcol; field != NULL; field = field->fieldptr_nextcol)
    {
      f_col = sparse->cols[field->col_no];
      if (f_col->cnt > f_smallestcol->cnt)
        f_smallestcol = f_col;
    }

    /* Innerhalb der kleinsten Spalte werden nun alle vorhandenen Zeilen geprueft */
    for (field = f_smallestcol->fieldptr_firstrow; field != NULL; field = nextfield)
    {
      nextfield = field->fieldptr_nextrow;
      f_comprow = sparse->rows[field->row_no];
      
      /* Ueberpruefung, ob Zeilen mehrfach identisch in der Matrix vorhanden sind. Nur wenn das angenommene (wg. cnt) Duplikat weiter unten 
         liegt wird die betrachtete Zeile auf Identitaet untersucht (sonst haette sie uns ja schon frueher auffallen muessen!) */
      if (f_row->cnt == f_comprow->cnt && f_row->no < f_comprow->no) 
        if (maRowContainment(f_row, f_comprow))
        {
          (void) maMatrixDeleteRow(sparse, f_comprow->no);
          break;
        }   
      
      /* Ueberpruefung auf Containment nur bei all den Spalten, die mehr Elemente enthalten (sonst koennten sie wohl kaum dominieren) */
      if (f_row->cnt < f_comprow->cnt) 
        if (maRowContainment(f_row, f_comprow))
        {
          (void) maMatrixDeleteRow(sparse, f_comprow->no);
          break;
        }   
    }
  }
  rtc -= sparse->row_cnt;
  return rtc;
}

/* ----- maMatrixReduceEssentials ------------------- */
int maMatrixReduceEssentials (ma_ptr2matrix sparse, ma_ptr2solution sol_sofar, int *weight, int cost_max)
{
  int rtc = 0, count = 0;
  ma_ptr2row f_row, f_ess;
  ma_ptr2field field;
  
  if (maRowInit(&f_ess))
  {
    /* 1.) Der Rowvektor f_ess wird mit den essentiellen Spalten belegt */
    for (f_row = sparse->rowptr_first; f_row != NULL && rtc >= 0; f_row = f_row->rowptr_next)
      if (f_row->cnt == 1)
        if (!maRowInsertField(f_ess, f_row->fieldptr_firstcol->col_no))
          rtc = -2;

    /* 2.) Der Rowvektor f_ess wird zur Loesung hinzugefuegt und die Matrix reduziert */
    for (field = f_ess->fieldptr_firstcol; field != NULL && rtc >= 0; field = field->fieldptr_nextcol)
    {
      if (maMatrixReduceAndSolutionAccept(sparse, sol_sofar, ((weight == NULL) ? 1 : weight[field->col_no]), field->col_no))
      {
        /* Wenn die Reduktion inzw. zu teuer geworden ist, dann bricht die Rekursion hier komplett ab */
/* ttt-Debug-Test Kostenoberschranke war: sol_sofar->cost >= cost_max und ist jetzt nur noch groesser als ... */
        if (sol_sofar->cost > cost_max && rtc >= 0)
          rtc = -1;
        count++;
        /* printf("essential '%d'\n", field->col_no); */
      }
      else
        rtc = -2;
    }
  }
  else
    rtc = -2;

  if (rtc == 0)
    rtc = count;

  (void) maRowDestroy(f_ess);

  return rtc;
}


/* ----- maMatrixReduceGimpel ----------------------- */
int maMatrixReduceGimpel (ma_ptr2matrix sparse, ma_ptr2solution sol_fromparent, ma_ptr2solution sol_toparent, int *weight, int cost_min, int cost_max, int recursion_depth, ma_ptr2statistic statistic)
{ 
  ma_ptr2col f_col1, f_col2, f_swapcol;
  ma_ptr2row f_row, f_rowcopy;
  ma_ptr2field field, field2;
  int f_colno1, f_colno2, f_rowno1, f_rowno2, gimpel = 0, rtc = 0;

  /* Bisherige Loesung schon zu teuer ? */
  if (((cost_min >= sol_fromparent->cost) ? cost_min : sol_fromparent->cost) >= cost_max)
  {
    return (-1);
  }
  
  /* Suche nach Moeglichkeiten fuer die Gimpelreduktion: Ein Zeile in der Matrix finden, die nur von genau zwei Spalten abgedeckt wird.
     Weiterhin darf eine der beiden abdeckenden Spalten wiederum fuer sich genau zwei Zeilen abdecken -> Gimpel!
           c  c
           o  o
           l  l
           1  2

row1 0  0  1  1  0  0  0  0  0  0  0
row2 ?b ?b 1  0  ?b ?b ?b ?b ?b ?b ?b   (die Null ergibt sich implizit, weil die erste Spalte  
     ?  ?  0  ?a ?  ?  ?  ?  ?  ?  ?     sonst vorher schon als von der zweiten Spalte 
     ?  ?  0  ?a ?  ?  ?  ?  ?  ?  ?     "dominiert" wegreduziert worden waere)
     ?  ?  0  ?a ?  ?  ?  ?  ?  ?  ?  
     ?  ?  0  ?a ?  ?  ?  ?  ?  ?  ?    (auf die Bedeutung von ?a und ?b wird weiter unten eingegangen)
     ?  ?  0  ?a ?  ?  ?  ?  ?  ?  ?  */
     
  for (f_row = sparse->rowptr_first; f_row != NULL; f_row = f_row->rowptr_next)
  {
    if (f_row->cnt == 2)
    {
      /* Wenn die betreffende Zeile nur zwei Felder enthaelt beide referenzierten Zeilen (in f_col1 und f_col2) festhalten */
      f_col1 = sparse->cols[f_row->fieldptr_firstcol->col_no]; 
      f_col2 = sparse->cols[f_row->fieldptr_lastcol->col_no]; 
   
      if (f_col1->cnt == 2)
      {
        /* Modellfall gefunden! Siehe oben! */
        gimpel = 1;
        break;
      }
      else
        /* die beiden Spalten so vertauschen, dass immer col1 die beiden und col2 nur eine der beiden Zeilen abdeckt */
        if (f_col2->cnt == 2)
        {
          f_swapcol = f_col1;
          f_col1 = f_col2;
          f_col2 = f_swapcol;
          gimpel = 1;
          break;
        }
      }
  }
                    
  if (gimpel)
  {
/* ma_ptr2matrix sparse_forchild;*/
    statistic->gimpel_reduction_cnt++;

    /* Die Spaltennummern festhalten. Beide Spalten werden geloescht, die Restmatrix weiterverarbeitet und zum Schluss wird
       eine der beiden der Loesung hinzugefuegt */
    f_colno1 = f_col1->no;
    f_colno2 = f_col2->no;

    /* Die Row1 (bzw. Row-No.: 1) enthaelt immer das Gimpel-Feld */
    f_rowno1 = f_row->no;
    f_rowno2 = f_col1->fieldptr_lastrow->row_no;
      
    /* Wenn das zweite Feld in der f_col1 das Gimpelfeld enthielt, muss die zweite Zeile auf das erste Feld in f_col1 gesetzt werden */
    if (f_rowno1 == f_rowno2)
      f_rowno2 = f_col1->fieldptr_firstrow->row_no;
        
    /* Eine Kopie der zweiten Zeile anlegen, damit nach Bestimmung des Ergebnisses der Submatrix entschieden werden kann, welche Spalte 
       zusaetzlich zur Loesung hinzugefuegt werden muss (wird ueber die Intersection mit entstandener Loesung entschieden) */
    f_row = sparse->rows[f_rowno2]; 
        
    rtc = -2;
    if (maRowInit(&f_rowcopy))
      if (maRowCopy(f_rowcopy, f_row))
        rtc = 1;
                    
    if (rtc > 0)
    {
      /* (void) maRowDeleteFieldByCol(f_rowcopy, f_colno1); */
     
      /* Fuer die Loesung muss die Submatrix noch ergaenzt werden: (siehe Theorie in der schriftlichen Ausarbeitung):
         alle [?a = 1] Zeilen ....  */
      for (field = f_col2->fieldptr_firstrow; field != NULL && rtc > 0; field = field->fieldptr_nextrow)

        /* erhalten die Felder [?b = 1] der row2 */
        for (field2 = f_rowcopy->fieldptr_firstcol; field2 != NULL && rtc > 0; field2 = field2->fieldptr_nextcol)
          if (!maMatrixInsertField(sparse, field2->col_no, field->row_no))
            rtc = -2;

      /* Loeschen der Spalte 1 / Zeile 1 */
    (void) maMatrixDeleteCol(sparse, f_colno1); 
      (void) maMatrixDeleteRow(sparse, f_rowno1);

      /* Loeschen der Spalte 2 / Zeile 2 */
      (void) maMatrixDeleteCol(sparse, f_colno2);
      (void) maMatrixDeleteRow(sparse, f_rowno2);
    }
    
    if (rtc > 0)
    {
/* rtc = -2;  
   if (maMatrixInitSized(&sparse_forchild, sparse->col_cnt, sparse->row_cnt))
     if (maMatrixCopy(sparse_forchild, sparse))
       rtc = 1;  */
    
      /* Wenn es keine Speicherprobleme gab, geht's jetzt an die Loesung der Submatrix */
      statistic->gimpels_above++;
      if (!maMatrixCoverRek(sparse, sol_fromparent, sol_toparent, weight, (cost_min - 1), (cost_max - 1), (recursion_depth + 1), statistic))
      {
        /* Gab es Probleme mit dem Speicher ? */
/* (void) maMatrixDestroy(sparse_forchild); */
        rtc = -2;
      }
      statistic->gimpels_above--;
    }
    
    if (rtc > 0)
    {

/* if (!maSolutionVerify(sparse, sol_toparent))
     (void) maSolutionClear(sol_toparent);
   else 
   { */
  
        /* Wenn keine gueltige Loesung zustande kam, dann wird der Loesungsansatz verworfen */
        if (maRowIntersection(sol_toparent->row, f_rowcopy))
        {
          if (!maSolutionAddCol(sol_toparent, ((weight == NULL) ? 1 : weight[f_colno2]), f_colno2))
            rtc = -2;
        }
        else
          if (!maSolutionAddCol(sol_toparent, ((weight == NULL) ? 1 : weight[f_colno1]), f_colno1))
            rtc = -2;
/* } */
    }
    
    (void) maRowDestroy(f_rowcopy);
/* (void) maMatrixDestroy(sparse_forchild); */
  }

  return rtc;
}

/* ----- maMatrixSelectBranching ----------------------- */
int maMatrixSelectBranching (ma_ptr2matrix sparse, int *weight, int *indep_cost)
{
  ma_ptr2matrix intersection;
  ma_ptr2solution solution;
  int rtc = -1;

  if (maMatrixInitSized(&intersection, sparse->rowptr_last->no, sparse->rowptr_last->no))
    if (maMatrixBuildRowIntersection(sparse, intersection) && maSolutionInit(&solution))
      if (maMatrixFindIndepCols(sparse, intersection, weight, solution))
      {
        *indep_cost = solution->cost;

        /* Auch maMatrixSelectCol gibt -1 bei einem Fehler zurueck */
        rtc = maMatrixSelectCol(sparse, weight, solution);
      }      

  (void) maMatrixDestroy(intersection);
  (void) maSolutionDestroy(solution);  

  return rtc;
} 

/* ----- maMatrixSelectCol ------------------------------- */
int maMatrixSelectCol (ma_ptr2matrix sparse, int *weight, ma_ptr2solution solution)
{
  int rtc = -1;
  double c_temp, c_weight = -1;
  ma_ptr2col f_col;
  ma_ptr2row f_row, indep_cols;
  ma_ptr2field field, sol_field;
  
  if (maRowInit(&indep_cols))
  {
    /* In der solution->row, die uebergeben wurde, sind in den einzelnen Feldern Zeilen !!! abgespeichert,
       in der solution->cost die niedrigst moeglichen Kosten fuer Spalten, die die oben genannten Zeilen referenzieren */
    if (solution != NULL)
    {    
      if (solution->row->cnt > 0)
        rtc = solution->row->fieldptr_firstcol->col_no;
        
      for (sol_field = solution->row->fieldptr_firstcol; sol_field != NULL && rtc >= 0; sol_field = sol_field->fieldptr_nextcol)
      {
        f_row = sparse->rows[sol_field->col_no];
        for (field = f_row->fieldptr_firstcol; field != NULL && rtc >= 0; field = field->fieldptr_nextcol)
          if (!maRowInsertField(indep_cols, field->col_no))
            rtc = -1;
      } 
    }
    else /* dieser Fall tritt ein, wenn es bei den Zeilen z.B. gar keine Intersektion gibt, also die Intersectionmatrix voellig leer war.*/
    {
      for (f_col = sparse->colptr_first; f_col !=NULL && rtc >= 0; f_col = f_col->colptr_next)
        if (!maRowInsertField(indep_cols, field->col_no))
          rtc = -1;
    }
     
    /* Heraussuchen der geeignetsten Spalte aus dem Set */
    for (field = indep_cols->fieldptr_firstcol; field != NULL && rtc != -1; field = field->fieldptr_nextcol)
    {
      f_col = sparse->cols[field->col_no];
      for (c_temp = 0.0, sol_field = f_col->fieldptr_firstrow; sol_field != NULL; sol_field = sol_field->fieldptr_nextrow)
      {
        f_row = sparse->rows[sol_field->row_no];
        c_temp += (1.0 / ((double) f_row->cnt - 1.0));
      }
      
      /* Divisor ist das relativ fuer diese Spalte vergebene Gewicht */
      if (weight != NULL)
        c_temp /= ((double) weight[f_col->no]);
            
      /* Ist eine eher geeignete Spalte zum branchen gefunden worden ? */
      if (c_temp > c_weight)
      {
        c_weight = c_temp;
        rtc = f_col->no;
      }
    }
    (void) maRowDestroy(indep_cols);
  }
  
  return rtc;
}

/* ----- maMatrixShow1 ------------------------------ */
void maMatrixShow1 (ma_ptr2matrix sparse)
{
  (void) maMatrixShowSol1(sparse, NULL); 
}

/* ----- maMatrixShow2 ------------------------------ */
void maMatrixShow2 (ma_ptr2matrix sparse)
{
  (void) maMatrixShowSol2(sparse, NULL); 
}

/* ----- maMatrixShowSol1 --------------------------- */
void maMatrixShowSol1 (ma_ptr2matrix sparse, ma_ptr2solution solution)
{
  int anzeige, used, no_cols = 0, no_rows = 0, no_elems = 0;  
  ma_ptr2col f_col;
  ma_ptr2row f_row;
  ma_ptr2field field;
  
  if (sparse->colptr_first != NULL && sparse->rowptr_first != NULL)
  {    
    tprintf("\n# cover-analysis:\n#\n#\n");

    /* Splatenbeschriftung: Tausender Stellen ausgeben */
    if (sparse->colptr_last->no >= 1000)
    {
      tprintf("#      ");  
      for (f_col = sparse->colptr_first; f_col != NULL; f_col = f_col->colptr_next)
        tprintf("%d", ((f_col->no / 1000) % 10));
      tprintf("\n");
    }

    /* Splatenbeschriftung: Hunderter Stellen ausgeben */
    if (sparse->colptr_last->no >= 100)
    {
      tprintf("#      ");  
      for (f_col = sparse->colptr_first; f_col != NULL; f_col = f_col->colptr_next)
        tprintf("%d", ((f_col->no / 100) % 10));
      tprintf("\n");
    }

    /* Splatenbeschriftung: Zehner Stellen ausgeben */
    if (sparse->colptr_last->no >= 10)
    { 
      tprintf("#      ");  
      for (f_col = sparse->colptr_first; f_col != NULL; f_col = f_col->colptr_next)
        tprintf("%d", ((f_col->no / 10) % 10));
      tprintf("\n");
    }

    /* Splatenbeschriftung: Einer Stellen ausgeben */
    tprintf("#      ");  
    for (f_col = sparse->colptr_first; f_col != NULL; f_col = f_col->colptr_next)
      {
        no_cols++;        
        tprintf("%d", (f_col->no % 10));
      }
    tprintf("\n#\n");
  
    /* Zeilenbeschriftung und Matrixfelder anzeigen*/
    for (f_row = sparse->rowptr_first, used = 0; f_row != NULL; f_row = f_row->rowptr_next, used = 0)
    {
      no_rows++;        
      tprintf("# %04d ", f_row->no);

      for (f_col = sparse->colptr_first; f_col != NULL; f_col = f_col->colptr_next)
      {
        (void) maRowFindField(f_row, &field, f_col->no);
        if (field != NULL)
          no_elems++;
        if (solution != NULL)
          if (maSolutionCheckCol(solution, f_col->no))
            used += (field != NULL);
        tprintf("%c", "-+" [(field != NULL)]);
      }
      if (solution != NULL)
      {
        tprintf("  => [%c]\n", " X" [(used >= 1)]);
      }
      else
        tprintf("\n");
    }

    if (solution != NULL)
    {
      /* Auswahlvektor ausgeben */
      tprintf("#      ");  
      for (f_col = sparse->colptr_first; f_col != NULL; f_col = f_col->colptr_next)
        tprintf("%c", " ^" [maSolutionCheckCol(solution, f_col->no)]);
      tprintf("\n#\n");
    }
    
    tprintf("# array     = %d by %d with %d elements (%.1f%%)\n#", no_cols, no_rows, no_elems, (((float)no_elems * 100.0)/((float)(no_cols*no_rows))));
  }
  else
    tprintf("\n#\n# No matrix to display!");
}

/* ----- maMatrixShowSol2 --------------------------- */
void maMatrixShowSol2 (ma_ptr2matrix sparse, ma_ptr2solution solution)
{
  int anzeige, col_used, no_elems = 0;  
  ma_ptr2col f_col, f_check;
  ma_ptr2row f_row;
  ma_ptr2field field;
  
  if (sparse->colptr_first != NULL && sparse->rowptr_first != NULL  && maColInit(&f_check))
  {    
    tprintf("\n# cover-analysis:\n#\n#\n");

    /* Spaltenbeschriftung und Matrixfelder anzeigen*/
    for (f_col = sparse->colptr_last; f_col != NULL; f_col = f_col->colptr_prev)
    {
      tprintf("# %04d ", f_col->no);
      if (solution != NULL)
        col_used = maSolutionCheckCol(solution, f_col->no);
      else
        col_used = 0;
        
      for (f_row = sparse->rowptr_first; f_row != NULL; f_row = f_row->rowptr_next)
      {
        (void) maRowFindField(f_row, &field, f_col->no);
        tprintf("%c", "-+" [(field != NULL)]);
        if (field != NULL)
        {
          no_elems++;
          if (col_used)
          {
            if (!maColInsertField(f_check, f_row->no))
            {
              (void) maColDestroy(f_check);
              return;
            } 
          }
        }
      }
      if (solution != NULL)
        tprintf("  [%c]\n", " X" [col_used]);
      else
        tprintf("\n");
      
    }

    if (solution != NULL)
    {
      /* Auswahlvektor ausgeben */
      tprintf("#\n#      ");  
      for (f_row = sparse->rowptr_first; f_row != NULL; f_row = f_row->rowptr_next)
      {
        (void) maColFindField(f_check, &field, f_row->no);
        tprintf("%c", "-+" [(field != NULL)]);
      }
      tprintf("\n#\n");
    }
    
    /* Zeilenbeschriftung: Tausender Stellen ausgeben */
    if (sparse->rowptr_last->no >= 1000)
    {
      tprintf("#      ");  
      for (f_row = sparse->rowptr_first; f_row != NULL; f_row = f_row->rowptr_next)
        tprintf("%d", ((f_row->no / 1000) % 10));
      tprintf("\n");
    }

    /* Zeilenbeschriftung: Hunderter Stellen ausgeben */
    if (sparse->rowptr_last->no >= 100)
    {
      tprintf("#      ");  
      for (f_row = sparse->rowptr_first; f_row != NULL; f_row = f_row->rowptr_next)
        tprintf("%d", ((f_row->no / 100) % 10));
      tprintf("\n");
    }

    /* Zeilenbeschriftung: Zehner Stellen ausgeben */
    if (sparse->colptr_last->no >= 10)
    { 
      tprintf("#      ");  
      for (f_row = sparse->rowptr_first; f_row != NULL; f_row = f_row->rowptr_next)
        tprintf("%d", ((f_row->no / 10) % 10));
      tprintf("\n");
    }

    /* Zeilenbeschriftung: Einer Stellen ausgeben */
    tprintf("#      ");  
    for (f_row = sparse->rowptr_first; f_row != NULL; f_row = f_row->rowptr_next)
      tprintf("%d", (f_row->no % 10));
    tprintf("\n#\n");

    tprintf("# array     = %d by %d with %d elements (%.1f%%)\n#", sparse->col_cnt, sparse->row_cnt, no_elems, (((float)no_elems * 100.0)/((float)(sparse->col_cnt*sparse->row_cnt))));

    (void) maColDestroy(f_check);
  }
  else
    tprintf("\n#\n# No matrix to display!");
  
}

/* ----- maMatrixVADestroy -------------------------- */
void maMatrixVADestroy (int n, ...)
{
  va_list va;
  int i;

  va_start(va, n);
  for (i = 0; i < n; i++)
    (void) maMatrixDestroy(va_arg(va, ma_ptr2matrix));
  va_end(va);
}

/* ----- maMatrixVisitCol --------------------------- */
int maMatrixVisitCol(ma_ptr2matrix sparse, ma_ptr2col h_col, int *no_cols, int *no_rows)
{
  int rtc = 1;
  ma_ptr2row f_row;
  ma_ptr2field field;

  if (h_col->visited == 0)
  {
    h_col->visited++;
    (*no_cols)++;
    
    /* Wenn alle Spalten besucht wurden, laesst sich die Matrix nicht zerlegen (rtc = 0) */
    if (*no_cols != sparse->col_cnt)
    {
      for (field = h_col->fieldptr_firstrow; field != NULL && rtc != 0; field = field->fieldptr_nextrow)
      {
        f_row = sparse->rows[field->row_no];

        /* Alle noch unbesuchten, aber in dieser Spalte referenzierten Zeilen ueberpruefen ... */
        if (f_row->visited == 0)
          rtc = maMatrixVisitRow(sparse, f_row, no_cols, no_rows);
      }
    }
    else
      rtc = 0;
  }

  return rtc;
}

/* ----- maMatrixVisitRow --------------------------- */
int maMatrixVisitRow(ma_ptr2matrix sparse, ma_ptr2row h_row, int *no_cols, int *no_rows)
{
  int rtc = 1;
  ma_ptr2col f_col;
  ma_ptr2field field;
  
  if (h_row->visited == 0)
  {
    h_row->visited++;
    (*no_rows)++;
    
    /* Wenn alle Zeilen besucht wurden, laesst sich die Matrix nicht zerlegen (rtc = 0) */
    if (*no_rows != sparse->row_cnt)
    {
      for (field = h_row->fieldptr_firstcol; field != NULL && rtc != 0; field = field->fieldptr_nextcol)
      {
        f_col = sparse->cols[field->col_no];

        /* Alle noch unbesuchten, aber in dieser Zeile referenzierten Spalten ueberpruefen ... */
        if (f_col->visited == 0)
          rtc = maMatrixVisitCol(sparse, f_col, no_cols, no_rows);
      }
    }
    else
      rtc = 0;
  }

  return rtc;
}

/* ===== maRow... (Zeilen-Fkt.) ===================== */

/* ----- maRowContainment --------------------------- */
int maRowContainment (ma_ptr2row h_rowptr_src1, ma_ptr2row h_rowptr_src2)
{
  int rtc = 1;
  ma_ptr2field field1, field2;
  
  field1 = h_rowptr_src1->fieldptr_firstcol;
  field2 = h_rowptr_src2->fieldptr_firstcol;
  
  /* Nur ueberpruefen, wenn h_rowptr_src1 ueberhaupt Felder enthaelt, sonst "containt" ihn der h_rowptr_src2 in jedem Fall */
  while (field1 != NULL)
  {
    if (field2 == NULL || field1->col_no < field2->col_no)
    {
      rtc = 0;
      break;
    }
    else
    {
      if (field1->col_no == field2->col_no)
      {
        field1 = field1->fieldptr_nextcol;
        field2 = field2->fieldptr_nextcol;
      }
      else
        field2 = field2->fieldptr_nextcol;
    }
  }
  
  return rtc;
}

/* ----- maRowCopy ---------------------------------- */
int maRowCopy (ma_ptr2row h_rowptr_dest, ma_ptr2row h_rowptr_src)
{
  ma_ptr2field field, prevfield;
  int rtc = 1;

  /* h_rowptr_dest von evtl. Altlasten befreien */
  for (field = h_rowptr_dest->fieldptr_lastcol; field != NULL; field = prevfield)
  {
    prevfield = field->fieldptr_prevcol;
    (void) maRowDeleteFieldByPointer (h_rowptr_dest, field);
  }

  /* Feld fuer Feld kopieren ... */
  for (field = h_rowptr_src->fieldptr_firstcol; field != NULL && rtc > 0; field = field->fieldptr_nextcol)
    if (!maRowInsertField(h_rowptr_dest, field->col_no))
      rtc = 0;

  return rtc;
}

/* ----- maRowDebug --------------------------------- */
void maRowDebug (ma_ptr2row h_rowptr)
{
  FILE *fp_hlp;

  if ( fp_out == NULL )
    fp_out = stderr;

  fp_hlp = fp_out;
  fp_out = stderr;

  (void) maRowShow(h_rowptr);

  fp_out = fp_hlp;
}

/* ----- maRowDestroy ------------------------------- */
void maRowDestroy (ma_ptr2row h_row)
{
  ma_ptr2field thisfield, nextfield; 

  if (h_row != NULL)
  {
    /* Alle enthaltenen Felder entfernen */
    for (thisfield = h_row->fieldptr_firstcol; thisfield != NULL; thisfield = nextfield)
    {
      nextfield = thisfield->fieldptr_nextcol;
      (void) maFieldDestroy(thisfield);
    }
  
    /* die Felder der Struktur zuruecksetzen ... */
    h_row->fieldptr_firstcol = NULL;
    h_row->fieldptr_lastcol = NULL;
    h_row->rowptr_next = NULL;
    h_row->rowptr_prev = NULL;
    h_row->visited = 0;
    h_row->no = 0;
    h_row->cnt = 0;
  
    /* und freigeben. */
    free(h_row);
  }
}

/* ----- maRowDeleteFieldByCol ---------------------- */
void maRowDeleteFieldByCol (ma_ptr2row h_rowptr, int h_col)
{
  ma_ptr2field field;

  if (h_rowptr != NULL && h_col > 0)
  {
    /* Pointer auf betreffendes Feld stellen */
    if (h_col <= ((h_rowptr->fieldptr_lastcol->col_no + h_rowptr->fieldptr_firstcol->col_no) / 2))
      for (field = h_rowptr->fieldptr_firstcol; field != NULL && field->col_no < h_col; field = field->fieldptr_nextcol);
    else
      for (field = h_rowptr->fieldptr_lastcol; field != NULL && field->col_no > h_col; field = field->fieldptr_prevcol);
    
    /* Existiert das Feld in der Zeile ueberhaupt ? */
    if (field != NULL && field->col_no == h_col)
      (void) maRowDeleteFieldByPointer (h_rowptr, field);
  }
}

/* ----- maRowDeleteFieldByPointer ------------------ */
void maRowDeleteFieldByPointer (ma_ptr2row h_row, ma_ptr2field h_field)
{
  if (h_row != NULL && h_field != NULL)
  {
    /* Das Feld muss aus seinem Zeilenbezug geloest werden */

    /* Vorderen Bezug des loesen */
    if (h_field->fieldptr_prevcol == NULL)
      h_row->fieldptr_firstcol = h_field->fieldptr_nextcol;
    else
      h_field->fieldptr_prevcol->fieldptr_nextcol = h_field->fieldptr_nextcol;

    /* Hinteren Bezug des loesen */
    if (h_field->fieldptr_nextcol == NULL)
      h_row->fieldptr_lastcol = h_field->fieldptr_prevcol;
    else
      h_field->fieldptr_nextcol->fieldptr_prevcol = h_field->fieldptr_prevcol;

    h_row->cnt--;
    (void) maFieldDestroy(h_field);
  }
}

/* ----- maRowFindField ----------------------------- */
void maRowFindField (ma_ptr2row h_rowptr, ma_ptr2field *h_field, int h_col)
{
  /* Anfahren der Feldnnummer im Feldkontext */
  if (h_col <= ((h_rowptr->fieldptr_lastcol->col_no + h_rowptr->fieldptr_firstcol->col_no) / 2))
    /* Pointer auf betreffendes Feld stellen (je nachdem was schneller zu erreichen ist: von vorne) */
    for ((*h_field) = h_rowptr->fieldptr_firstcol; (*h_field) != NULL && (*h_field)->col_no < h_col; (*h_field) = (*h_field)->fieldptr_nextcol);
  else
    /* Pointer auf betreffendes Feld stellen (je nachdem was schneller zu erreichen ist: von hinten) */
   for ((*h_field) = h_rowptr->fieldptr_lastcol; (*h_field) != NULL && (*h_field)->col_no > h_col; (*h_field) = (*h_field)->fieldptr_prevcol);
 
 /* Gibt es ueberhaupt Felder in dieser Row und existiert das gesuchte Feld No: # h_col ?? */
 if ((*h_field) != NULL)
   if ((*h_field)->col_no != h_col)
     (*h_field) = NULL;
}

/* ----- maRowInit ---------------------------------- */
int maRowInit (ma_ptr2row *newrow)
{
  int rtc = 0;
  
  *newrow = (ma_ptr2row)malloc(sizeof(struct _ma_row_struct));
  if (*newrow != NULL)
  {
    (*newrow)->fieldptr_firstcol = NULL;
    (*newrow)->fieldptr_lastcol = NULL;
    (*newrow)->rowptr_next = NULL;
    (*newrow)->rowptr_prev = NULL;
    (*newrow)->no = 0;
    (*newrow)->cnt = 0;
    (*newrow)->visited = 0;
    rtc++;
  }
  return rtc;
}

/* ----- maRowInsertField --------------------------- */
int maRowInsertField (ma_ptr2row h_rowptr, int h_col)
{
  ma_ptr2field newfield, nf_copy;
  int rtc = 0;

  /* Das neue Feld der Zeile aufbauen ... */
  if (maFieldInit(&newfield))
  {  
    nf_copy = newfield;
  
    /* Das evtl. neue Feld in die Zeile einfuegen */
    (void) maRowLinkField(h_rowptr, h_col, &newfield);

    /* Wenn beim Linken klar wurde, dass das Feld nicht neu war wieder loeschen */
    if (newfield != nf_copy)
      (void) maFieldDestroy(nf_copy);

    rtc++;
  }

  return rtc;
}

/* ----- maRowIntersection -------------------------- */
int maRowIntersection (ma_ptr2row h_rowptr_src1, ma_ptr2row h_rowptr_src2)
{
  int rtc = 0;
  ma_ptr2field field1, field2;
  
  field1 = h_rowptr_src1->fieldptr_firstcol;
  field2 = h_rowptr_src2->fieldptr_firstcol;
  
  if (field1 != NULL && field2 != NULL)
    while (rtc == 0)
    {
      if (field1->col_no < field2->col_no)
      {
        field1 = field1->fieldptr_nextcol;
        if (field1 == NULL)
          break;
      }
      else
      {
        if (field1->col_no > field2->col_no)
        {
          field2 = field2->fieldptr_nextcol;
          if (field2 == NULL)
            break;
        }
        else
          rtc++;
      }
    }
  
  return rtc;
}

/* ----- maRowLinkField ----------------------------- */
void maRowLinkField (ma_ptr2row h_rowptr, int h_colno, ma_ptr2field *h_fieldptr)
{                   
  ma_ptr2field lo_fieldptr;
  
  (*h_fieldptr)->col_no = h_colno;
  h_rowptr->cnt++;

  /* Fall 1.) Die Row enthaelt noch gar keine Felder ... */
  if (h_rowptr->fieldptr_lastcol == NULL)
  {
    (*h_fieldptr)->fieldptr_prevcol = NULL;
    (*h_fieldptr)->fieldptr_nextcol = NULL;

    h_rowptr->fieldptr_firstcol = (*h_fieldptr);
    h_rowptr->fieldptr_lastcol = (*h_fieldptr);
  }
  else 
  {
    /* Fall 2.) Das spezifizierte Feld ist letztes Feld der Zeile und muss linksseitig in den Feldkontext der Matrix einsortiert werden ... */
    if (h_rowptr->fieldptr_lastcol->col_no < h_colno)
    {
      (*h_fieldptr)->fieldptr_prevcol = h_rowptr->fieldptr_lastcol;
      (*h_fieldptr)->fieldptr_nextcol = NULL;

      h_rowptr->fieldptr_lastcol->fieldptr_nextcol = (*h_fieldptr);
      h_rowptr->fieldptr_lastcol = (*h_fieldptr);
    }
    else 
    {
      /* Fall 3.) Das spezifizierte Feld ist erstes Feld der Zeile und muss rechtsseitig in den Feldkontext der Matrix einsortiert werden ... */
      if (h_rowptr->fieldptr_firstcol->col_no > h_colno)
      {
        (*h_fieldptr)->fieldptr_prevcol = NULL;
        (*h_fieldptr)->fieldptr_nextcol = h_rowptr->fieldptr_firstcol;

        h_rowptr->fieldptr_firstcol->fieldptr_prevcol = (*h_fieldptr);
        h_rowptr->fieldptr_firstcol = (*h_fieldptr);
      } 
      else
      {
        /* Anfahren der Feldnnummer im Feldkontext "von vorne" */
        if (h_colno <= ((h_rowptr->fieldptr_lastcol->col_no + h_rowptr->fieldptr_firstcol->col_no) / 2))
          for (lo_fieldptr = h_rowptr->fieldptr_firstcol; lo_fieldptr->col_no < h_colno; lo_fieldptr = lo_fieldptr->fieldptr_nextcol);
        else
        {
          /* Anfahren des Feldes "von hinten" */
          lo_fieldptr = h_rowptr->fieldptr_lastcol;
          if (h_colno != lo_fieldptr->col_no)
          {
            for (; lo_fieldptr->col_no >= h_colno; lo_fieldptr = lo_fieldptr->fieldptr_prevcol);

            lo_fieldptr = lo_fieldptr->fieldptr_nextcol;
          }
        }
        
        /* Fall 4.) Das spezifizierte Feld muss beidseitig in den Feldkontext der Matrix einsortiert werden ... */
        if (lo_fieldptr->col_no > h_colno)
        {
          /* zu weit -> wieder eine Position nach voerne ruecken (auf alle Faelle ist jetzt bekannt, dass das Feld noch nicht exisitierte) */
          lo_fieldptr = lo_fieldptr->fieldptr_prevcol;

          (*h_fieldptr)->fieldptr_prevcol = lo_fieldptr;
          (*h_fieldptr)->fieldptr_nextcol = lo_fieldptr->fieldptr_nextcol;

          lo_fieldptr->fieldptr_nextcol->fieldptr_prevcol = (*h_fieldptr);
          lo_fieldptr->fieldptr_nextcol = (*h_fieldptr);
        }
        else
        {
          /* Fall 5.) Das spezifizierte Feld exisitert bereits (dann war die Erheohung des Zeilen-Counters unzulaessig)... */
          h_rowptr->cnt--;
          (*h_fieldptr) = lo_fieldptr;
        }
      }
    }
  }
}

/* ----- maRowLogAnd -------------------------------- */
int maRowLogAnd (ma_ptr2row h_rowptr_dest, ma_ptr2row h_rowptr_src1, ma_ptr2row h_rowptr_src2)
{
  ma_ptr2field field1, field2;
  int rtc = 1;

  if (h_rowptr_dest->cnt > 0)
    rtc = 0;

  field1 = h_rowptr_src1->fieldptr_firstcol;
  field2 = h_rowptr_src2->fieldptr_firstcol;

  if (field1 != NULL && field2 != NULL)  
    while (rtc != 0)
    {
      if (field1->col_no < field2->col_no)
      {
        field1 = field1->fieldptr_nextcol;
        if (field1 == NULL)
          break;
      }
      else
      {
        if (field1->col_no > field2->col_no)
        {
          field2 = field2->fieldptr_nextcol;
          if (field2 == NULL)
            break;
        }
        else
        {
          if (!maRowInsertField(h_rowptr_dest, field1->col_no))
            rtc = 0;
          
          field1 = field1->fieldptr_nextcol;
          field2 = field2->fieldptr_nextcol;
          if (field1 == NULL || field2 == NULL)
            break;
        }
      }
    }    

  return rtc;
}

/* ----- maRowShow ---------------------------------- */
void maRowShow (ma_ptr2row h_rowptr)
{
  ma_ptr2field field;

  tprintf("\n");

  for (field = h_rowptr->fieldptr_firstcol; field != NULL; field = field->fieldptr_nextcol)
    tprintf("%d ", field->col_no);

  tprintf("\n");
}

/* ===== maSolution... (Loesungs-Fkt.) ============== */

/* ----- maSolutionAddCol --------------------------- */
int maSolutionAddCol (ma_ptr2solution solution, int h_colweight, int h_colno)
{
  int rtc = 0;

  if (maRowInsertField(solution->row, h_colno))
  {
    solution->cost += h_colweight;
    rtc++;  
  }
  
  return rtc;
}

/* ----- maSolutionClear ---------------------------- */
void maSolutionClear (ma_ptr2solution solution)
{
  ma_ptr2field thisfield, nextfield; 

  solution->cost = 0;

  /* Alle enthaltenen Felder entfernen */
  for (thisfield = solution->row->fieldptr_firstcol; thisfield != NULL; thisfield = nextfield)
  {
    nextfield = thisfield->fieldptr_nextcol;
    (void) maFieldDestroy(thisfield);
  }
  
  /* die Felder der Struktur zuruecksetzen ... */
  solution->row->fieldptr_firstcol = NULL;
  solution->row->fieldptr_lastcol = NULL;
  solution->row->rowptr_next = NULL;
  solution->row->rowptr_prev = NULL;
  solution->row->visited = 0;
  solution->row->no = 0;
  solution->row->cnt = 0;
}

/* ----- maSolutionInit ----------------------------- */
int maSolutionInit (ma_ptr2solution *solution)
{
  int rtc = 0;

  (*solution) = (ma_ptr2solution)malloc(sizeof(struct _ma_solution_struct));
  if (*solution != NULL)
  {
    (*solution)->cost = 0;
    if (maRowInit(&(*solution)->row))
      rtc++;    
  }
  return rtc;
}

/* ----- maSolutionDestroy -------------------------- */
void maSolutionDestroy (ma_ptr2solution solution)
{
  if (solution != NULL)
  {
    (void) maRowDestroy(solution->row);
    free(solution);
  }
}

/* ----- maSolutionVADestroy -------------------------- */
void maSolutionVADestroy (int n, ...)
{
  va_list va;
  int i;

  va_start(va, n);
  for (i = 0; i < n; i++)
    maSolutionDestroy(va_arg(va, ma_ptr2solution));
  va_end(va);
}

/* ----- maSolutionVAInit --------------------------- */
int maSolutionVAInit (int n, ...)
{
  va_list va;
  int i, rtc = 1;

  va_start(va, n);
  for (i = 0; i < n; i++)
    if (!maSolutionInit(va_arg(va, ma_ptr2solution *)))
    {
      rtc = 0;
      break;
    }
  va_end(va);

  if (!rtc)
  {
    va_start(va, n);
    while( i-- >= 0 )
      (void) maSolutionDestroy(*va_arg(va, ma_ptr2solution *));
    va_end(va);
  }
  return rtc;
}

/* ----- maSolutionCopy ----------------------------- */
int maSolutionCopy (ma_ptr2solution sol_dest, ma_ptr2solution sol_src)
{
  int rtc = 1;
  
  sol_dest->cost = sol_src->cost;
  if (maRowCopy(sol_dest->row, sol_src->row) == 0)
    rtc = 0;

  return rtc;
}

/* ----- maSolutionVerify --------------------------- */
int maSolutionVerify (ma_ptr2matrix sparse, ma_ptr2solution solution)
{
  int rtc = 1;
  ma_ptr2row f_row;
  
  for (f_row = sparse->rowptr_first; f_row != NULL; f_row = f_row->rowptr_next)
    if (!maRowIntersection ( f_row,  solution->row))
    {
      rtc = 0;
      break;
    }
  return rtc;
} 

/* ----- maSolutionCheckCol ------------------------- */
int maSolutionCheckCol (ma_ptr2solution solution, int h_colno)
{
  int rtc = 0;
  ma_ptr2field field;
  
  for (field = solution->row->fieldptr_firstcol; field != NULL; field = field->fieldptr_nextcol)
    if (field->col_no == h_colno)
    {
      rtc = 1;
      break;
    }

  return rtc;
}

/* ===== Tools ====================================== */

/* ----- tprintf ------------------------------------ */
void tprintf (char *fmt, ...)
{
  va_list va;
  
  if ( fp_out == NULL )
    fp_out = stderr;
  
  va_start(va, fmt);
  if (fp_out == NULL || fp_out == stderr)
    vfprintf(stderr, fmt, va);
  else
    vfprintf(fp_out, fmt, va);
    
  va_end(va);
}

long t_process_msec(void)
{
  return 0;
/*
 *    struct rusage resource;
 *    long ms;
 *    
 *    if (getrusage( RUSAGE_SELF, &resource) != 0)
 *      ms = 0L;
 *    else
 *      ms = resource.ru_utime.tv_sec*1000L+resource.ru_utime.tv_usec/1000L;
 *    return ms;
 */
}
