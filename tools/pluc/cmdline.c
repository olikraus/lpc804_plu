/*

   CMDLINE.C

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

   On unix machines, use -DUNIX
   
   todo: default ausgabe fuer ESCSEQ

   number:

   {'$'|[{+| }]['0'][{'x'|'X'}]}[digits][{'k'|'K'}]

   If the first character is 0 and the second character is not
   'x' or 'X', the string is interpreted as an octal integer.
   Otherwise, it is interpreted as a decimal number.


   Beginnt die Zahl mit '$' oder '0x', wird die Basis 16 verwendet,
   bei '0' die Basis 8, sonst die Basis 10. Endet eine Zahl mit 'k'
   oder 'K' wird sie mit 1024 multipliziert.

   escape sequence

   escaped_char ::= <char> | <escape_seq1>
   escape_seq1  ::= '\' <escape_seq>
   escape_seq   ::= 'a' | 'b' | 'f' | 'n' | 'r' | 't' | 'v' | '?' | ''' |
                    '"' | '\' | <escape_hex> | <escape_oct>
   escape_hex   ::= 'x' <hex_char> <hex_char>
   escape_oct   ::= ('0'|'1'|'2'|'3') <oct_char> <oct_char>

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "cmdline.h"
#include "mwc.h"

char cl_file_list[CL_FILE_MAX][_MAX_PATH];
int cl_file_cnt = 0;

unsigned char cl_escaped_char(unsigned char **src)
{
  unsigned char c;
  int i;
  if ( **src == '\\' )
  {
    (*src)++;
    if ( **src == '\0' )
    {
      return '\\';
    }
    switch(**src)
    {
      case 'a': c = '\a'; (*src)++; break;
      case 'b': c = '\b'; (*src)++; break;
      case 'f': c = '\f'; (*src)++; break;
      case 'n': c = '\n'; (*src)++; break;
      case 'r': c = '\r'; (*src)++; break;
      case 't': c = '\t'; (*src)++; break;
      case 'v': c = '\v'; (*src)++; break;
      case '\?': c = '\?'; (*src)++; break;
      case '\'': c = '\''; (*src)++; break;
      case '\"': c = '\"'; (*src)++; break;
      case '\\': c = '\\'; (*src)++; break;
      case 'x':
        c = 0;
        (*src)++;
        for( i = 0; i < 2; i++ )
        {
          if ( **src == '\0' )
            break;
          if ( !(isdigit((int)(unsigned char)**src) != 0 ||
               (toupper((int)(unsigned char)**src) >= 'A' &&
                toupper((int)(unsigned char)**src) <= 'F'   )) )
          {
            break;
          }
          c *= 16;
          if ( **src <= '9' )
            c += (**src)-'0';
          else
            c += toupper((int)(unsigned char)**src)-'A'+10;
          (*src)++;
        }
        break;
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
        c = 0;
        for( i = 0; i < 3; i++ )
        {
          if ( **src == '\0' )
            break;
          if ( **src <= '0' || **src >= '7' )
          {
            break;
          }
          c *= 8;
          c += (**src)-'0';
          (*src)++;
        }
        break;
      default:
        c = **src;
        (*src)++;
        break;
    }
  }
  else
  {
    c = **src;
    (*src)++;
  }
  return c;
}

size_t cl_strlen(s)
   char *s; 
{
  size_t i = 0;
  for(;;)
  {
    if ( *s == '\0' )   
      break;
    if ( *s == CL_HELP_TEXT_DELIMITER )   /* help delimiter */
      break;
    i++;
    s++;
  }
  return i;
}

int cl_strcmp(s1, s2)
   char *s1; 
   char *s2;
{
   for(;;)
   {
      if ( *s1 == '\0' )   /* name aus der tabelle */
         return 0;
      if ( *s1 == CL_HELP_TEXT_DELIMITER )   /* help delimiter */
         return 0;
      if ( *s2 == '\0' )   /* name von der commandline */
         return -1;

      if ( toupper((int)(unsigned char)*s1) < toupper((int)(unsigned char)*s2) )
         return -1;
      if ( toupper((int)(unsigned char)*s1) > toupper((int)(unsigned char)*s2) )
         return 1;
      s1++;
      s2++;
   }
}

long cl_strtol( nptr, endptr )
   char *nptr;
   char **endptr;
{
   long res = 0L;
   while( isspace((unsigned char)*nptr) )
      nptr++;
   if ( *nptr == '$' )
   {
      nptr++;
      for(;;)
      {
         if ( *nptr >= '0' && *nptr <= '9' )
         {
            res *= 16L;
            res += (long)(*nptr-'0');
         }
         else if ( *nptr >= 'a' && *nptr <= 'f' )
         {
            res *= 16L;
            res += (long)(*nptr-'a');
            res += 10L;
         }
         else if ( *nptr >= 'A' && *nptr <= 'F' )
         {
            res *= 16L;
            res += (long)(*nptr-'A');
            res += 10L;
         }
         else
         {
            break;
         }
         nptr++;
      }
   }
   else
   {
      res = strtol( nptr, &nptr, 0 );
   }
   if ( *nptr == 'k' || *nptr == 'K' )
   {
      res *= 1024;
      nptr++;
   }
   if ( endptr != NULL )
      *endptr = nptr;
   return res;
}

int cl_GetOptionNo(list, option)
   cl_entry_struct *list; 
   char **option;
{
   int i;
   if ( list == NULL || option == NULL )
      return -3;
   while( isspace((unsigned char)**option) )
      (*option)++;
#ifdef UNIX
   if ( **option == '-' )
#else
#ifdef unix
   if ( **option == '-' )
#else
   if ( **option == '/' || **option == '-' )
#endif
#endif
   {
      i = 0;
      while( list[i].typ != CL_TYP_NONE )
      {
         if ( list[i].typ != CL_TYP_GROUP )
         {
            if ( cl_strcmp(list[i].name, (*option)+1) == 0 )
            {
               (*option) += 1 + cl_strlen(list[i].name);
               return i;
            }
         }
         i++;
      }
   }
   else
   {
      return -2;
   }
   return -1;
}

char *cl_get_help_text(entry)
   cl_entry_struct *entry;
{
  char *desc = entry->name;
  for(;;)
  {
    if ( *desc == '\0' )
      break;
    if ( *desc == CL_HELP_TEXT_DELIMITER )
    {
      desc++;
      break;
    }
    desc++;
  }
  return desc;
}

void cl_out_one_line_desc(entry, fp, prefix, fl1)
   cl_entry_struct *entry;
   FILE *fp;
   char *prefix;
   int fl1;
{
  char *s;

  if ( cl_get_help_text(entry)[0] == '.' )
    return;

  if ( prefix != NULL )
  {
    fprintf(fp, "%s", prefix);
  }
  
  putc((int)'-', fp);
  fl1--;
    
  s = entry->name;
  for(;;)
  {
    if ( *s == '\0' || *s == CL_HELP_TEXT_DELIMITER )
      break;
    putc((int)(unsigned char)*s, fp);
    s++;
    fl1--;
  }
  
  s = "";
  switch(entry->typ)
  {
    case CL_TYP_LONG: s= " <num>"; break;
    case CL_TYP_PATH: s= " <file>"; break;
    case CL_TYP_STRING: s= " <str>"; break;
    case CL_TYP_NAME: s= " <name>"; break;
  }
  
  for(;;)
  {
    if ( *s == '\0' || *s == CL_HELP_TEXT_DELIMITER )
      break;
    putc((int)(unsigned char)*s, fp);
    s++;
    fl1--;
  }
  
  while( fl1 > 0 )
  {  
    putc((int)(unsigned char)' ', fp);
    fl1--;
  }
  
  fprintf(fp, " %s", cl_get_help_text(entry));
  
  switch(entry->typ)
  {
    case CL_TYP_ON:
      if ( *(int *)entry->var_ptr != 0 )
        fprintf(fp, " (default)");
      break;
    case CL_TYP_OFF:
      if ( *(int *)entry->var_ptr == 0 )
        fprintf(fp, " (default)");
      break;
    case CL_TYP_SET:
      if ( *(long *)entry->var_ptr == (long)entry->var_size )
        fprintf(fp, " (default)");
      break;
    case CL_TYP_LONG:
      fprintf(fp, " (default: %ld)", *(long *)entry->var_ptr);
      break;
    case CL_TYP_PATH:
    case CL_TYP_STRING:
    case CL_TYP_NAME:
      if ( strlen((char *)entry->var_ptr) > 0 )
        fprintf(fp, " (default: '%s')", (char *)entry->var_ptr);
      break;
    case CL_TYP_ESCSEQ: /* no default at the moment :-( */
      break;
    case CL_TYP_GROUP:
      break;
  }
  putc((int)(unsigned char)'\n', fp);
  
}

void cl_OutHelp(entry, fp, prefix, fl1)
   cl_entry_struct *entry;
   FILE *fp;
   char *prefix;
   int fl1;
{
  int i;
  
  if ( entry == NULL )
    return;
  
  i = 0;
  while( entry[i].typ != CL_TYP_NONE )
  {
    if ( entry[i].typ == CL_TYP_GROUP )
        fprintf(fp, "%s\n", entry[i].name);
    else
      cl_out_one_line_desc(entry + i, fp, prefix, fl1);
    i++;
  }
}

int cl_Convert(entry, option, is_string_space )
   cl_entry_struct *entry;
   char **option;
   int is_string_space;
{
   size_t i;
   switch(entry->typ)
   {
      case CL_TYP_NONE:
         return -1;
      case CL_TYP_ON:
         *(int *)entry->var_ptr = 1;
         return 1;
      case CL_TYP_OFF:
         *(int *)entry->var_ptr = 0;
         return 1;
      case CL_TYP_SET:
         *(long *)entry->var_ptr = (long)entry->var_size;
         return 1;
      case CL_TYP_LONG:
         if ( **option == '\0' )
            return 0;
         *(long *)entry->var_ptr = cl_strtol( *option, option );
         return 1;
      case CL_TYP_PATH:
      case CL_TYP_STRING:
      case CL_TYP_NAME:
         if ( **option == '\0' )
            return 0;
         i = 0;
         /*
         if ( **option == '\"' )
            (*option)++;
         */
         for(;;)
         {
            if ( isspace((unsigned char)**option) && !is_string_space )
               break;
            /*
            if ( **option == '\"' )
               break;
            */
            if ( **option == '\0' )
               break;
            if ( i >= entry->var_size-1 )
               break;
            ((char *)entry->var_ptr)[i] = **option;
            (*option)++;
            i++;
         }
         if ( i >= entry->var_size )
            ((char *)entry->var_ptr)[entry->var_size-1] = '\0';
         else
            ((char *)entry->var_ptr)[i] = '\0';
         return 1;
      case CL_TYP_ESCSEQ:
         if ( **option == '\0' )
            return 0;
         i = 0;
         for(;;)
         {
            if ( isspace((unsigned char)**option) && !is_string_space )
               break;
            if ( **option == '\0' )
               break;
            ((char *)entry->var_ptr)[i] = 
              cl_escaped_char((unsigned char **)option);
            if ( i >= entry->var_size-1 )
               break;
            i++;
         }
         if ( i >= entry->var_size )
            ((char *)entry->var_ptr)[entry->var_size-1] = '\0';
         else
            ((char *)entry->var_ptr)[i] = '\0';
         return 1;
      case CL_TYP_GROUP:
        return 1;
        
   }
   return -1;
}

int cl_String(list, s)
   cl_entry_struct *list;
   char *s;
{
   int n;
   if ( s == NULL )
      return 0;
   while( *s != '\0' )
   {
      while ( isspace((unsigned char)*s) )
         s++;
      n = cl_GetOptionNo(list, &s);
      if ( n < 0 )
      {
         while ( *s != '\0' && !isspace((unsigned char)*s) )
            s++;
      }
      else
      {
         cl_Convert(list+n, &s, 1);
      }
   }
   return 1;
}

int cl_Do(list, argc, argv)
   cl_entry_struct *list; 
   int argc; 
   char **argv;
{
   int i;
   int n;
   int ret = 1;
   char *s;
   i = 1;
   cl_file_cnt = 0;
   while( argv[i] != NULL )
   {
      s = argv[i];
      while( *s != '\0' )
      {
         n = cl_GetOptionNo(list, &s);
         if ( n < 0 )
         {
            if ( n == -2 && cl_file_cnt < CL_FILE_MAX )
            {
               strcpy(cl_file_list[cl_file_cnt++], s);
               break;
            }
            else
            {
               ret = 0;       /* unbekannte option */
               break;
            }
         }
         switch( cl_Convert(list+n, &s, 0) )
         {
            case -1:
               ret = 0;       /* fehler */
               break;
            case 1:
               break;
            case 0:
               i++;
               s = argv[i];
               if ( s == NULL )
               {
                  ret = 0;       /* fehler */
                  break;
               }
               if ( cl_Convert(list+n, &s, 1 ) != 1 )
               {
                  ret = 0;       /* fehler */
                  break;
               }
               break;
         }
         if ( argv[i] == NULL )
            break;
      }
      if ( argv[i] == NULL )
         break;
      i++;
      if ( argv[i] == NULL )
         break;
   }
   return ret;
}



/* #define cmdline_main_test */
#ifdef cmdline_main_test

int toggle = 0;
long value = 0;
long set = 0;
char str[100] = "test";
char es[100] = "\n";

cl_entry_struct cl_list[] =
{
   { CL_TYP_SET,  "blue-blue color",      &set,  0 },
   { CL_TYP_SET,  "red-red color",        &set,  1 },
   { CL_TYP_SET,  "green-green color",    &set,  2 },
   
   { CL_TYP_LONG, "val-assign a value",   &value,  0 },  
   
   { CL_TYP_ON,   "on-toggle enabled",     &toggle,  0 },
   { CL_TYP_OFF,  "off-toggle disabled",   &toggle,  0 },
   
   { CL_TYP_STRING,  "str-assign a character string",  str,  100 },
   
   { CL_TYP_ESCSEQ,  "es-assign a character string with escape seqences",  es,  100 },
   
   CL_ENTRY_LAST
};


void main(int argc, char *argv[])
{
   int i;
   if ( cl_Do(cl_list, argc, argv) == 0 )
   {
      puts("cmdline error");
      exit(1);
   }
   
   if ( cl_file_cnt == 0 )
   {
      puts("options:");
      cl_OutHelp(cl_list, stdout, " ", 7);
      exit(1);
   }
     
   printf("toggle: %d\n", toggle);
   printf("value:  %ld\n", value);
   printf("set:    %ld\n", set);
   printf("str:    %s\n", str);
   
   for( i = 0; i < cl_file_cnt; i++ )
      printf("%d.: %s\n", i, cl_file_list[i]);

}

#endif
