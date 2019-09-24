/*

  b_sp.c  (string parse)

  some very simple string parsing functions

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


void b_skipspace(char **s) 
{  
   while(**s <= ' ' && **s > '\0')
      (*s)++;
}

char *b_getid(char **s)
{
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
}

