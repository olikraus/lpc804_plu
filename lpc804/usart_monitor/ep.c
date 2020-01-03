
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "ep.h"


/*=================================================*/

void ep_init(ep_t *ep, void (*out)(ep_t *ep, int c))
{
  memset(ep, 0, sizeof(ep_t));
  ep->out = out;
}

/*=================================================*/


void ep_out_char(ep_t *ep, int c)
{
  if ( c == '\n' )
    ep->out(ep, '\r');		// windows sequence is \r\n
  ep->out(ep, c);
}

void ep_out_str(ep_t *ep, const char *s)
{
  while( *s != '\0' )
  {
    ep_out_char(ep, *s);
    s++;
  }
}


void ep_out_4str(ep_t *ep, uint32_t n)
{
  int j;
  uint32_t v;
  for( j = 0; j < 4; j++ )
  {
    v = n;
    v >>= 24;
    if ( v == 0 )
      ep_out_char(ep, ' ');
    else
      ep_out_char(ep, v);
    n <<= 8;
  }
}

void ep_to_dec_num(ep_t *ep, uint32_t n)
{
  int i = 9;
  uint32_t r;
  ep->num_start = i;
  do
  {
    r = n % 10;
    ep->num_buf[i] = r + '0';
    n /= 10;
    if ( n == 0 && r != 0 )
      ep->num_start = i;
    i--;
  } while( i >= 0 );
}

/*
  base = 0: bin
  base = 1: dec
  base = 2: hex
*/
void ep_out_num(ep_t *ep, uint32_t num, uint8_t base, uint8_t size)
{
  int i;
  uint32_t v;
  if ( base == 0 )	// binary
  {
    i = size-1;
    while( i >= 0 )
    {
      v = num;
      v >>= i;
      if ( v & 1 )
	ep_out_char(ep, '1');
      else
	ep_out_char(ep, '0');
      i--;
    }    
  }
  else if ( base == 1 )	// dezimal
  {
    ep_to_dec_num(ep, num);
    i = size - (10-ep->num_start);
    while( i > 0 )
    {
      ep_out_char(ep, ' ');
      i--;
    }
    for( i = ep->num_start; i < 10; i++ )
    {
      ep_out_char(ep, ep->num_buf[i]);
    }
  }
  else		// hex
  {
    i = size-1;
    while( i >= 0 )
    {
      v = num;
      v >>= i*4;
      v &= 15;
      if ( v >= 10 )
	v += 'a'-10;
      else
	v += '0';
      ep_out_char(ep, v);
      i--;
    }
  }
}

void ep_out_num_all_base_with_size(ep_t *ep, uint32_t num, uint8_t ld, uint8_t lh, uint8_t lb)
{
  ep_out_num(ep, num, 1, ld);
  ep_out_str(ep, " $");
  ep_out_num(ep, num, 2, lh);	
  ep_out_str(ep, " %");
  ep_out_num(ep, num, 0, lb);
}

void ep_out_num8_all_base(ep_t *ep, uint32_t num)
{
  ep_out_num_all_base_with_size(ep, num, 3, 2, 8);
}

void ep_out_num16_all_base(ep_t *ep, uint32_t num)
{
  ep_out_num_all_base_with_size(ep, num, 5, 4, 16);
}

void ep_out_num32_all_base(ep_t *ep, uint32_t num)
{
  ep_out_num_all_base_with_size(ep, num, 10, 8, 32);
}

/* size=0: 1 byte, size=1: 2 bytes, size=3: 4 bytes */
void ep_out_num_all_base(ep_t *ep, uint32_t num, uint8_t size)
{
  if ( size==0 )
    ep_out_num8_all_base(ep, num);
  else if ( size==1 )
    ep_out_num16_all_base(ep, num);
  else
    ep_out_num32_all_base(ep, num);
}



/*=================================================*/

uint32_t ep_parse_expr(ep_t *ep);

/*=================================================*/

/* size: 0->bytes, 1->word, 2->quad word */

uint32_t ep_get_mem_value(ep_t *ep, uint32_t adr, uint8_t size)
{
  if ( size == 0 )
    return (uint32_t)*(uint8_t *)adr;
  if ( size == 1 )
    return (uint32_t)*(uint16_t *)adr;

  return *(uint32_t *)adr;
  
}

/* size: 0->bytes, 1->word, 2->quad word */
void ep_exec_cmd_list_bytes(ep_t *ep, uint32_t adr, uint32_t lines_per_list, uint8_t size)
{
  while( lines_per_list > 0 )
  {
    ep_out_char(ep, '$');
    ep_out_num(ep, adr, 2, 8);
    ep_out_char(ep, ':');
    ep_out_char(ep, ' ');
    
    ep_out_num_all_base(ep, ep_get_mem_value(ep, adr, size), size);
    
    ep_out_char(ep, '\n');
    lines_per_list--;
    adr+= 1<<size;
  }
  
}

/* size: 0->bytes, 1->word, 2->quad word */
void ep_parse_address_arg(ep_t *ep, uint32_t bytes_per_line)
{
  if ( ep->current_type != EP_CURRENT_TYPE_END )
    ep->address = ep_parse_expr(ep);
  else
    ep->address += ep_get_lines_per_list_cmd(ep)*bytes_per_line;
}

int ep_parse_cmd_list_bytes(ep_t *ep)
{
  ep_parse_address_arg(ep, 1);
  ep_exec_cmd_list_bytes(ep, ep->address, ep_get_lines_per_list_cmd(ep), 0);
  return 1;
}

int ep_parse_cmd_list_words(ep_t *ep)
{
  ep_parse_address_arg(ep, 2);
  ep_exec_cmd_list_bytes(ep, ep->address, ep_get_lines_per_list_cmd(ep), 1);
  return 1;
}

int ep_parse_cmd_list_quads(ep_t *ep)
{
  ep_parse_address_arg(ep, 4);
  ep_exec_cmd_list_bytes(ep, ep->address, ep_get_lines_per_list_cmd(ep), 2);
  return 1;
}

/*=================================================*/

const ep_cmd_t ep_cmd_list[] = 
{
  { "lb", ep_parse_cmd_list_bytes },
  { "lw", ep_parse_cmd_list_words },
  { "lq", ep_parse_cmd_list_quads },
};

/*=================================================*/

void ep_set_current_type(ep_t *ep)
{
  ep->current_type = EP_CURRENT_TYPE_END;
  if ( ep->current == 0 )
    return;
  ep->current_type = EP_CURRENT_TYPE_WHITESPACE;
  if ( ep->current <= ' ' )
    return;
  ep->current_type = EP_CURRENT_TYPE_DIGIT;
  if ( ep->current >= '0' && ep->current <= '9' )
    return;
  ep->current_type = EP_CURRENT_TYPE_LOWER;
  if ( ep->current >= 'a' && ep->current <= 'z' )
    return;
  ep->current_type = EP_CURRENT_TYPE_UPPER;
  if ( ep->current >= 'A' && ep->current <= 'Z' )
    return;
  ep->current_type = EP_CURRENT_TYPE_OTHER;
}

void ep_next(ep_t *ep)
{
  if ( *ep->current_ptr != '\0' ) 
    ep->current_ptr++;
  ep->current = *ep->current_ptr;
  ep_set_current_type(ep);
}

void ep_skip_space(ep_t *ep)
{
  while( ep->current_type == EP_CURRENT_TYPE_WHITESPACE )
    ep_next(ep);
}

uint32_t ep_parse_hex(ep_t *ep)
{
  uint32_t v = 0;
  for(;;)
  {
    if ( ep->current_type == EP_CURRENT_TYPE_DIGIT )
    {
      v *= 16;
      v+=ep->current-'0';
    }
    else if ( ep->current_type == EP_CURRENT_TYPE_LOWER )
    {
      v *= 16;
      v+=ep->current-'a'+10;
    }
    else if ( ep->current_type == EP_CURRENT_TYPE_UPPER )
    {
      v *= 16;
      v+=ep->current-'A'+10;
    }
    else
      break;
    ep_next(ep);
  }
  ep_skip_space(ep);
  return v;
}

/* base = 2 or 10 */
uint32_t ep_parse_dec_bin(ep_t *ep, uint32_t base)
{
  uint32_t v = 0;
  for(;;)
  {
    if ( ep->current_type == EP_CURRENT_TYPE_DIGIT )
    {
      v *= base;
      v+=ep->current-'0';
    }
    else
      break;
    ep_next(ep);
  }
  ep_skip_space(ep);
  return v;
}

uint32_t ep_parse_num(ep_t *ep)
{
  if ( ep->current == '%' )
  {
    ep_next(ep);
    return ep_parse_dec_bin(ep, 2);
  }
  if ( ep->current == '$' )
  {
    ep_next(ep);
    return ep_parse_hex(ep);
  }
  return ep_parse_dec_bin(ep, 10);  
}

void ep_parse_identifier(ep_t *ep)
{
  ep->identifier_start = ep->current_ptr;
  ep->identifier_len = 0;
  if ( ep->current_type == EP_CURRENT_TYPE_LOWER || ep->current_type == EP_CURRENT_TYPE_UPPER || ep->current == '_' ) 
  {
    do
    {
      ep->identifier_len++;
      ep_next(ep);
    } while ( ep->current_type == EP_CURRENT_TYPE_DIGIT || ep->current_type == EP_CURRENT_TYPE_LOWER || ep->current_type == EP_CURRENT_TYPE_UPPER || ep->current == '_' );
  }
  ep_skip_space(ep);
}

/* compare s against the parsed identifier, return 1 for match, 0 for not match */
int ep_cmp_identifier(ep_t *ep, const char *s)
{
  size_t i = ep->identifier_len;
  const char *t;
  if ( strlen(s) != i )
    return 0;
  t = ep->identifier_start;
  while( i > 0 )
  {
    if ( *t != *s )
      return 0;
    t++;
    s++;
    i--;
  }
  return 1;
}


uint32_t ep_parse_expr(ep_t *ep)
{
  return ep_parse_num(ep);
}

int ep_parse_cmd(ep_t *ep, const char *s)
{
  int i;
  ep->current_ptr = s;
  ep->current = *ep->current_ptr;
  ep_set_current_type(ep);
  ep_skip_space(ep);
  ep_parse_identifier(ep);
  if ( ep->identifier_len > 0 )
  {
    for( i = 0; i < sizeof(ep_cmd_list)/sizeof(*ep_cmd_list); i++ )
    {
      if ( ep_cmp_identifier(ep, ep_cmd_list[i].name) != 0 )
      {
	return ep_cmd_list[i].fn(ep);
      }
    }
  }
  return 0;	// unkonwn function
}


