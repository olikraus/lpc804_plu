
#ifndef EP_H
#define EP_H

#define EP_CURRENT_TYPE_END 0
#define EP_CURRENT_TYPE_WHITESPACE 1
#define EP_CURRENT_TYPE_DIGIT 2
#define EP_CURRENT_TYPE_LOWER 3
#define EP_CURRENT_TYPE_UPPER 3
#define EP_CURRENT_TYPE_OTHER 4

typedef struct ep_cmd_struct ep_cmd_t;
typedef struct ep_struct ep_t;

struct ep_cmd_struct
{
  const char name[4];
  int (*fn)(ep_t *);
};


struct ep_struct
{
  void (*out)(ep_t *ep, int c);  
  uint32_t address;
  int current;
  int current_type;
  const char *current_ptr;
  const char *identifier_start;
  int identifier_len;
  
  char num_buf[10];
  uint16_t num_start;
  
};

#define ep_get_lines_per_list_cmd(ep) 16

void ep_init(ep_t *ep, void (*out)(ep_t *ep, int c));
void ep_out_num(ep_t *ep, uint32_t num, uint8_t base, uint8_t size);
int ep_parse_cmd(ep_t *ep, const char *s);


#endif