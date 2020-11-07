/*

  flash.h

  LPC804 Project 

  Copyright (c) 2020, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  

  Code based on AN12125_LPC802
  https://www.nxp.com/docs/en/application-note/AN12125_LPC802.zip

  Usefull location for "dest_flash_adr" could be 0x7f00
  Note: 0x7f80 / 0x7fc0 should not be used.

*/


#ifndef _FLASH_H
#define _FLASH_H

#include "LPC8xx.h"


int flash_erase_page(uint32_t dest_flash_adr);
int flash_write_page(uint32_t dest_flash_adr, const uint8_t *src_ram_adr);

/*

  Description:
    Write the content of one page (64 bytes) to target location
    Executes erase and write operation.
    On a LPC804 with 15MHz, this call requires about 3ms

  Arguments:
    dest_flash_adr:		Destination address, 64 byte boundary, 0x0000-0x7F7F
    src_ram_adr:		Source location of the page, word boundary

  Return:
    0, in case of any error

*/
int flash_page(uint32_t dest_flash_adr, const uint8_t *src_ram_adr);



#endif /* _FLASH_H */
