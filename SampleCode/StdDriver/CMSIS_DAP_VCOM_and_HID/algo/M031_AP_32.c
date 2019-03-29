/* Flash OS Routines (Automagically Generated)
 * Copyright (c) 2009-2015 ARM Limited
 */
#include "flash_blob.h"

static const uint32_t flash_code[] = {
    0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,
    0x4603B530, 0x2164460C, 0x4D812059, 0x20166028, 0x20886028, 0x46286028, 0x07C06800, 0xD1010FC0,
    0xBD302001, 0x6800487B, 0x43282504, 0x60284D79, 0x68404628, 0x43282504, 0x60684D76, 0x4608BF00,
    0x28001E49, 0x4874D1FB, 0x25216800, 0x4D724328, 0x46286028, 0x250169C0, 0x4D6F4328, 0x462861E8,
    0x07C06800, 0xD1010FC0, 0xE7DA2001, 0x6800486A, 0x43282540, 0x60284D68, 0xE7D22000, 0xBF004601,
    0x69004865, 0x0FC007C0, 0x4863D1FA, 0x22216800, 0x4A614390, 0x46106010, 0x084069C0, 0x61D00040,
    0x47702000, 0xBF004601, 0x6900485B, 0x0FC007C0, 0x4859D1FA, 0x22406800, 0x4A574310, 0x20226010,
    0x088860D0, 0x60500080, 0x05402001, 0xD1014281, 0x60904852, 0x4A502001, 0xF3BF6110, 0xBF008F60,
    0x6900484D, 0x0FC007C0, 0x484BD1FA, 0x22406800, 0xD0064210, 0x68004848, 0x4A474310, 0x20016010,
    0x20004770, 0xB570E7FC, 0x460C4603, 0xE0092500, 0x1C6D4629, 0x58580089, 0xFFC4F7FF, 0xD0010006,
    0xBD704630, 0xD3F342A5, 0xE7FA2000, 0x4603B510, 0x08811CC8, 0xBF000089, 0x69004837, 0x0FC007C0,
    0x4835D1FA, 0x24406800, 0x4C334320, 0xE0206020, 0x4C312021, 0x089860E0, 0x60600080, 0x60A06810,
    0x61202001, 0x8F60F3BF, 0x482BBF00, 0x07C06900, 0xD1FA0FC0, 0x68004828, 0x42202440, 0x4826D006,
    0x43206800, 0x60204C24, 0xBD102001, 0x1D121D1B, 0x29001F09, 0x2000D1DC, 0xB510E7F7, 0x08991CCB,
    0xBF000089, 0x691B4B1C, 0x0FDB07DB, 0x4B1AD1FA, 0x2440681B, 0x4C184323, 0xE0256023, 0x4C162300,
    0x088360E3, 0x6063009B, 0x60A32300, 0x61232301, 0x8F60F3BF, 0x4B10BF00, 0x07DB691B, 0xD1FA0FDB,
    0x681B4B0D, 0x42232440, 0x4B0BD005, 0x4323681B, 0x60234C09, 0x4B08BD10, 0x6814689B, 0xD00042A3,
    0x1D00E7F8, 0x1F091D12, 0xD1D72900, 0xE7F1BF00, 0x40000100, 0x40000200, 0x4000C000, 0x0055AA03,
    0x00000000
};

const program_target_t flash_algo = {
    0x20000021,  // Init
    0x2000009D,  // UnInit
    0x2000001F,  // EraseChip
    0x200000C5,  // EraseSector
    0x2000014D,  // ProgramPage

    // BKPT : start of blob + 1
    // RSB  : address to access global/static data
    // RSP  : stack pointer
    {
        0x20000001,
        0x20000C00,
        0x20001000
    },

    0x20000400,  // mem buffer location
    0x20000000,  // location to write prog_blob in target RAM
    sizeof(flash_code),  // prog_blob size
    flash_code,  // address of prog_blob
    0x00000200,  // ram_to_flash_bytes_to_be_written
};
