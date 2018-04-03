#ifndef __FONT_NUM_H
#define __FONT_NUM_H
#include "game.h"
#define SHADOW_OFFSET 2
void Font_DrawNum_Big( u16 x, u16 y, u8 * num );
void Font_DrawNum_Small( u16 x, u16 y, u8 * num );
void Font_Num_DrawNum( u16 x, u16 y, u16 num, u8 mode );
void Font_DrawNum_Black( u16 x, u16 y, u8 * num );
void Font_Num_DrawNum_Multi( u16 x, u16 y, u32 num, u8 mode );
#endif
