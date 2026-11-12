#ifndef __OLED_DATA_H
#define __OLED_DATA_H

#include <stdint.h>

/* ASCII font data. The image data (Diode) was stripped for this port. */

extern const uint8_t OLED_F8x16[][16];
extern const uint8_t OLED_F6x8[][6];

/* ---- 汉字字模 ----
 * ⚠️ 取 3 = UTF-8（本工程源码就是 UTF-8; 取 2 是 GB2312, 会切错字节）。
 * 一个汉字 = 16x16 像素 = 32 字节, 版式见 OLED_Data.c 顶部注释。 */
#define OLED_CHN_CHAR_WIDTH 3

typedef struct
{
    char    Index[OLED_CHN_CHAR_WIDTH + 1];   /* 汉字索引（UTF-8, 与源码字面量逐字节比较）*/
    uint8_t Data[32];                         /* 字模数据（16x16）*/
} ChineseCell_t;

extern const ChineseCell_t OLED_CF16x16[];

#endif
