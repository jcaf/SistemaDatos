/*
 * File:   lcdan_aux.h
 * Author: jcaf
 *
 * Created on February 12, 2015, 10:24 AM
 */

#ifndef LCDAN_AUX_H
#define	LCDAN_AUX_H
#include "lcdan.h"

#define LCDAN_STR_MAXSIZE (LCDAN_COL+1)// 16/20 chars printable + '\0'

#define lcdan_set_cursor_in_row0(x) do{lcdan_write_cmd(LCDAN_BASEADDR_ROW_0+x);__delay_us(10);}while(0)
#define lcdan_set_cursor_in_row1(x) do{lcdan_write_cmd(LCDAN_BASEADDR_ROW_1+x);__delay_us(10);}while(0)
#define lcdan_set_cursor_in_row2(x) do{lcdan_write_cmd(LCDAN_BASEADDR_ROW_2+x);__delay_us(10);}while(0)
#define lcdan_set_cursor_in_row3(x) do{lcdan_write_cmd(LCDAN_BASEADDR_ROW_3+x);__delay_us(10);}while(0)

#define lcdan_clear() do{lcdan_write_cmd(LCDAN_DISP_CLEAR);__delay_ms(2);}while(0)

void lcdan_print_string(const char *p);
void lcdan_print_PSTRstring(const char *p);
void lcdan_set_cursor(int8_t x, int8_t y);


//+--ADDED:
void lcdan_str_lineformat_align(char *str_dest, const char *str_src, int8_t pos_i);//int8_t pos_i = 0
void lcdan_str_lineformat_align_P(char *str_dest, const char *str_src, int8_t pos_i);//int8_t pos_i = 0
void lcdan_str_lineformat_trimEOL3dots(char *str);
uint8_t lcdan_str_get_align_col(const char *str_src, int8_t col_align);
#define LCDAN_STR_FORMAT_ALIGN_CENTER -1
int8_t lcdan_str_get_center_hor(const char *str);
//--+

void lcdan_prepare_str_clearformat(char *str_dest, int8_t str_dest_maxsize,  const char *str_src, int8_t pos_i /* = 0 */);

void lcdanBuff_clear(char str_dest[LCDAN_ROW][LCDAN_COL]);
//void lcdanBuff_print_string(int8_t x, int8_t y, char * str_dest, const char * str_src);
void lcdanBuff_print_string(int8_t col, int8_t row, char str_dest[LCDAN_ROW][LCDAN_COL], const char * str_src);
void lcdanBuff_dump2device(const char str_src[LCDAN_ROW][LCDAN_COL]);

#ifdef	__cplusplus
extern "C" {
#endif


#ifdef	__cplusplus
}
#endif

#endif	/* LCDAN_AUX_H */

