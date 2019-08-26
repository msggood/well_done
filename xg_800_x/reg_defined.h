/*
 * reg_defined.h
 *
 *  Created on: 2019Äê7ÔÂ18ÈÕ
 *      Author: zhou
 */

#ifndef SRC_REG_DEFINED_H_
#define SRC_REG_DEFINED_H_

#define BASED_ADDR       0x40000000

#define FPGA_VERSION_OFFSET    0x00    
#define DMA_LENGTH_OFFSET      0x04
#define INTR_GAP_OFFSET        0x08
#define TX_10M_GAP_OFFSET      0X0c
#define TX_LED_GAP_OFFSET      0X14

typedef unsigned int uint32_t;                
int reg_init(uint32_t base_addr,uint32_t end);
void writew(uint32_t addr,uint32_t value);    
uint32_t readw(uint32_t addr);                
void reg_exit();                              


#endif /* SRC_REG_DEFINED_H_ */
