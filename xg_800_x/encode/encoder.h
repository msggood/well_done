/*
 * encoder.h
 *
 *  Created on: 2019��6��25��
 *      Author: zhou
 */

#ifndef SRC_ENCODER_H_
#define SRC_ENCODER_H_

void genTurboInterleaver( short code_len, short *interleaver_tab);
void encoder(char *Src,char *Dst,short *l_IntTab,short code_len,char rate );


#endif /* SRC_ENCODER_H_ */
