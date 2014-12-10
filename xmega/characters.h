/*
 * characters.h
 *
 * Created: 12/9/2014 9:33:03 PM
 *  Author: Ian
 */ 


#ifndef CHARACTERS_H_
#define CHARACTERS_H_

//**A**
//F   B
//**G**
//E   C
//**D**
//
//Bit order:  (7) A B C D E F G H (0)

//Numbers
#define zero_bm		0b11111100
#define one_bm		0b01100000
#define two_bm		0b11011010
#define three_bm	0b11110010
#define four_bm		0b01100110
#define five_bm		0b10110110
#define six_bm		0b10111110
#define seven_bm	0b11100000
#define eight_bm	0b11111110
#define nine_bm		0b11100110

//Letters
#define a_bm	0b11101110
#define b_bm	0b00111110
#define c_bm	0b10011100
#define d_bm	0b01111010
#define e_bm	0b10011110
#define f_bm	0b10001110

#define s_bm	0b10110110

#define t_bm	0b00011110

#define underscore_bm 0b00001000




#endif /* CHARACTERS_H_ */