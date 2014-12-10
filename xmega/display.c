/*
 * display.c
 *
 * Created: 12/9/2014 8:42:40 PM
 *  Author: Ian
 */ 
#include <asf.h>
#include "display.h"

PORT_t *control_port;
PORT_t *data_port;

void init_display(PORT_t *data, PORT_t *control){
	control_port = control;
	data_port = data;
	
	data->DIRSET |= 0xFF;
	data->OUTCLR |= 0xFF;
	//Upper 4 bits for control.
	control->DIRSET |= 0xF0;
	control->OUTCLR =  0xF0;
}

void clear_display(){
	data_port->OUTCLR = 0xFF;
	control_port->OUTSET &= 0x1F;
	control_port->OUTSET &= 0x2F;
	control_port->OUTSET &= 0x4F;
	control_port->OUTSET &= 0x8F;
}

void clear_seg(segment seg){
	set_seg(seg);
	data_port->OUTCLR = 0xFF;
}

void set_seg(segment seg){
	switch (seg){
	case SEG4:
		control_port->OUTSET &= 0x1F;
		break;
	case SEG3:
		control_port->OUTSET &= 0x2F;
		break;
	case SEG2:
		control_port->OUTSET &= 0x4F;
		break;
	case SEG1:
		control_port->OUTSET &= 0x8F;
		break;
	default:
		break;
	}
}

void write_num(uint8_t output){
	switch (output)
	{
	case 0x00:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = zero_bm;
		break;
	case 0x01:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = one_bm;
		break;
	
	case 0x02:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = two_bm;
		break;
	
	case 0x03:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = three_bm;
		break;
	
	case 0x04:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = four_bm;
		break;
	
	case 0x05:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = five_bm;
		break;
	
	case 0x06:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = six_bm;
		break;
	
	case 0x07:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = seven_bm;
		break;
	
	case 0x08:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = eight_bm;
		break;
	
	case 0x09:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = nine_bm;
		break;
	
	case 0x0A:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = a_bm;
		break;
	
	case 0x0B:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = b_bm;
		break;
	
	case 0x0C:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = c_bm;
		break;
	
	case 0x0D:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = d_bm;
		break;
	
	case 0x0E:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = e_bm;
		break;
	
	case 0x0F:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = f_bm;
		break;
	
	default:
		data_port->OUTCLR = 0xFF;
		data_port->OUTSET = underscore_bm;
		break;
	}	
}

void write_state(uint8_t state){
	clear_display();
	set_seg(SEG1);
	data_port->OUTSET = s_bm;
	set_seg(SEG2);
	data_port->OUTSET = t_bm;
	set_seg(SEG3);
	write_num(((state & 0xF0) >> 4));
	set_seg(SEG4);
	write_num((state & 0x0F));
}