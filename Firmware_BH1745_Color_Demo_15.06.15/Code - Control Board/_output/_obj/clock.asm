;; Compile Options : /TML610111 /MS /near /Icommon /Imain /Iirq /Itimer /Iclock /Itbc /Ipwm /Iuart /Ivolume /Iled /Ii2c /SS 256 /SD /Oa /Ot /W 1 /Ff /Fa_output\_obj\ 
;; Version Number  : Ver.3.41.8
;; File Name       : clock.c

	type (ML610111) 
	fastfloat
	model small, near
	$$clk_setSysclk$clock segment code 2h #0h
CVERSION 3.41.8
CGLOBAL 01H 03H 0000H "clk_setSysclk" 08H 02H 00H 00H 80H 00H 00H 00H 07H
CSTRUCTTAG 0000H 0000H 0000H 0008H 00000001H "_Notag"
CSTRUCTMEM 52H 00000001H 00000000H "b0" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000001H "b1" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000002H "b2" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000003H "b3" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000004H "b4" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000005H "b5" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000006H "b6" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000007H "b7" 02H 00H 00H
CTYPEDEF 0000H 0000H 43H "_BYTE_FIELD" 04H 00H 05H 00H 00H
CFILE 0001H 00000028H "main\\mcu.h"
CFILE 0002H 000007EEH "main\\ML610111.H"
CFILE 0003H 00000022H "clock\\clock.h"
CFILE 0004H 00000057H "irq\\irq.h"
CFILE 0005H 000001B6H "timer\\timer.h"
CFILE 0000H 00000064H "clock\\clock.c"

	rseg $$clk_setSysclk$clock
CFUNCTION 0

_clk_setSysclk	:
CBLOCK 0 1 74

;;{
CLINEA 0000H 0001H 004AH 0001H 0001H
CBLOCK 0 2 74

;;	SYSC0  = 0;
CLINEA 0000H 0001H 0051H 0002H 000CH
	rb	0f002h.0

;;	SYSC1  = 0;	// 8MHz OSCLK(==OSCLK/2)
CLINEA 0000H 0001H 0052H 0002H 0025H
	rb	0f002h.1
CBLOCKEND 0 2 99

;;}
CLINEA 0000H 0001H 0063H 0001H 0001H
	rt
CBLOCKEND 0 1 99
CFUNCTIONEND 0

	public _clk_setSysclk
	extrn code near : _main

	end
