; list names of your variables one in a row below
; each of them will be allocated a GPR
; and can be referenced by name in your code
; (the assembler will place actual address automatically)
; examples:
;Ctr1	RES	1	; your vaiable, e.g., for counter 1
;Ctr2	RES	1
Temp	Res	1	; this is used by the examples
A		Res	1
Bop		Res	1
LFSR_STATE  Res	1
LFSR_MASK   Res	1
Count	Res 1
lfsr_reg    Res	1
bit_pos	Res 1
	
Temp_W	Res 1
Temp_Status Res	1

State	Res 1
	
ADC_HI Res  1
ADC_PREV    Res	1
    
VPWM_Speed  Res	1
VPWM_Bright1	Res 1
VPWM_Bright2	Res 1
	
US_CNT	Res 1
	
Bright_Difference   Res	1

Strobe_Speed	Res 1
Strobe_Direction    Res	1
Strobe_Current	Res 1
Strobe_FB   Res	1
    
LFSR_Speed  Res	1

  
Bright_DI   Res	1
;   
Bright_Const	Res 1
   
Program_State	Res 1
	
value	Res 1
	
configmode  Res	1
  
Bright_Choice1	Res 1
Bright_Choice2	Res 1