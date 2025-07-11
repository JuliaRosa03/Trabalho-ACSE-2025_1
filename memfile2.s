MAIN	SUB R0, R15, R15 		; R0 = 0				
	ADD R2, R0, #5      	; R2 = 5             
	ADD R3, R0, #12    	; R3 = 12            
	SUB R7, R3, #9    	; R7 = 3             
	ORR R4, R7, R2    	; R4 = 3 OR 5 = 7              	  		
        AND R5, R3, R4    	; R5 = 12 AND 7 = 4            	
	ADD R5, R5, R4    	; R5 = 4 + 7 = 11              			
        SUBS R8, R5, R7    	; R8 <= 11 - 3 = 8, set Flags   	  		
        BEQ END        		; shouldn't be taken            	  		
        SUBS R8, R3, R4    	; R8 = 12 - 7  = 5             			
        BGE AROUND       	; should be taken               
	ADD R5, R0, #0     	; should be skipped             	
AROUND   
	SUBS R8, R7, R2   	; R8 = 3 - 5 = -2, set Flags   	         	
        ADDLT R7, R5, #1  	; R7 = 11 + 1 = 12				          	
        SUB R7, R7, R2    	; R7 = 12 - 5 = 7				
    	STR R7, [R3, #84]  	; mem[12+84] = 7		     	
	LDR R2, [R0, #96]  	; R2 = mem[96] = 7				
	ADD R15, R15, R0	; PC <- PC + 8 (skips next)     	         
	ADD R2, R0, #14    	; shouldn't happen              	
	B END             	; always taken					
	ADD R2, R0, #13   	; shouldn't happen				
  	ADD R2, R0, #10		; shouldn't happen			  
END	STR R2, [R0, #100] 	; mem[100] = 7
    MOV R1, #10  ; R1 = 10
	MOV R2, R3  ; R2 = 12
	CMP R1,R2  ; R1-R2 & set Flags
	BLT LOAD  ; should be taken
	ADD R3, R1, #5  ; shouldn't be taken

LOAD MOV R3,#0xF  ; R3 = 15
	LSL R4, R3, #28  ; R4 = R3 << 28 = 15 << 28 = -268435456
	ORR R5, R4, #32  ; R5 = R4 OR 32 = -268435424
	ASR R6, R5, #3  ; R6 = R5 >>> 3 = -33554428
	ASR R7, R3, #2  ; R7 = R3 >>> 2 = 3
	CMP R3, R3
	STR R6, [R0, #108]  ; mem[108] = -33554428
	BL COMPARE  ; should be taken
	MOV R10, #0xA
	B DONE

COMPARE TST R1, #0  ; 10 AND 0 = 0 & set Flags
	SUBNE R9, R1, R3  ; shouldn't be taken
	EOREQ R9, R1, R3  ; R9 = 10 XOR 15 = 5
	MOV R15, R14  ; PC <- LR

DONE STR R9, [R0, #112]  ; mem[112] = 5
	ADD R12, R7, R2, LSL #2  ; R12 = 3 + (12 << 2) = 51
    STR R12, [R0, #116]      ; mem[116] = 51