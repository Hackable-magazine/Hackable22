org 0x0000

RAMTOP:	equ 0x0040       ; juste au dessus du max

	LD SP,RAMTOP     ; initialisation pointeur de pile
PLOP:	LD A,(VAR)       ; on charge ce qu'il y a à VAR dans A
	INC A            ; on incrémente A
	LD (VAR),A       ; on prend ce qu'il y a dans A et le met à VAR
	NOP
	NOP
	PUSH AF          ; A puis F sur la pile
	NOP
	POP AF           ; F puis A sont pris sur la pile
	NOP
	CALL ROUT        ; on appel la routine ROUT
	JP PLOP          ; on boucle en sautant vers PLOP

; routine / procédure
ROUT:	NOP
	NOP
	NOP
	RET              ; retour

; ici vie notre valeur
VAR:	db	0x42
