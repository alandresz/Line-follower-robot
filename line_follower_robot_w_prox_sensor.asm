; seguidor_de_linea.asm
;
; Created: 10/17/2018 11:42:23 AM
; Author : ALAN DRESZMAN
;

;		DEFINICION DE CONSTANTES USADAS		;

; *** Velocidades de los motores ***;
.EQU	low_speed_val				= 60		; tiempo de pulso de PWM para velocidad lenta de rueda 
.EQU	med_speed_val_left			= 85		; tiempo de pulso de PWM para velocidad media de rueda izq
.EQU	med_speed_val_right			= 97		; tiempo de pulso de PWM para velocidad media de rueda der
.EQU	high_speed_val				= 140		; tiempo de pulso de PWM para velocidad alta de ruda
.EQU	no_speed_val				= 0
.EQU	force_overfl_timers_val		= 255
										; CAMBIO 7/3/2019 19.43HS : CAMBIE LOS VALORES DE LAS VELOCIDADES PARA MEJORAR EL GIRO

; *** Valores para comparar con lo sensado *** ;
.EQU	is_right_fast		= 0b00001110
.EQU	is_right_normal		= 0b00001101
.EQU	is_left_normal		= 0b00001011
.EQU	is_left_fast		= 0b00000111
.EQU	is_right_one		= 0b00001100
.EQU	is_right_two		= 0b00001000
.EQU	is_left_one			= 0b00000011
.EQU	is_left_two			= 0b00000001
.EQU	is_center_1001		= 0b00001001
.EQU	is_error_stop_0000	= 0b00000000
;.EQU	is_error_stop_1111	= 0b00001111	; CAMBIO 7/3/2019 19.43hs : AGREGUE TODAS ESTAS CONSTANTES PARA ESPECIFICAR NUEVOS COMPORTAMIENTOS

; *** Valor de reset de registros contadores ***;
.EQU	reset_value = 0

; *** Mascara para leer sensores *** ;
.EQU	MASK_SENSOR = 0x0F

;	      DEFINICION DE NOMBRES PARA LOS REGISTROS USADOS         ;

; ***	Registros de los motores
.EQU	motor_der = OCR0A
.EQU	motor_izq = OCR0B

;        Constantes ULTRASONIDO    ;

.EQU CONST_DIST = 3 ;Esta es la constante que sirve para determinar cuan lejos se detecta el obstaculo. 
.EQU FLAG_TIMER = 2 ; Valor de R30 para el cual se ejecuta la rutina del ultrasonido (cuanto mas grande, mas tarda)
.EQU PRESCALER = 1 ;Prescaler para el timer del delay (Puede ser del 1 al 5, por lo que el prescaler sería dividido 1, 8, 64, 256,1024)
.EQU CONST_TIMER = 240; Valor al que empiezo el timer del delay. Cuanto menor sea, más tarda


; ***	Registros usados como timers de delay	
.DEF	delay_timer_1 = R17
.DEF	delay_timer_2 = R18

; ***	Registros usados para almacenar las velocidades de los motores
.DEF	no_speed			= R23
.DEF	med_speed_left		= R24
.DEF	med_speed_right		= R20
.DEF	high_speed			= R25
.DEF	low_speed			= R26
.DEF	force_ovfl_timers	= R30

; *** Registro usado para almacenar los valores de los sensores
.DEF	sensor = R22

; *** Registro usado temporalmente para cargar los valores de configuracion del TIMER0 usado para el PWM
.DEF	temp_pwm_config_reg = R21

.ORG 0x00
JMP main

.ORG 0x0014
JMP INT_TIMER

.ORG 0x001A
JMP INT_DELAY

tabla:

.db 0x7E, 0x30; 0 1
.db 0x6D, 0x79; 2 3
.db 0x33, 0x5B; 4 5
.db 0x5F, 0x70; 6 7
.db 0x7F, 0x7B; 8 9
.db 0x4F, 0x00; E PADDING


main:        

; *** Inicializacion del SP ***;


   	LDI 	R28, HIGH(RAMEND)
	OUT 	SPL, R28
	LDI 	R28, LOW(RAMEND)
	OUT 	SPH, R28

	ldi		R28, 0xFF
	out		DDRC, R28
	ldi		r19, 0
	SEI

	SBI		DDRB, 1 ;salida (trig pin)
	CBI		DDRB, 0 ;entrada (echo pin)
	sbi		ddrb, 2

	call	iniciar_timer_delay ; Preparo el timer usado para el delay del medio segundo aprox

; *** 
	JMP 	start_engine


start_engine:
;**** CONFIGURACION DE LOS PUERTOS DE ENTRADA (SENSORES) Y DE SALIDA (MOTORES) ****;

	CBI 	DDRD, 0					; seteo el pin0 del registro D como entrada (sensor DD)	(PIN2)
	CBI 	DDRD, 1					; seteo el pin1 del registro D como entrada	(sensor DC)	(PIN3)
	CBI 	DDRD, 2					; seteo el pin2 del registro D como entrada	(sensor IC)	(PIN4)
	CBI 	DDRD, 3					; seteo el pin3 del registro D como entrada (sensor II)	(PIN5)
	SBI 	DDRD, 5       			; seteo el pin5 del registro D como salida (RUEDA DERECHA)	 (Salida OCR0B, PIN11)
	SBI 	DDRD, 6       			; seteo el pin6 del registro D como salida (RUEDA IZQUIERDA) (Salida OCR0A, PIN12)

;**** CONFIGURACION DE REGISTROS DEL TIMER0 (PWM) ****;

	LDI		temp_pwm_config_reg, (1<<COM0A1)|(1<<COM0B1)|(0<<WGM01)|(1<<WGM00) ; Config: Non-inverted OC0A, OC0B - Fast PWM
	OUT		TCCR0A, temp_pwm_config_reg

	LDI		temp_pwm_config_reg, (0<<CS02)|(0<<CS01)|(1<<CS00)|(0<<WGM02) ; Config: SET DEL PRESCALER (010 -> 8  011 -> 64  100 -> 256  101->1024)
	OUT		TCCR0B, temp_pwm_config_reg


; **** CARGA DE VALORES DE VELOCIDAD EN REGISTROS ****;
	
	LDI		low_speed, low_speed_val
	LDI		med_speed_left, med_speed_val_left
	LDI		med_speed_right, med_speed_val_right
	LDI		high_speed, high_speed_val
	LDI		no_speed, no_speed_val
	LDI		force_ovfl_timers, force_overfl_timers_val

	JMP 	go

go:
	cpi		r19,FLAG_TIMER ;Comparo R30 con una constante, como R30 se incrementa en la interrupcion del timer, cuenta cuantas vueltas dio el timer. R30 sería el flag
	brge	llamar_funcion 
	cpi		r27, 1
	brne	go_go

	OUT		motor_der, no_speed			; la rueda der va a 0
	OUT		motor_izq, no_speed			; la rueda izq va a 0
	OUT		TCNT0, force_ovfl_timers	; fuerzo overflow del timer (TCNT0 = 0xFF) para que se actualicen OCR0A y OCR0B
	CALL	reset_delay_timers			; reset de timers de delay
	CALL	delay						; delay de 0.5 seg	
	jmp		go

llamar_funcion:
	ldi r19, 0  ;Si llego al valor crítico, lo reseteo
	sts TCCR1B, R19 ;Deshabilito el timer para después prepararlo para el ultrasonido
	call funcion
	jmp go

funcion:
	call iniciar_timer_ICR	;Inicio el timer para el ICR del ultrasonido
	CBI PORTB, 1
	CALL delay_2us
	SBI PORTB, 1
	CALL delay_2us
	CALL delay_2us
	CALL delay_2us               ;Pongo el trig en 0 por un cachito, despues en 1 por 10 us como se deberia hacer
	CALL delay_2us
	CALL delay_2us
	CBI PORTB, 1
/*start:	
	IN R29, PINB  ;En start se busca el primer momento en que llega el 1 al echo
	ANDI R29, 1
	CPI R29, 1
	BRNE start*/
	STS TCCR1B, R28 ;Cuando el echo recibe el pulso empiezo el timer (R28 se seteó en la rutina iniciar_timer_ICR)
	ret

; **** Rutina de censado y determinacion de direccion a tomar ***;

go_go:

	IN 		sensor, PIND				; guardo en "sensor" el valor de PIND
	ANDI 	sensor, MASK_SENSOR			; seteo en 0 el nibble alto de "sensor", el resto de los bits quedan con el valor que tengan antes de esta operacion (MASK_SENSOR = 0x0F)			
	

	CPI 	sensor, is_right_normal		; comparo "sensor" con el valor que corresponde a que doble normal hacia la derecha
	BREQ 	turn_right_normal			; si son iguales (o sea, si "sensor" = 0x02) entonces salta a la rutina de doblar normal a la derecha
	
	CPI		sensor, is_right_one
	BREQ	turn_right_normal

	CPI		sensor, is_right_two
	BREQ	turn_right_fast

	CPI		sensor, is_left_normal		; comparo "sensor" con el valor que corresponde a que doble normal hacia la izquierda
	BREQ 	turn_left_normal				; si son iguales (o sea, si "sensor" = 0x04) entonces salta a la rutina de doblar normal a la izquierda

	CPI		sensor, is_left_one
	BREQ	turn_left_normal

	CPI 	sensor, is_right_fast		; comparo "sensor" con el valor que corresponde a que doble rapido hacia la derecha
	BREQ 	turn_right_fast				; si son iguales (o sea, si "sensor" = 0x01) entonces salta a la rutina de doblar rapido a la derecha
	
	CPI		sensor, is_left_fast		; comparo "sensor" con el valor que corresponde a que doble rapido hacia la izquierda
	BREQ 	turn_left_fast				; si son iguales (o sea, si "sensor" = 0x08) entonces salta a la rutina de doblar rapido a la izquierda
	
	CPI		sensor, is_left_two
	BREQ	turn_left_fast
	
	CPI		sensor, is_error_stop_0000
	BREQ	stop

	;CPI	sensor, is_error_stop_1111
	;BREQ	stop
	
	CPI		sensor, is_center_1001
	BREQ	go_straight
	
	JMP 	go_straight					; de lo contrario salta a la de ir derecho


turn_right_normal:
	OUT		motor_der, low_speed		; la rueda der va medio
	OUT		motor_izq, med_speed_left	; la rueda izq va rapido
	OUT		TCNT0, force_ovfl_timers	; fuerzo overflow del timer (TCNT0 = 0xFF) para que se actualicen OCR0A y OCR0B
	CALL	reset_delay_timers			; reset de timers de delay
	CALL	delay						; delay de 0.5 seg
	JMP		go							; vuelta a la rutina de lectura de sensores

turn_left_normal:
	OUT		motor_der, med_speed_right	; la rueda izq va medio
	OUT		motor_izq, low_speed		; la rueda derecha va rapido
	OUT		TCNT0, force_ovfl_timers	; fuerzo overflow del timer (TCNT0 = 0xFF) para que se actualicen OCR0A y OCR0B
	CALL	reset_delay_timers			; reset de timers de delay
	CALL	delay						; delay de 0.5 seg
	JMP		go							; vuelta a la rutina de lectura de sensores

turn_right_fast:
	OUT		motor_der, low_speed		; la rueda der va lento
	OUT		motor_izq, high_speed		; la rueda izq va rapido
	OUT		TCNT0, force_ovfl_timers	; fuerzo overflow del timer (TCNT0 = 0xFF) para que se actualicen OCR0A y OCR0B
	CALL	reset_delay_timers			; reset de timers de delay
	CALL	delay						; delay de 0.5 seg
	JMP		go							; vuelta a la rutina de lectura de sensores

turn_left_fast:
	OUT		motor_der, high_speed		; la rueda izq va lento
	OUT		motor_izq, low_speed		; la rueda derecha va rapido
	OUT		TCNT0, force_ovfl_timers	; fuerzo overflow del timer (TCNT0 = 0xFF) para que se actualicen OCR0A y OCR0B
	CALL	reset_delay_timers			; reset de timers de delay
	CALL	delay						; delay de 0.5 seg
	JMP		go							; vuelta a la rutina de lectura de sensores

go_straight:
	OUT		OCR0A, med_speed_right		; la rueda izq va medio
	OUT		OCR0B, med_speed_left		; la rueda derecha va medio
	OUT		TCNT0, force_ovfl_timers	; fuerzo overflow del timer (TCNT0 = 0xFF) para que se actualicen OCR0A y OCR0B
	CALL	reset_delay_timers			; reset de timers de delay
	CALL	delay						; delay de 0.5 seg
	JMP		go							; vuelta a la rutina de lectura de sensores

stop:
	OUT		OCR0A, no_speed
	OUT		OCR0B, no_speed
	OUT		TCNT0, force_ovfl_timers
	CALL	reset_delay_timers
	CALL	delay
	JMP		go


delay:
	INC 	delay_timer_1
	CPI		delay_timer_1, 100
	BRNE 	delay
	LDI 	delay_timer_1, reset_value
	INC 	delay_timer_2
	CPI		delay_timer_2, 100
	BRNE	delay
	ret

reset_delay_timers:
	LDI		delay_timer_1, reset_value
	LDI		delay_timer_2, reset_value
	ret









;Este programa mezcla ultrasonido con display. Se fija si hay obstaculo, y si hay tira una distancia relativa (entre 0 y 9, cada digito representa una cierta cantidad de cm 
;que puede variar dependiendo de cuantos LSR se haga a R29). Si R29 es mas grande que 10 tira error (no lo puede mostrar) y si no ve obstaculo se apaga
;Usa los registros R27, R2, R28, R29, R26, R19







iniciar_timer_ICR:
sbi TIFR1,5  ;Me aseguro que el flag esté en  0
ldi R28, 0
sts TCNT1H, R28 ;Cargo 0 en el valor del timer por las dudas
STS TCNT1L, R28
LDI R28, 1<<ICIE1
STS TIMSK1, R28 ;Habilito la interrupcion por Capture
LDI R28, (1 << ICNC1) | (1<<CS10) ;Pongo los valores a cargar con noise canceler y por flanco descendente
ret

iniciar_timer_delay: ;El timer del delay es uno con interrupcion por overflow donde modifico el valor inicial del registro TCNT y aplico algun prescaler
	LDI R28, 1<<TOIE0
	STS TIMSK1, R28
	LDI R28, 255
	STS TCNT1H, R28
	LDI R28, 0
	STS TCNT1L, R28
	LDI R28, PRESCALER
	STS TCCR1B,R28  
	ret

INT_TIMER:
    push R28
	IN R28, SREG        
	PUSH R28
	;sbi portb, 2
	;call delay_100ms
	;cbi portb, 2
	;call delay_100ms
	LDS R2, ICR1L            ;Cargo el valor del timer al momento del flanco en R28, tengo que cargar el low porque sino no anda
	LDS R28, ICR1H
	mov R29, R28
	LSR R29
	CPI R29, CONST_DIST 
	BRLO parar
	call display_off  ;Si no hay obstaculo apago el display
	CBI PORTB, 2       ;Acá iría lo que hay que hacer si no hay que parar
	ldi r27, 0

seguir:
	call iniciar_timer_delay  
	POP R28
	OUT SREG, R28
	POP R28
	RETI

parar: 
	call DISPLAY_show
	;Acá iría lo que hay que hacer si se detectó obstáculo, que sería poner los PWM en 0. en este caso, es prender un puerto
	sbi portb, 2
	ldi r27, 1
	JMP seguir

INT_DELAY:
PUSH R28
in R28, sreg
push R28
inc r19  ;Incremento R30 en uno para llegar al valor con el que quiero comparar
LDI R28, 255
STS TCNT1H, R28
LDI R28, CONST_TIMER ;Reseteo el timer en la configuracion que tenia
STS TCNT1L, R28
pop R28
out sreg, R28
POP R28
reti




delay_2us: ;Espera 2us.
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	RET

error:
	ldi R29, 255
	call DISPLAY_show
	ret
    

DISPLAY_SHOW:
	ldi ZH, HIGH(tabla<<1)
	ldi ZL, LOW(tabla<<1)
	cpi r29, 10
	brsh DISPLAY_E
	add ZL, r29
	lpm r16, Z
	rjmp DISPLAY_RET
DISPLAY_E:
	ldi r16, 0x4F
DISPLAY_RET:
	out portc, r16
	ret 

DISPLAY_OFF:
    ldi R16, 0
    out portc, R16
    ret
