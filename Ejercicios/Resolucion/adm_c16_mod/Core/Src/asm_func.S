.syntax unified
.thumb
.text

@ Simbolos exportados
.global asm_svc
.global asm_sum
.global asm_zeros
.global asm_productoEscalar32
.global asm_productoEscalar16
.global asm_productoEscalar12
.global asm_max
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_svc (void)
@
.thumb_func
    asm_svc:
        svc 0
        bx lr


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   uint32_t asmSum (uint32_t firstOperand, uint32_t secondOperand)
@
@ Parametros recibidos:
@   r0: firstOperand
@   r1: secondOperand
@
@ Valor de retorno:
@   r0: resultado de la suma de firstOperand y secondOperand
@
.thumb_func
    asm_sum:
        add r0, r1  @ r0 = r0 + r1
        bx lr       @ vuelve adonde fue llamada (especificamente, si "pc"
                    @ es la direccion, en el codigo del programa, de la
                    @ instruccion "bl asm_sum" usada para saltar a
                    @ esta funcion, entonces "lr" contiene ese "pc" + 4).
                    @ el valor de retorno debe quedar en el registro r0.
                    @ En este caso, r0 es el resultado de r0 + r1.

        @ Otras alternativas para el retorno
        @ mov pc,lr
        @ bx lr
        @ push {lr} ... pop {pc}



@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_zeros (uint32_t * vector, uint32_t longitud)
@
@ Parametros recibidos:
@   r0: puntero al vector
@   r1: longitud del vector
@
@
.thumb_func
    asm_zeros:
        mov		r2, 0
    .asm_zeros_for:
        str		r2, [r0], 4
        subs	r1, 1
        bne		.asm_zeros_for
        bx lr
                    @ es la direccion, en el codigo del programa, de la
                    @ instruccion "bl asm_sum" usada para saltar a
                    @ esta funcion, entonces "lr" contiene ese "pc" + 4).
                    @ el valor de retorno debe quedar en el registro r0.
                    @ En este caso, r0 es el resultado de r0 + r1.

        @ Otras alternativas para el retorno
        @ mov pc,lr
        @ bx lr
        @ push {lr} ... pop {pc}


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_productoEscalar32 (uint32_t * vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar)
@
@ Parametros recibidos:
@   r0: puntero al vector de entrada
@   r1: puntero al vector de salida
@	r2: longitud
@	r3: escalar


.thumb_func
    asm_productoEscalar32:
    	ldr		r4, [r0]
    .asm_productoEscalar32_for:
		mul		r4, r4, r3
		str		r4,	[r1], 4
		ldr		r4, [r0,4]!
        subs	r2, 1
        bne		.asm_productoEscalar32_for
        bx lr

.thumb_func
    asm_productoEscalar16:
    	ldrh	r4, [r0]
    .asm_productoEscalar16_for:
		mul		r4, r4, r3
		strh	r4,	[r1], 2
		ldrh	r4, [r0,2]!
        subs	r2, 1
        bne		.asm_productoEscalar16_for
        bx lr

.thumb_func
    asm_productoEscalar12:
    	push 	{r4}
    	ldrh	r4, [r0]
    .asm_productoEscalar12_for:
		mul		r4, r4, r3
		mov		r5, #0x0FFF
		and 	r4, r5
		strh	r4,	[r1], 2
		ldrh	r4, [r0,2]!
        subs	r2, 1
        bne		.asm_productoEscalar12_for
        pop		{r4}
        bx lr

@ 		5) Realice una función que implemente un filtro de ventana móvil de 10 valores sobre un vector de muestras.
@
@		void filtroVentana10(uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn);

@		6) Realizar una función que reciba un vector de números signados de 32 bits y los “empaquete” en
@		otro vector de 16 bits. La función deberá adecuar los valores de entrada a la nueva precisión.
@
@		void pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);





@ 7) Realizar una función que reciba un vector de números signados de 32 bits y devuelva la posición del máximo del vector.
@
@ Prototipo en "C":
@		int32_t max (int32_t * vectorIn, uint32_t longitud);
@
@ Parametros recibidos:
@   r0: puntero al vector de entrada
@   r1: longitud

.thumb_func
	asm_max:
		push	{r4}
		mov		r4, r1
	.asm_maximoDelVector_loop:
		ldr 	r2,[r0],#4

		cmp		r4, r1
		IT		EQ
		moveq	r3, r2

		cmp		r3, r2
		IT		LT
		movlt	r3, r2

		subs	r1,1						@	Descuento al valor de la longitud para decrementar el loop y actualizo flags
		bne		.asm_maximoDelVector_loop	@	Si la resta anterior no da 0 me dirjo a una nueva iteracion
		pop		{r4}
		push	{r0}
		mov		r0, r3
		bx 		lr							@	Salgo de la funcion enviando el valor de retorno
