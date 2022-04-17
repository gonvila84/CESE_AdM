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
.global asm_downsampleM
.global asm_invertir
.global	asm_rotatetest
.global asm_filtroVentana10
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
@		int32_t asm_max (int32_t * vectorIn, uint32_t longitud);
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

@ 8) Realizar una función que reciba un vector de muestras signadas de 32 bits y lo decime descartando una cada N muestras.
@
@ Prototipo en "C":
@		void asm_downsampleM (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);
@
@ Parametros recibidos:
@   r0: puntero al vector de entrada
@	r1:	puntero al vector de salida
@   r2: longitud
@	r3:	numero de muestras que se debe descartar

@ Mis registros:
@ r4: Contador de muestras
@ r5: Almacenamiento del valor actual de r0
@ r6: Almacenamiento del valor actual de r1
@ r7: Copia del vector de salida
.thumb_func
	asm_downsampleM:
		push	{r4,r5,r6,r7}
		mov		r7,#0
		mov		r4,#1						@ Inicio un contador de muestras

	.asm_downsampleM_loop:
		subs	r2,1						@ Decremento el contador de iteraciones
		blt		.asm_downsample_end

		ldr 	r5,[r0],#4					@ Cargo el valor del vector de entrada y me dezplazo

		cmp		r4,r3						@ Resto al valor del contador la cantidad provista como parametro (n de cada M muestras a descartar)
		ITT		EQ
		moveq	r4,#1						@ Reseteo el contador de muestras
		streq	r7,	[r1]
		beq		.asm_downsampleM_loop		@ Salto condicional al comienzo del loop porque llegue a las muestras a descartar definidas

		add		r4,1						@ Incremento el contador de muestras
		str		r5,	[r1],4					@ Actualizo el valor en el vector de salida
		b		.asm_downsampleM_loop		@ Salto condicional al comienzo del loop si no llegue a recorrer todas las muestras
	.asm_downsample_end:
		pop		{r4,r5,r6,r7}
		bx		lr

@ 9) Realizar una función que reciba un vector de muestras no signadas de 16 bits e invierta su orden.
@
@ Prototipo en "C":
@		void invertir (uint16_t * vector, uint32_t longitud);
@
@ Parametros recibidos:
@   r0: puntero al vector
@   r1: longitud

@ Mis registros:
@	r2: Registro para almacenar la posicion Vi[i]
@	r3:	Registro para almacenar la posision Vi[longitud]
@	r4:	Registro para almacenar el valor de i
@	r5:

@.thumb_func
@	asm_invertir:
@		mov		r4,0
@	.asm_invertir_loop
@		ldrh 	r2,[r0],#2

@		sub		r3,1	@decremento longitud
@		add		r4,1	@incremento i
@		cmp		r4,r3	@si i>=j salgo del loop
@		blt		.asm_invertir_loop
@		bx		lr


@void	asm_rotatetest (uint16_t * asmInvertirArrayIn, uint32_t longitud);

//R6 = i
//R4 = N
//R5 = Direccion base de Vin
.thumb_func
	asm_invertir:
		push	{r4,r5,r6,r7}
		mov 	r2,r0	//Me guardo la direccion base
		mov		r7,#2
		sub 	r1,1	//Decremento la longitud para hacerla coincidir con V[i] = Long-1 como ultima posicion
		mul 	r1,r7	//Multiplico por 4 bytes, eso me da la distancia en bytes con r0
		add 	r3,r1	//Almaceno la ultima dir de memoria V
		mov 	r1,0	//Reutilizo r1 para cumplir la funcion de variable i del for
		.asm_invertir_loop:
			subs	r6, r3, r2
			beq		.asm_invertir_loop_end
			cmp		r6, 2
			beq		.asm_invertir_loop_end
			ldrh	r4, [r2]
			ldrh	r5, [r3]
			strh	r4, [r3]
			strh	r5,	[r2]
			add		r1, 1
			add		r2, 2
			sub		r3, 2
			b		.asm_invertir_loop
			.asm_invertir_loop_end:
			pop			{r4,r5,r6,r7}
			bx			lr


//void 	asm_filtroVentana10(uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn);
// R0, R1 Y R2 ESTAN TOMADOS
//r3
//r4
//r5
//r6

//.thumb_func
//	asm_filtroVentana10:
//		push	{r4,r5,r6,r7,r8}
//		mov		r8, 0
//		sub		r2, 1

//		mov		r3, r0				//Copio la direccion base del V a R3
//		.asm_f10_loop:
//			ldrh 	r4, [r3]			//Cargo el valor al que apunta R3 a R4
/*
			mov		r7, r3
			.asm_fv10_averageLoop:
				add		r3, 2
				ldrh	r5,[r3]
				add		r4,r5
				subs 	r6,r3,r7
				cmp		r6,4
			blt		.asm_fv10_averageLoop
			sub		r3, 2
			mov		r6,#3
			udiv	r6,r4,r6
			strh	r6,[r1],2
			add		r8,1
			cmp		r8,r2
		blt		.asm_f10_loop
		pop		{r4,r5,r6,r7,r8}
	bx	lr
*/


//R0 - Vin
//R1 - Vout
//R2 - longitud

.thumb_func
	asm_filtroVentana10:
	push	{r4,r5,r6,r7,r8}
	sub		r2, 1
	mov		r3, r0
	mov		r4, 3
	.asm_f10_loop:
	mov		r6, r3
	mov		r8, 0
		.asm_f10_avg_loop:
			mov		r5, 2
			mul		r5, r2,	r5
			add		r5, r0, r5

			ldrh	r7,[r3]
			add		r8,r7

			subs	r5, r3	//Se fija si llego a la ultima direccion
			bne	.withoutReturn

			cmp		r4, 1	//Se fija si esta recolectando la ultima muestra
			IT		NE
			movne	r3, r0
			b		.toSamplesDecrement

			.withoutReturn:

			cmp		r4, 1	//Se fija si esta recolectando la ultima muestra
			IT		NE
			addne	r3, 2

			.toSamplesDecrement:
			subs	r4, 1
		bne		.asm_f10_avg_loop
	mov		r4, 3
	udiv	r8,r8,r4
	strh	r8,[r1],2
	mov		r3, r6
	mov		r5, 2
	mul		r5, r2,	r5
	add		r5, r0, r5
	subs	r5,r3
	IT		GT
	addgt	R3, 2
bne		.asm_f10_loop
pop		{r4,r5,r6,r7,r8}
bx		lr
		//si no termino de recolectar salta a r0




/*
	push	{r4,r5,r6,r7,r8}
	sub		r2,1
	mul		r5,r2,2
	add		r3,r0,r5
	mov		r4,r0
	mov		r5,3			//Poner aqui las muestras adecuadas
	mov		r6,0
	.asm_f10_loop:

		mov		r7,2			//Lo reinicion en 2 en cada iteracion asi lo puedo reutilizar
		mul		r7,r6
		add		r4,r0,r7

			mov	r7,1
			.asm_f10_avg_loop:
				ldrh	r8,[r4]
				push	{r7}
				add 	r7,r8
				mov		r8,r7
				pop		{r7}
				add r7,1
				cmp r7,r5
			blt	.asm_f10_avg_loop:




	add		r6,1
	cmp		r6,r2
	blt		.asm_f10_loop

	pop		{r4,r5,r6,r7,r8}
	bx		lr
*/
