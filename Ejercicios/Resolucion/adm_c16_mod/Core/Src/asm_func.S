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
.global asm_pack32to16
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


/*
1) Realizar una función que inicialice un vector con ceros. La función debe tener el siguiente prototipo:

void zeros (uint32_t * vector, uint32_t longitud);
*/

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



/*
2) Realizar una función que realice el producto de un vector y un escalar (por ejemplo, podría servir para cambiar el nivel de amplitud de una señal).

void productoEscalar32 (uint32_t * vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar);
*/

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


/*
3) Adapte la función del ejercicio 2 para realizar operaciones sobre vectores de 16 bits:

void productoEscalar16 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);
*/

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

/*
4) Adapte la función del ejercicio 3 para saturar el resultado del producto a 12 bits:

void productoEscalar12 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);
*/

.thumb_func
    asm_productoEscalar12:
    push	{r4}
         .asm_productoEscalar12_loop:
    		ldrh	r4,[r0],2
    		mul		r4,r4,r3
    		usat 	r4,12,r4
    		strh	r4,[r1],2
    		subs	r2,1
    	bne		.asm_productoEscalar12_loop
    pop		{r4}
	bx lr

/*
5) Realice una función que implemente un filtro de ventana móvil de 10 valores sobre un vector de muestras.

void filtroVentana10(uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn);
*/

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

/*
7) Realizar una función que reciba un vector de números signados de 32 bits y devuelva la posición del máximo del vector.

int32_t max (int32_t * vectorIn, uint32_t longitud);
*/

.thumb_func
	asm_max:
	push	{r4,r5}
	mov r2,0										//Inicio el contador de iteraciones en 0
	mov r4,0										//Guardo el maximo valor (inicio en 0)
	mov r5,0										//Guardo la posicion del maximo valor  (inicio en 0)
	.asm_max_loop:
	ldr		r3,[r0],4								//Cargo el valor V[i]
	cmp		r2,0									//Verifico si estoy en la iteracion 0 (El bloque solo se ejecuta en la primer iteracion)
	ITT		EQ
	moveq	r4,r3									//En la iteracion 0 cargo el valor V[i]
	moveq	r5,r2									//En la iteracion 0 cargo numero de iteracion actual

	cmp		r4,r3									//Comparo el maximo registrado con el valor de V[i]
	ITT		LT										//Verifico si el maximo es < V[i]
	movlt	r4,r3									//Si la condicion se cumple cargo el nuevo maximo
	movlt	r5,r2									//Si la condicion se cumple la posicion del nuevo maximo

	add		r2,1									//Incremento el contador de iteraciones en 1
	cmp		r2,r1									//Me fijo si se alcanzo el maximo de iteraciones comparando con el valor de (Longitud-1)
	blt		.asm_max_loop							//Si el maximo de iteraciones no se alcanza se vuelve a iterar, sin se sale del loop
	mov		r0,r5									//Cargo la posicion del valor maximo en R0 y devuelvo a la rutina principal
	pop	{r4,r5}
	bx	lr

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



.thumb_func
	sub		r2,1
	asm_pack32to16:
		.asm_pack32to16_loop:
		ldr		r3,[r0],4
		asr		r3,2
		strh	r3,[r1],2
		subs 	r2,1
		bne		.asm_pack32to16_loop
	bx		lr


