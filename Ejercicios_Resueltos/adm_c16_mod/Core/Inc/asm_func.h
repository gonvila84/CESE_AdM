#pragma once

#include <stdint.h>


void asm_svc (void);

/*
1) Realizar una función que inicialice un vector con ceros. La función debe tener el siguiente
prototipo:

void zeros (uint32_t * vector, uint32_t longitud);
*/
void asm_zeros 	(uint32_t * vector, uint32_t longitud);

/*
2) Realizar una función que realice el producto de un vector y un escalar (por ejemplo, podría servir para cambiar el nivel de amplitud de una señal).

void productoEscalar32 (uint32_t * vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar);
*/
void asm_productoEscalar32 (uint32_t * vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar);

/*
3) Adapte la función del ejercicio 2 para realizar operaciones sobre vectores de 16 bits:

void productoEscalar16 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);
*/
void asm_productoEscalar16 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);

/*
4) Adapte la función del ejercicio 3 para saturar el resultado del producto a 12 bits:

void productoEscalar12 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);
*/
void asm_productoEscalar12 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);

/*
5) Realice una función que implemente un filtro de ventana móvil de 10 valores sobre un vector de muestras.

void filtroVentana10(uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn);
*/
void asm_filtroVentana10(uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn);

/*
6) Realizar una función que reciba un vector de números signados de 32 bits y los “empaquete” en otro vector de 16 bits. La función deberá adecuar los valores de entrada a la nueva precisión.

void pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);
*/
void asm_pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);

/*
7) Realizar una función que reciba un vector de números signados de 32 bits y devuelva la posición del máximo del vector.

int32_t max (int32_t * vectorIn, uint32_t longitud);
*/
int32_t asm_max (int32_t * vectorIn, uint32_t longitud);

/*
8) Realizar una función que reciba un vector de muestras signadas de 32 bits y lo decime
descartando una cada N muestras.

void downsampleM (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);
*/
void asm_downsampleM (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);

/*
9) Realizar una función que reciba un vector de muestras no signadas de 16 bits e invierta su orden.

void invertir (uint16_t * vector, uint32_t longitud);
*/
void asm_invertir (uint16_t * vector, uint32_t longitud);













/*
uint32_t asm_sum (uint32_t firstOperand, uint32_t secondOperand);




void 	asm_productoEscalar12 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);
int32_t asm_max (int32_t * vectorIn, uint32_t longitud);
void	asm_invertir (uint16_t * vector, uint32_t longitud);
void 	asm_downsampleM (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);
void 	asm_filtroVentana10(uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn);
void 	asm_pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);
*/
