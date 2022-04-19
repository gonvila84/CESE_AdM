#include <stdint.h>
/*
1) Realizar una función que inicialice un vector con ceros. La función debe tener el siguiente
prototipo:

void zeros (uint32_t * vector, uint32_t longitud);
*/
void c_zeros (uint32_t * vector, uint32_t longitud)
{
	for (int i = 0; i<longitud;i++)
	{
		vector[i]=0;
	}
}

/*
2) Realizar una función que realice el producto de un vector y un escalar (por ejemplo, podría servir para cambiar el nivel de amplitud de una señal).

void productoEscalar32 (uint32_t * vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar);
*/
void c_productoEscalar32 (uint32_t * vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar)
{
	for (int i=0;i<longitud;i++)
	{
		vectorOut[i] = vectorIn[i] * escalar;
	}
}

/*
3) Adapte la función del ejercicio 2 para realizar operaciones sobre vectores de 16 bits:

void productoEscalar16 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);
*/
void c_productoEscalar16 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar)
{
	for (int i=0;i<longitud;i++)
	{
		vectorOut[i] = vectorIn[i] * escalar;
	}
}

/*
4) Adapte la función del ejercicio 3 para saturar el resultado del producto a 12 bits:

void productoEscalar12 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);
*/
void c_productoEscalar12 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar)
{
	uint16_t tmp = 0;
	for (int i=0;i<longitud;i++)
	{
		tmp = vectorIn[i] * escalar;
		if (tmp > 4095)
		{
			vectorOut[i] = 4095;
		}
		else
		{
			vectorOut[i] = tmp;
		}
	}
}

/*
5) Realice una función que implemente un filtro de ventana móvil de 10 valores sobre un vector de muestras.

void filtroVentana10(uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn);
*/

//NOTA:
//EL CODIGO SE REALIZO DE ACUERDO A LO DEFINIDO EN CLASE UTILIZANDO UNA VENTANA MOVIL DE 3

void c_filtroVentana10(uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn)
{
	uint16_t *p = 0;
	uint32_t samplesNumber = 3;
	uint16_t samplesSum = 0;
	for (int i=0;i<(longitudVectorIn);i++)
	{
		samplesSum = 0;
		p = vectorIn + i;
		for (int j=0; j<samplesNumber; j++)
		{
			if ((p+1) > (vectorIn + longitudVectorIn))
			{
				p = vectorIn;
			}
			samplesSum = samplesSum + (*p);
			p++;
		}
		vectorOut[i] = samplesSum / samplesNumber;
	}
}

/*
6) Realizar una función que reciba un vector de números signados de 32 bits y los “empaquete” en otro vector de 16 bits. La función deberá adecuar los valores de entrada a la nueva precisión.

void pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);
*/

void c_pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud)
{
	for (int i=0; i< longitud; i++)
	{
		vectorOut[i] = vectorIn[i] >> 2;
	}
}

/*
7) Realizar una función que reciba un vector de números signados de 32 bits y devuelva la posición del máximo del vector.

int32_t max (int32_t * vectorIn, uint32_t longitud);
*/
int32_t c_max (int32_t * vectorIn, uint32_t longitud)
{
	int32_t	max = 0;
	int32_t pos = 0;
	for (int i=0;i<longitud;i++)
	{
		if (0 == i)
		{
			max = vectorIn[i];
			pos = i;
		}
		else
		{
			if(max < vectorIn[i])
			{
				max = vectorIn[i];
				pos = i;
			}
		}
	}
	return pos;
}

/*
8) Realizar una función que reciba un vector de muestras signadas de 32 bits y lo decime
descartando una cada N muestras.

void downsampleM (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);
*/

void c_downsampleM (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N)
{
	uint32_t sampleNumber = 0;
	uint32_t lastVectorOutPosition = 0;
	for (int i=0;i<longitud;i++)
	{
		sampleNumber ++;
		if (sampleNumber < N)
		{
			vectorOut[lastVectorOutPosition] = vectorIn[i];
			lastVectorOutPosition++;
		}
		else
		{
			sampleNumber = 0;
		}
	}
}

/*
9) Realizar una función que reciba un vector de muestras no signadas de 16 bits e invierta su orden.

void invertir (uint16_t * vector, uint32_t longitud);
*/
void c_invertir (uint16_t * vector, uint32_t longitud)
{
	uint32_t rightIndex=(longitud - 1);
	uint32_t leftIndex = 0;
	uint16_t placeholder = 0;
	while (leftIndex < rightIndex)
	{
		placeholder = vector[leftIndex];
		vector[leftIndex] = vector[rightIndex];
		vector[rightIndex] = placeholder;

		rightIndex --;
		leftIndex ++;
	}
}
