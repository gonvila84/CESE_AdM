#pragma once

#include <stdint.h>

void asm_svc (void);

void asm_zeros (uint32_t * vector, uint32_t longitud);
void asm_productoEscalar32 (uint32_t * vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar);
