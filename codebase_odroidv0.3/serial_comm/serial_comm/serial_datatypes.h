#ifndef SERIAL_DATATYPES_H
#define SERIAL_DATATYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

  // functions to convert (interpret bytes) between floats and various other datatypes
  void interpret_uint64_as_2float(uint64_t k, float *a1, float *a2);
  uint64_t interpret_2float_as_uint64(float a1, float a2);
  float interpret_uint32_as_float(uint32_t a);
  uint32_t interpret_float_as_uint32(float a);
  float interpret_int32_as_float(int a);
  int interpret_float_as_int32(float a);
  float interpret_4uint8_as_float(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4);
  void interpret_float_as_4uint8(float f, uint8_t *a1, uint8_t *a2, uint8_t *a3, uint8_t *a4);
  float interpret_2uint16_as_float(uint16_t a1, uint16_t a2);
  void interpret_float_as_2uint16(float f, uint16_t *a1, uint16_t *a2);

#ifdef __cplusplus
}
#endif

#endif
