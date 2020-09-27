/**
 * Copyright (c) 2003-2015 Cirrus Logic, Inc.
 *
 * @file  cl_types.h
 *        
 * @version 01.00.01
 * @author Cirrus Logic, Inc.
 *
 * @bug No known bugs 
 *
 *
 */

#ifndef _CL_TYPES_H
#define _CL_TYPES_H
#include <stdint.h>
#include <stdbool.h>

typedef int32_t clFract;
typedef int32_t clFractNS;
typedef int64_t clAccum;
typedef int32_t clInt;
typedef int32_t clIntNS;
typedef int64_t clLongInt;


#define FRACT_BIT_INQ1_15 (15)
#define CL_NATIVE_INTSIZE 32
#define CL_NATIVE_LONGSIZE 64

#define CL_NATIVE_INT_BITS 32
#define CL_NATIVE_LONGINT_BITS 64
#define CL_NATIVE_FIX_BITS 32
#define CL_NATIVE_ACCUM_BITS 64
#define CL_NATIVE_FIX_INT_BITS 3
#define CL_NATIVE_DB_SHIFT 8
#define CL_NATIVE_DB_SCALE (1.0 * (1 << CL_NATIVE_DB_SHIFT))

#define CL_NATIVE_MAX_INT_POS (CL_NATIVE_INT_BITS-1)
#define CL_NATIVE_MAX_LONGINT_POS (CL_NATIVE_LONGINT_BITS-1)

#define CL_NATIVE_MAX_FIX_POS   (CL_NATIVE_FIX_BITS-1)
#define CL_NATIVE_BP_POS (CL_NATIVE_MAX_FIX_POS-CL_NATIVE_FIX_INT_BITS)  // Position of binary point in native fixed point
#define CL_NATIVE_DB_BP_POS (CL_NATIVE_BP_POS-CL_NATIVE_DB_SHIFT)
#define CL_MAX_INT_CLFRACT ((1<<CL_NATIVE_FIX_INT_BITS) - 1) //maximum integer that can be converted in clFract
#define CL_MIN_INT_CLFRACT (-(1<<CL_NATIVE_FIX_INT_BITS)) //minimum integer that can be converted in clFract

#define CL_NATIVE_MAX_ACCUM_POS  (CL_NATIVE_ACCUM_BITS-1)
#define CL_NATIVE_ACCUM_BP_POS	(CL_NATIVE_BP_POS*2)
// This is derived from the definition of the accumulator binary point
#define CL_NATIVE_ACCUM_INT_BITS (CL_NATIVE_ACCUM_BITS - (1 + CL_NATIVE_ACCUM_BP_POS))
#define CL_MAX_INT_CLACCUM ((1<<(CL_NATIVE_FIX_INT_BITS*2)) - 1) //maximum integer that can be converted in clAccum
#define CL_MIN_INT_CLACCUM (-(1<<(CL_NATIVE_FIX_INT_BITS*2))) //minimum integer that can be converted in clAccum

#define CL_DBFRACTION_MASK ((1 << (CL_NATIVE_DB_BP_POS))-1)
#define CL_NATIVE_FIX_MASK ((1 << (CL_NATIVE_BP_POS))-1)
#define CL_NATIVE_INT_MASK (~(CL_NATIVE_FIX_MASK))
#define CL_NATIVE_ACCUM_LOWORD_MASK ((((int64_t)1)<<CL_NATIVE_FIX_BITS)-1)
#define CL_NATIVE_ACCUM_HIWORD_MASK (-((((int64_t)1)<<CL_NATIVE_FIX_BITS)-1))

#define CL_MASK_FRACT_UNSIGNED (~(1<<(CL_NATIVE_FIX_BITS-1)))

#define clDblFractMul (double)(1 << CL_NATIVE_BP_POS)
#define clDblAccumMul (double)(((int64_t)1) << CL_NATIVE_ACCUM_BP_POS)

#define clFRACTMAX_INT ((int32_t)0x7fffffff)      // (7.9999999962747097015380859375)
#define clFRACTMIN_INT ((int32_t)0x80000000)          // (-8.0)

#define clACCUMMAX_LL ((int64_t)0x7fffffffffffffff) // (127.999999940395355224609375)
#define clACCUMMIN_LL ((int64_t)0x8000000000000000) // (-128.0)

#define clFRACTMAX (clFract)clFRACTMAX_INT
#define clFRACTMIN (clFract)clFRACTMIN_INT
#define clACCUMMAX (clAccum)clACCUMMAX_LL
#define clACCUMMIN (clAccum)clACCUMMIN_LL     


#define clFRACTMAX_DOUBLE (((double)((int32_t)clFRACTMAX_INT)) / clDblFractMul)
#define clFRACTMIN_DOUBLE (((double)((int32_t)clFRACTMIN_INT)) / clDblFractMul)

#define clACCUMMAX_DOUBLE (((double)((int64_t)clACCUMMAX_LL)) / clDblAccumMul)
#define clACCUMMIN_DOUBLE (((double)((int64_t)clACCUMMIN_LL)) / clDblAccumMul)


// Maximum and minimum fract values expressed in terms of clAccum
#define CL_ACCUM_MAX_FRACT (clAccum)((int64_t)0x000000007fffffff)
#define CL_ACCUM_MIN_FRACT (clAccum)((int64_t)0xffffffff80000000)


#define clMin(x, y) (((x) < (y)) ? (x) : (y))
#define clMax(x, y) (((x) > (y)) ? (x) : (y))


// Saturation behaviour when converting from clAccum to clFract
// inline clAccum clAccumSaturateToFract(clAccum x) { return clMax(CL_ACCUM_MIN_FRACT, clMin(CL_ACCUM_MAX_FRACT, x)); }


// The number of bits to right-shift an clAccum to convert to clFract
#define CL_ACCUM_TO_FRACT_RSHIFT (CL_NATIVE_ACCUM_BP_POS - CL_NATIVE_BP_POS)
// The number of bits to left-shift the top word of a clAccum to convert to clFract
#define CL_ACCUM_HI_TO_FRACT_LSHIFT (CL_NATIVE_ACCUM_INT_BITS - CL_NATIVE_FIX_INT_BITS)
#define CL_NATIVE_FRACT_HALF (1<<(CL_NATIVE_BP_POS-1))

#endif // _CL_TYPES_H

