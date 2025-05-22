/*
 * mc_mutex.h
 *
 *  Created on: Mar. 19, 2025
 *
 */

#ifndef CHIP_BSP_MC_MUTEX_H_
#define CHIP_BSP_MC_MUTEX_H_

#include <stdint.h>
#include <stdlib.h>
#include "nmsis_gcc.h"
#include "core_feature_base.h" // NMSIS header for basic RISC-V instructions
#include "riscv_encoding.h" // NMSIS header for CSR registers
#include "dbg_assert.h"


// The multi-core mutex is used for exclusive access to a shared resource on multi-core platform.
// mc_mutex is actually a WORD data located on non-cacheable & non-local RAM (out of any MCU core),
// and exclusive access is implemented based on Atom Instructions supported on specific CPU ARCH.
#define MCMTX_LOCKED_VAL        0x4D434D01 // 'MCM'_0x01
#define MCMTX_UNLOCKED_VAL      0x4D430000 // 'MC'_0x0000

//-----------------------------------------------------------------------------
///
///  Take ownership of the mc_mutex: block until the mc_mutex is owned.
///  The mc_mutex must have been initialized.
///
///  \param     mc_mutex           Pointer to mc_mutex object
///
///  \return    Returns 0 on success, else a negative value.
///
//-----------------------------------------------------------------------------
int32_t MCMutex_Lock(volatile uint32_t *mc_mutex);


//-----------------------------------------------------------------------------
///
///  Try to take ownership of the mc_mutex for several times, but do not block
///  if the mc_mutex is taken. Return immediately.
///  The mc_mutex must have been initialized.
///
///  \param     mc_mutex           Pointer to mc_mutex object.
///  \param     try_times          number of attempts.
///
///  \return    Returns 0 on success (mc_mutex owned), else a negative value.
///
//-----------------------------------------------------------------------------
int32_t MCMutex_TryLock(volatile uint32_t *mc_mutex, uint32_t try_times);


//-----------------------------------------------------------------------------
///
///  Release ownership of the mc_mutex. The mc_mutex must have been
///  initialized and must be owned by the calling core.
///
///  \param     mc_mutex           Pointer to mc_mutex object.
///
///  \return    Returns 0 on success, else a negative value.
///
//-----------------------------------------------------------------------------
int32_t MCMutex_Unlock(volatile uint32_t *mc_mutex);


//-----------------------------------------------------------------------------
///
///  Return the state of the mc_mutex (locked or unlocked) but do not attempt to
///  take ownership. The mutex must have been initialized.
///
///  \param     mc_mutex           Pointer to mc_mutex object.
///
///  \return    Returns 0 if unlocked, 1 if locked, -1 on error.
///
//-----------------------------------------------------------------------------
static inline int32_t MCMutex_IsLocked(const volatile uint32_t *mc_mutex)
{
    assert(mc_mutex != NULL);
    if (mc_mutex != NULL) {
        return (*mc_mutex != MCMTX_LOCKED_VAL) ? 1 : 0;
    }
    return -1;
}


//-----------------------------------------------------------------------------
///
///  Initialize a mc_mutex object before first use.
///
///  \param     mc_mutex           Pointer to mc_mutex object.
///
///  \return    Returns 0 on success, else a negative value.
///
//-----------------------------------------------------------------------------
int32_t MCMutex_Init(volatile uint32_t *mc_mutex);


//-----------------------------------------------------------------------------
// define aliases for RV32 atomic functions
//-----------------------------------------------------------------------------
#define ATM_CmpSwp      __CAS_W
#define ATM_Swp         __AMOSWAP_W
#define ATM_Add         __AMOADD_W
#define ATM_And         __AMOAND_W
#define ATM_Or          __AMOOR_W
#define ATM_Xor         __AMOXOR_W
#define ATM_MaxU        __AMOMAXU_W
#define ATM_Max         __AMOMAX_W
#define ATM_MinU        __AMOMINU_W
#define ATM_Min         __AMOMIN_W

// Atomic operations extracted from core_feature_base.h
//__STATIC_FORCEINLINE uint32_t __CAS_W(volatile uint32_t *addr, uint32_t oldval, uint32_t newval);
//__STATIC_FORCEINLINE uint32_t __AMOSWAP_W(volatile uint32_t *addr, uint32_t newval);
//__STATIC_FORCEINLINE int32_t __AMOADD_W(volatile int32_t *addr, int32_t value);
//__STATIC_FORCEINLINE int32_t __AMOAND_W(volatile int32_t *addr, int32_t value);
//__STATIC_FORCEINLINE int32_t __AMOOR_W(volatile int32_t *addr, int32_t value);
//__STATIC_FORCEINLINE int32_t __AMOXOR_W(volatile int32_t *addr, int32_t value);
//__STATIC_FORCEINLINE uint32_t __AMOMAXU_W(volatile uint32_t *addr, uint32_t value);
//__STATIC_FORCEINLINE int32_t __AMOMAX_W(volatile int32_t *addr, int32_t value)
//__STATIC_FORCEINLINE uint32_t __AMOMINU_W(volatile uint32_t *addr, uint32_t value)
//__STATIC_FORCEINLINE int32_t __AMOMIN_W(volatile int32_t *addr, int32_t value)


//-----------------------------------------------------------------------------
// Check current core/hart
//-----------------------------------------------------------------------------
__STATIC_FORCEINLINE uint32_t HAL_GetCoreID()
{ return __RV_CSR_READ(CSR_MHARTID); }

//-----------------------------------------------------------------------------
// Check current execution context
//-----------------------------------------------------------------------------
typedef enum {
  EXCTX_NORMAL = 0, // Normal Machine Mode
  EXCTX_INTERRUPT = 0x1, // Interrupt Handling
  EXCTX_EXECEPTION = 0x2, // Exception Handling
  EXCTX_NMI = 0x3, // NMI Handling
  EXCTX_GENERAL_ISR = 0x4 // generalized Interrupt context (including Interrupt, Exception, NMI etc.)
} emEXCTX;

__STATIC_FORCEINLINE emEXCTX HAL_GetRunningCTX()
{
    CSR_MSUBM_Type msubm_val = (CSR_MSUBM_Type)__RV_CSR_READ(CSR_MSUBM);
    return (emEXCTX)msubm_val.b.typ;
}

//-----------------------------------------------------------------------------
// Check current IRQ whose handler is running
// Return IRQ# if Interrupt context, Exception Code if Exception context, and
//  -1 in other cases.
//-----------------------------------------------------------------------------
__STATIC_FORCEINLINE int32_t HAL_GetRunningIRQ()
{
    CSR_MSUBM_Type msubm_val = (CSR_MSUBM_Type)__RV_CSR_READ(CSR_MSUBM);
    if (msubm_val.b.typ != EXCTX_INTERRUPT && msubm_val.b.typ != EXCTX_EXECEPTION)
        return -1;
    CSR_MCAUSE_Type mcause_val = (CSR_MCAUSE_Type)__RV_CSR_READ(CSR_MCAUSE);
    return mcause_val.b.exccode;
}


#endif /* CHIP_BSP_MC_MUTEX_H_ */
