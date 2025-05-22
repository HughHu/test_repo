/*
 * mc_mutex.c
 *
 *  Created on: Mar. 19, 2025
 *
 */

#include "mc_mutex.h"

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
int32_t MCMutex_Lock(volatile uint32_t *mc_mutex)
{
    assert(mc_mutex != NULL);
    uint32_t oldval;
    do {
        oldval = __AMOSWAP_W(mc_mutex, MCMTX_LOCKED_VAL);
    } while (oldval == MCMTX_LOCKED_VAL);
    return 0;
}


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
int32_t MCMutex_TryLock(volatile uint32_t *mc_mutex, uint32_t try_times)
{
    assert(mc_mutex != NULL);
    uint32_t oldval, i = 0;

    if (try_times == 0)
        return -1;
    do {
        oldval = __AMOSWAP_W(mc_mutex, MCMTX_LOCKED_VAL);
    } while (oldval == MCMTX_LOCKED_VAL && i++ < try_times);

    return (i < try_times ? 0 : -1);
}


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
int32_t MCMutex_Unlock(volatile uint32_t *mc_mutex)
{
    assert(mc_mutex != NULL);
    uint32_t oldval = __AMOSWAP_W(mc_mutex, MCMTX_UNLOCKED_VAL);

    //TODO: return error if not locked before or always return 0?
    return (oldval == MCMTX_LOCKED_VAL);
    //return 0;
}


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
//static inline int32_t MCMutex_IsLocked(const volatile uint32_t *mc_mutex);


//-----------------------------------------------------------------------------
///
///  Initialize a mc_mutex object before first use.
///
///  \param     mc_mutex           Pointer to mc_mutex object.
///
///  \return    Returns 0 on success, else a negative value.
///
//-----------------------------------------------------------------------------
int32_t MCMutex_Init(volatile uint32_t *mc_mutex)
{
    assert(mc_mutex != NULL);
    if (mc_mutex == NULL)
        return -1;
    *mc_mutex = MCMTX_UNLOCKED_VAL;
    return 0;
}


