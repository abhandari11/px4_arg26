/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file atomic.h
 *
 * Provides atomic integers and counters, modelled on std::atomic. Each method is
 * executed atomically (no torn or lost values) and can be given a memory ordering
 * argument, just like the C++ standard library.
 *
 * The implementation uses the built-in methods from GCC (supported by Clang as well).
 * @see https://gcc.gnu.org/onlinedocs/gcc/_005f_005fatomic-Builtins.html.
 *
 * ## Two separate guarantees
 *
 * A memory ordering provides two distinct things, which are easy to conflate:
 *
 *  1. *Atomicity* — the value never tears and updates are never lost. For an aligned
 *     word (<= 32 bit) a plain load/store is already atomic on Cortex-M; a
 *     read-modify-write needs LDREX/STREX; a 64-bit type on a 32-bit core needs a
 *     critical section.
 *  2. *Ordering* — neighbouring memory accesses are not reordered past the atomic so
 *     another observer sees them in the intended order. This has a *compiler* part
 *     (fixed by a compiler fence, zero instructions) and a *hardware* part across
 *     observers such as a second CPU core or DMA (fixed by a `dmb`).
 *
 * ## Single-core targets
 *
 * On a single-core (uniprocessor) target there is no second core, so for
 * inter-thread synchronisation the hardware `dmb` that acquire/release/seq_cst would
 * emit is dead weight — only the compiler ordering is actually required. This mirrors
 * Linux, where smp_mb()/smp_rmb()/smp_wmb() collapse to a plain compiler barrier on
 * uniprocessor builds. We therefore lower the ordering to a compiler-only fence
 * (__atomic_signal_fence) when building NuttX without CONFIG_SMP.
 *
 * @note This class provides *inter-thread* ordering only. Synchronisation with DMA or
 * device memory needs explicit barriers (e.g. up_clean_dcache / __DMB) and must not
 * rely on the ordering here.
 *
 * @note: on ARM, the instructions LDREX and STREX might be emitted. To ensure correct
 * behavior, the exclusive monitor needs to be cleared on a task switch (via CLREX).
 * This happens automatically e.g. on ARMv7-M as part of an exception entry or exit
 * sequence.
 */

#pragma once

#ifdef __cplusplus

#include <stdbool.h>
#include <stdint.h>

#if defined(__PX4_NUTTX)
# include <nuttx/config.h>
# include <nuttx/irq.h>
#endif // __PX4_NUTTX

// On a single-core NuttX build the hardware memory barriers are unnecessary for
// inter-thread ordering; a compiler-only fence is sufficient and saves the `dmb`.
#if defined(__PX4_NUTTX) && !defined(CONFIG_SMP)
# define PX4_ATOMIC_SINGLE_CORE 1
#else
# define PX4_ATOMIC_SINGLE_CORE 0
#endif

namespace px4
{

// Mirrors std::memory_order; values map directly onto the GCC __ATOMIC_* builtins.
// Use the scoped form at call sites, e.g. px4::memory_order::acquire.
enum class memory_order : int {
	relaxed = __ATOMIC_RELAXED,
	consume = __ATOMIC_CONSUME,
	acquire = __ATOMIC_ACQUIRE,
	release = __ATOMIC_RELEASE,
	acq_rel = __ATOMIC_ACQ_REL,
	seq_cst = __ATOMIC_SEQ_CST,
};

template <typename T>
class atomic
{
public:

#if defined(__PX4_POSIX)
	// Ensure that all operations are lock-free, so that 'atomic' can be used from
	// IRQ handlers. This might not be required everywhere though.
	static_assert(__atomic_always_lock_free(sizeof(T), 0), "atomic is not lock-free for the given type T");
#endif // __PX4_POSIX

	atomic() = default;
	explicit atomic(T value) : _value(value) {}

	/**
	 * Atomically read the current value
	 */
	inline T load(memory_order order = memory_order::seq_cst) const
	{
#if defined(__PX4_NUTTX)

		if (!__atomic_always_lock_free(sizeof(T), 0)) {
			// Not lock-free (e.g. 64-bit on a 32-bit core): a critical section
			// provides both atomicity and full ordering.
			irqstate_t flags = enter_critical_section();
			T val = _value;
			leave_critical_section(flags);
			return val;
		}

#endif // __PX4_NUTTX

#if PX4_ATOMIC_SINGLE_CORE
		// Relaxed load (a plain, non-tearing ldr) plus a compiler-only fence: no `dmb`.
		T val;
		__atomic_load(&_value, &val, __ATOMIC_RELAXED);
		__atomic_signal_fence(static_cast<int>(order));
		return val;
#else
		T val;
		__atomic_load(&_value, &val, static_cast<int>(order));
		return val;
#endif
	}

	/**
	 * Atomically store a value
	 */
	inline void store(T value, memory_order order = memory_order::seq_cst)
	{
#if defined(__PX4_NUTTX)

		if (!__atomic_always_lock_free(sizeof(T), 0)) {
			irqstate_t flags = enter_critical_section();
			_value = value;
			leave_critical_section(flags);
			return;
		}

#endif // __PX4_NUTTX

#if PX4_ATOMIC_SINGLE_CORE
		__atomic_signal_fence(static_cast<int>(order));
		__atomic_store(&_value, &value, __ATOMIC_RELAXED);
#else
		__atomic_store(&_value, &value, static_cast<int>(order));
#endif
	}

	/**
	 * Atomically add a number and return the previous value.
	 * @return value prior to the addition
	 */
	inline T fetch_add(T num, memory_order order = memory_order::seq_cst)
	{
#if defined(__PX4_NUTTX)

		if (!__atomic_always_lock_free(sizeof(T), 0)) {
			irqstate_t flags = enter_critical_section();
			T ret = _value;
			_value += num;
			leave_critical_section(flags);
			return ret;
		}

#endif // __PX4_NUTTX

#if PX4_ATOMIC_SINGLE_CORE
		__atomic_signal_fence(static_cast<int>(order));
		T ret = __atomic_fetch_add(&_value, num, __ATOMIC_RELAXED);
		__atomic_signal_fence(static_cast<int>(order));
		return ret;
#else
		return __atomic_fetch_add(&_value, num, static_cast<int>(order));
#endif
	}

	/**
	 * Atomically substract a number and return the previous value.
	 * @return value prior to the substraction
	 */
	inline T fetch_sub(T num, memory_order order = memory_order::seq_cst)
	{
#if defined(__PX4_NUTTX)

		if (!__atomic_always_lock_free(sizeof(T), 0)) {
			irqstate_t flags = enter_critical_section();
			T ret = _value;
			_value -= num;
			leave_critical_section(flags);
			return ret;
		}

#endif // __PX4_NUTTX

#if PX4_ATOMIC_SINGLE_CORE
		__atomic_signal_fence(static_cast<int>(order));
		T ret = __atomic_fetch_sub(&_value, num, __ATOMIC_RELAXED);
		__atomic_signal_fence(static_cast<int>(order));
		return ret;
#else
		return __atomic_fetch_sub(&_value, num, static_cast<int>(order));
#endif
	}

	/**
	 * Atomic AND with a number
	 * @return value prior to the operation
	 */
	inline T fetch_and(T num, memory_order order = memory_order::seq_cst)
	{
#if defined(__PX4_NUTTX)

		if (!__atomic_always_lock_free(sizeof(T), 0)) {
			irqstate_t flags = enter_critical_section();
			T val = _value;
			_value &= num;
			leave_critical_section(flags);
			return val;
		}

#endif // __PX4_NUTTX

#if PX4_ATOMIC_SINGLE_CORE
		__atomic_signal_fence(static_cast<int>(order));
		T ret = __atomic_fetch_and(&_value, num, __ATOMIC_RELAXED);
		__atomic_signal_fence(static_cast<int>(order));
		return ret;
#else
		return __atomic_fetch_and(&_value, num, static_cast<int>(order));
#endif
	}

	/**
	 * Atomic XOR with a number
	 * @return value prior to the operation
	 */
	inline T fetch_xor(T num, memory_order order = memory_order::seq_cst)
	{
#if defined(__PX4_NUTTX)

		if (!__atomic_always_lock_free(sizeof(T), 0)) {
			irqstate_t flags = enter_critical_section();
			T val = _value;
			_value ^= num;
			leave_critical_section(flags);
			return val;
		}

#endif // __PX4_NUTTX

#if PX4_ATOMIC_SINGLE_CORE
		__atomic_signal_fence(static_cast<int>(order));
		T ret = __atomic_fetch_xor(&_value, num, __ATOMIC_RELAXED);
		__atomic_signal_fence(static_cast<int>(order));
		return ret;
#else
		return __atomic_fetch_xor(&_value, num, static_cast<int>(order));
#endif
	}

	/**
	 * Atomic OR with a number
	 * @return value prior to the operation
	 */
	inline T fetch_or(T num, memory_order order = memory_order::seq_cst)
	{
#if defined(__PX4_NUTTX)

		if (!__atomic_always_lock_free(sizeof(T), 0)) {
			irqstate_t flags = enter_critical_section();
			T val = _value;
			_value |= num;
			leave_critical_section(flags);
			return val;
		}

#endif // __PX4_NUTTX

#if PX4_ATOMIC_SINGLE_CORE
		__atomic_signal_fence(static_cast<int>(order));
		T ret = __atomic_fetch_or(&_value, num, __ATOMIC_RELAXED);
		__atomic_signal_fence(static_cast<int>(order));
		return ret;
#else
		return __atomic_fetch_or(&_value, num, static_cast<int>(order));
#endif
	}

	/**
	 * Atomic NAND (~(_value & num)) with a number
	 * @return value prior to the operation
	 */
	inline T fetch_nand(T num, memory_order order = memory_order::seq_cst)
	{
#if defined(__PX4_NUTTX)

		if (!__atomic_always_lock_free(sizeof(T), 0)) {
			irqstate_t flags = enter_critical_section();
			T ret = _value;
			_value = ~(_value & num);
			leave_critical_section(flags);
			return ret;
		}

#endif // __PX4_NUTTX

#if PX4_ATOMIC_SINGLE_CORE
		__atomic_signal_fence(static_cast<int>(order));
		T ret = __atomic_fetch_nand(&_value, num, __ATOMIC_RELAXED);
		__atomic_signal_fence(static_cast<int>(order));
		return ret;
#else
		return __atomic_fetch_nand(&_value, num, static_cast<int>(order));
#endif
	}

	/**
	 * Atomic compare and exchange operation.
	 * This compares the contents of _value with the contents of *expected. If
	 * equal, the operation is a read-modify-write operation that writes desired
	 * into _value. If they are not equal, the operation is a read and the current
	 * contents of _value are written into *expected.
	 * @return If desired is written into _value then true is returned
	 */
	inline bool compare_exchange(T *expected, T desired, memory_order order = memory_order::seq_cst)
	{
#if defined(__PX4_NUTTX)

		if (!__atomic_always_lock_free(sizeof(T), 0)) {
			irqstate_t flags = enter_critical_section();

			if (_value == *expected) {
				_value = desired;
				leave_critical_section(flags);
				return true;

			} else {
				*expected = _value;
				leave_critical_section(flags);
				return false;
			}
		}

#endif // __PX4_NUTTX

#if PX4_ATOMIC_SINGLE_CORE
		__atomic_signal_fence(static_cast<int>(order));
		bool ret = __atomic_compare_exchange(&_value, expected, &desired, false, __ATOMIC_RELAXED, __ATOMIC_RELAXED);
		__atomic_signal_fence(static_cast<int>(order));
		return ret;
#else
		return __atomic_compare_exchange(&_value, expected, &desired, false, static_cast<int>(order), __ATOMIC_RELAXED);
#endif
	}

private:

	T _value {};
};

using atomic_int = atomic<int>;
using atomic_int32_t = atomic<int32_t>;
using atomic_bool = atomic<bool>;

} /* namespace px4 */

#endif /* __cplusplus */
