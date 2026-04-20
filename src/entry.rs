#![allow(unsafe_code)]

/// Bootstrap hardware before main() runs.
///
/// # Safety
/// This is a copy of what hal::entry does. Call only once at bootup before threads start.
/// Uses raw pointers to hardware registers (spinlocks, co-processor config) that must be
/// initialized before any Rust code runs. Caller guarantees single-threaded init context.
pub unsafe fn entry() {
    const SIO_BASE: u32 = 0xd0000000;
    const SPINLOCK0_PTR: *mut u32 = (SIO_BASE + 0x100) as *mut u32;
    const SPINLOCK_COUNT: usize = 32;
    for i in 0..SPINLOCK_COUNT {
        // Reset all spinlocks via raw pointer write. Safe because: (1) addresses are correct
        // in RP2350 memory map, (2) writing to spinlock registers is idempotent, (3) we're
        // in single-threaded init context.
        unsafe { SPINLOCK0_PTR.wrapping_add(i).write_volatile(1) };
    }
    #[cfg(target_arch = "arm")]
    {
        // Enable the Double-Co-Pro and the GPIO Co-Pro in the CPACR register.
        // We have to do this early, before there's a chance we might call
        // any accelerated functions.
        const SCB_CPACR_PTR: *mut u32 = 0xE000_ED88 as *mut u32;
        const SCB_CPACR_FULL_ACCESS: u32 = 0b11;
        // Do a R-M-W, because the FPU enable is here and that's already been enabled
        let mut temp = unsafe { SCB_CPACR_PTR.read_volatile() };
        // DCP Co-Pro is 4, two-bits per entry
        temp |= SCB_CPACR_FULL_ACCESS << (4 * 2);
        // GPIO Co-Pro is 0, two-bits per entry
        #[allow(clippy::erasing_op, clippy::identity_op)]
        {
            temp |= SCB_CPACR_FULL_ACCESS << (0 * 2);
        }
        // Configure CPACR to allow DCP and GPIO co-processor access. Safe because:
        // (1) CPACR address is correct for Cortex-M33, (2) register is meant for this config,
        // (3) we're in single-threaded init context.
        unsafe { SCB_CPACR_PTR.write_volatile(temp) };
        // Don't allow any DCP code to be moved before this fence.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}
