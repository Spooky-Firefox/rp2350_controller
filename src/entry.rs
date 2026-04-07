#![allow(unsafe_code)]

/// # Safety
/// this is a copy of what hal::entry does, so do not use this except in the beginning of the program
pub unsafe fn entry() {
    const SIO_BASE: u32 = 0xd0000000;
    const SPINLOCK0_PTR: *mut u32 = (SIO_BASE + 0x100) as *mut u32;
    const SPINLOCK_COUNT: usize = 32;
    for i in 0..SPINLOCK_COUNT {
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
        unsafe { SCB_CPACR_PTR.write_volatile(temp) };
        // Don't allow any DCP code to be moved before this fence.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}
