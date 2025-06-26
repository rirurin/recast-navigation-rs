use allocator_api2::alloc::{ AllocError, Allocator };
use std::{
    alloc::Layout,
    ffi::c_void,
    ptr::NonNull,
};

pub struct DetourAllocator;

unsafe impl Allocator for DetourAllocator {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        match unsafe { recast_nav_atl_sys::bindings::root::dtAlloc(layout.size(), 0) } {
            alloc if alloc == std::ptr::null_mut() => Err(AllocError),
            alloc => Ok(NonNull::slice_from_raw_parts(
                unsafe { NonNull::new_unchecked(alloc as *mut u8) }, 
                layout.size()
            ))
        }
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, _layout: Layout) {
        recast_nav_atl_sys::bindings::root::dtFree(ptr.as_ptr() as *mut c_void)
    }
}

impl DetourAllocator {
    pub unsafe fn set_custom(
        malloc: recast_nav_atl_sys::bindings::root::dtAllocFunc,
        free: recast_nav_atl_sys::bindings::root::dtFreeFunc
    ) { recast_nav_atl_sys::bindings::root::dtAllocSetCustom(malloc, free); }
}