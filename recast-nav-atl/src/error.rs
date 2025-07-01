use bitflags::bitflags;
use recast_nav_atl_sys::bindings::root::{ 
    dtStatus, 
    DT_FAILURE,
    DT_STATUS_DETAIL_MASK
};
use std::{
    error::Error as ErrorStd,
    fmt::Display
};

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    pub struct Error : u32 {
        const DT_WRONG_MAGIC = 1 << 0;		// Input data is not recognized.
        const DT_WRONG_VERSION = 1 << 1;	// Input data is in wrong version.
        const DT_OUT_OF_MEMORY = 1 << 2;	// Operation ran out of memory.
        const DT_INVALID_PARAM = 1 << 3;	// An input parameter was invalid.
        const DT_BUFFER_TOO_SMALL = 1 << 4;	// Result buffer for the query was too small to store all results.
        const DT_OUT_OF_NODES = 1 << 5;		// Query ran out of nodes during search.
        const DT_PARTIAL_RESULT = 1 << 6;	// Query did not reach the end location, returning best guess. 
        const DT_ALREADY_OCCUPIED = 1 << 7;	// A tile has already been assigned to the given x,y coordinate
    }
}

impl Error {
    pub fn make_error(status: dtStatus) -> Option<Error> {
        if status & DT_FAILURE != 0 {
            Some(Error::from_bits_truncate(status & DT_STATUS_DETAIL_MASK))
        } else {
            None
        }
    }
}

impl From<Error> for dtStatus {
    fn from(value: Error) -> Self {
        value.bits()
    }
}

impl ErrorStd for Error {}

impl Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Recast Error: {:?}", *self)
    }
}