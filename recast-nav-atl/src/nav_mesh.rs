use allocator_api2::boxed::Box;
use crate::{
    allocator::DetourAllocator,
    crowd::QueryFilter,
    error::Error as RcError
};
use glam::Vec3A;
use std::mem::MaybeUninit;
use recast_nav_atl_sys::bindings::root::{
    dtNavMesh,
    dtNavMeshQuery
};

#[allow(dead_code)]
pub struct NavMeshQuery(pub(crate) dtNavMeshQuery);

impl From<&dtNavMeshQuery> for &NavMeshQuery {
    fn from(value: &dtNavMeshQuery) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtNavMeshQuery> for &mut NavMeshQuery {
    fn from(value: &mut dtNavMeshQuery) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtNavMeshQuery> for &NavMeshQuery {
    fn from(value: &mut dtNavMeshQuery) -> Self { unsafe { std::mem::transmute(value) } }
}

const MESH_QUERY_MAX: usize = 256;

impl NavMeshQuery {
    /// `dtNavMeshQuery::init`
    pub fn new(nav: &NavMesh, max_nodes: usize) -> Result<Box<Self, DetourAllocator>, RcError> {
        let mut new = Box::new_in(Self(unsafe { dtNavMeshQuery::new() }), DetourAllocator);
        match RcError::make_error(unsafe { new.as_mut().0.init(&raw const nav.0 as *mut _, max_nodes as i32) }) {
            Some(v) => Err(v),
            None => Ok(new)
        }
    }

    /// `dtNavMeshQuery::findPath`
    /// 
    /// Finds a path from the start polygon to the end polygon.
    pub fn find_path(
        &self, 
        start_ref: usize, 
        end_ref: usize, 
        start_pos: Vec3A,
        end_pos: Vec3A,
        filter: &QueryFilter,
    ) -> Result<Vec<u32>, RcError> {
        let start_raw = &raw const start_pos as *const f32;
        let end_raw = &raw const end_pos as *const f32;
        let mut path = Vec::with_capacity(MESH_QUERY_MAX);
        let mut path_len  = MaybeUninit::uninit();
        let status = unsafe { self.0.findPath(
            start_ref as u32, end_ref as u32, start_raw, end_raw, &raw const filter.0, 
            path.as_ptr() as *mut u32, path_len.assume_init_mut(), MESH_QUERY_MAX as i32) };
        unsafe { path.set_len(path_len.assume_init() as usize) }
        match RcError::make_error(status) {
            Some(v) => Err(v),
            None => Ok(path)
        }
    }
}

#[allow(dead_code)]
pub struct NavMesh(pub(crate) dtNavMesh);

impl From<&dtNavMesh> for &NavMesh {
    fn from(value: &dtNavMesh) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtNavMesh> for &mut NavMesh {
    fn from(value: &mut dtNavMesh) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtNavMesh> for &NavMesh {
    fn from(value: &mut dtNavMesh) -> Self { unsafe { std::mem::transmute(value) } }
}