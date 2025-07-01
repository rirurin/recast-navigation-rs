use allocator_api2::boxed::Box;
use crate::allocator::DetourAllocator;
use glam::{ Vec2, Vec3 };
use std::mem::MaybeUninit;
use recast_nav_atl_sys::bindings::root::rcChunkyTriMesh;

type U<T> = MaybeUninit<T>;

#[repr(C)]
pub struct ChunkyTriMesh(rcChunkyTriMesh);

impl From<&rcChunkyTriMesh> for &ChunkyTriMesh {
    fn from(value: &rcChunkyTriMesh) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut rcChunkyTriMesh> for &mut ChunkyTriMesh {
    fn from(value: &mut rcChunkyTriMesh) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut rcChunkyTriMesh> for &ChunkyTriMesh {
    fn from(value: &mut rcChunkyTriMesh) -> Self { unsafe { std::mem::transmute(value) } }
}

impl ChunkyTriMesh {
    /// `rcChunkyTriMesh::rcCreateChunkyTriMesh`
    /// 
    /// Creates partitioned triangle mesh (AABB tree),
    /// where each node contains at max trisPerChunk triangles.
    pub fn new(verts: &[Vec3], tris: &[u32], tris_per_chunk: u32) -> Box<ChunkyTriMesh, DetourAllocator> {
        let mut out: Box<U<ChunkyTriMesh>, DetourAllocator> = Box::new_uninit_in(DetourAllocator);
        unsafe { 
            recast_nav_atl_sys::bindings::root::rcCreateChunkyTriMesh(
                verts.as_ptr() as *const f32, 
                tris.as_ptr() as *const i32,
                tris.len() as i32, 
                tris_per_chunk as i32,
                &raw mut out.assume_init_mut().0
            );
            out.assume_init()
        }
    }

    /// `rcChunkyTriMesh::rcGetChunksOverlappingRect`
    /// 
    /// Returns the chunk indices which overlap the input rectable.
    pub fn get_chunks_overlapping_rect<const N: usize>(&self, min: Vec2, max: Vec2) -> Vec<u32> {
        let mut ids = Vec::with_capacity(N);
        unsafe { 
            let count = recast_nav_atl_sys::bindings::root::rcGetChunksOverlappingRect(
                &raw const self.0,
                &raw const min as *mut f32,
                &raw const max as *mut f32,
                ids.as_mut_ptr() as *mut i32,
                N as i32
            );
            ids.set_len(count as usize);
            ids
        }
    }

    /// `rcChunkyTriMesh::rcGetChunksOverlappingSegment`
    /// 
    /// Returns the chunk indices which overlap the input segment.
    pub fn get_chunks_overlapping_segment<const N: usize>(&self, p: Vec2, q: Vec2) -> Vec<u32> {
        let mut ids = Vec::with_capacity(N);
        unsafe { 
            let count = recast_nav_atl_sys::bindings::root::rcGetChunksOverlappingSegment(
                &raw const self.0,
                &raw const p as *mut f32,
                &raw const q as *mut f32,
                ids.as_mut_ptr() as *mut i32,
                N as i32
            );
            ids.set_len(count as usize);
            ids
        }
    }
}