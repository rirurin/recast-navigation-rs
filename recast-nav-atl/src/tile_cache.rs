use allocator_api2::boxed::Box;
use crate::{
    allocator::DetourAllocator,
    nav_mesh::NavMesh,
    error::Error as RcError
};
use glam::{ Vec2, Vec3A, UVec2 };
use std::{
    marker::PhantomData,
    mem::MaybeUninit,
    num::NonZeroU32
};
use recast_nav_atl_sys::bindings::root::{ 
    dtCompressedTile,
    dtNavMeshCreateParams,
    dtStatus,
    dtTileCache,
    dtTileCacheLayerHeader,
    dtTileCacheObstacle,
    dtTileCacheParams,
};

type U<T> = MaybeUninit<T>;

#[allow(dead_code)]
pub struct Tile(pub(crate) dtCompressedTile);

impl From<&dtCompressedTile> for &Tile {
    fn from(value: &dtCompressedTile) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtCompressedTile> for &mut Tile {
    fn from(value: &mut dtCompressedTile) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtCompressedTile> for &Tile {
    fn from(value: &mut dtCompressedTile) -> Self { unsafe { std::mem::transmute(value) } }
}

#[allow(dead_code)]
pub struct Obstacle(pub(crate) dtTileCacheObstacle);

impl From<&dtTileCacheObstacle> for &Obstacle {
    fn from(value: &dtTileCacheObstacle) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtTileCacheObstacle> for &mut Obstacle {
    fn from(value: &mut dtTileCacheObstacle) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtTileCacheObstacle> for &Obstacle {
    fn from(value: &mut dtTileCacheObstacle) -> Self { unsafe { std::mem::transmute(value) } }
}

pub const OBSTACLE_MASK_SIZE: usize = 16;

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ObstacleState {
    Empty = 0,
    Processing = 1,
    Processed = 2,
    Removing = 3
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ObstacleType {
    Cylinder = 0,
    Box = 1,
    OrientedBox = 2
}

impl Obstacle {

    /// `dtTileCache::encodeObstacleId`
    /// 
    /// Encodes an obstacle id.
    pub fn encode_id(salt: u32, id: u32) -> u32 {
        (salt << OBSTACLE_MASK_SIZE) | id
    }

    /// `dtTileCache::decodeObstacleIdSalt`
    /// 
    /// Decodes an obstacle salt.
    pub fn decode_salt(_ref: u32) -> u32 {
        let mask = (1 << OBSTACLE_MASK_SIZE) - 1u32; // 0xffff
        (_ref >> OBSTACLE_MASK_SIZE) & mask
    }

    /// `dtTileCache::decodeObstacleIdObstacle`
    /// 
    /// Decodes an obstacle salt.
    pub fn decode_id(_ref: u32) -> u32 {
        let mask = (1 << OBSTACLE_MASK_SIZE) - 1u32; // 0xffff
        _ref & mask
    }

    /// `dtTileCache::getObstacleBounds`
    pub fn get_bounds(&self) -> (Vec3A, Vec3A) {
        let mut bmin: MaybeUninit<Vec3A> = MaybeUninit::uninit();
        let mut bmax: MaybeUninit<Vec3A> = MaybeUninit::uninit();
        unsafe { 
            recast_nav_atl_sys::bindings::root::dtTileCache_getObstacleBounds(
                std::ptr::null(), // SAFETY: No fields from TileCache are accessed
                &raw const self.0, 
                bmin.as_mut_ptr() as *mut f32, 
                bmax.as_mut_ptr() as *mut f32
            );
            (bmin.assume_init(), bmax.assume_init())
        }
    }

    pub fn get_type(&self) -> ObstacleType { unsafe { std::mem::transmute(self.0.type_) } }
    pub fn get_state(&self) -> ObstacleState { unsafe { std::mem::transmute(self.0.state) } }

    pub fn get_position(&self) -> Vec3A {
        match self.get_type() {
            ObstacleType::Cylinder => unsafe { self.0.__bindgen_anon_1.cylinder.pos.into() },
            ObstacleType::Box => {
                let _box = unsafe { &self.0.__bindgen_anon_1.box_ };
                let size = Into::<Vec3A>::into(_box.bmax) - Into::<Vec3A>::into(_box.bmin);
                Into::<Vec3A>::into(_box.bmin) + (size / 2.)
            },
            ObstacleType::OrientedBox => unsafe { self.0.__bindgen_anon_1.orientedBox.center.into() }
        }
    }

    pub fn get_radius(&self) -> Option<f32> {
        match self.get_type() {
            ObstacleType::Cylinder => Some(unsafe { self.0.__bindgen_anon_1.cylinder.radius }),
            _ => None
        }
    }

    pub fn get_height(&self) -> Option<f32> {
        match self.get_type() {
            ObstacleType::Cylinder => Some(unsafe { self.0.__bindgen_anon_1.cylinder.height }),
            _ => None
        }
    }

    pub fn get_min_bound(&self) -> Option<Vec3A> {
        match self.get_type() {
            ObstacleType::Box => Some(unsafe{ self.0.__bindgen_anon_1.box_.bmin.into() }),
            _ => None
        }
    }

    pub fn get_max_bound(&self) -> Option<Vec3A> {
        match self.get_type() {
            ObstacleType::Box => Some(unsafe{ self.0.__bindgen_anon_1.box_.bmax.into() }),
            _ => None
        }
    }

    pub fn get_half_extents(&self) -> Option<Vec3A> {
        match self.get_type() {
            ObstacleType::OrientedBox => Some(unsafe { self.0.__bindgen_anon_1.orientedBox.halfExtents.into() }),
            _ => None
        }
    }

    pub fn get_rot_aux(&self) -> Option<Vec2> {
        match self.get_type() {
            ObstacleType::OrientedBox => Some(unsafe { self.0.__bindgen_anon_1.orientedBox.rotAux.into() }),
            _ => None
        }
    }
}

#[allow(dead_code)]
pub struct TileCache<A, C, P> 
where A: TileCacheAllocator,
    C: TileCacheCompressor,
    P: MeshProcess
{
    pub(crate) data: dtTileCache,
    _allocator: PhantomData<A>,
    _compressor: PhantomData<C>,
    _processor: PhantomData<P>,
}

impl<A, C, P> From<&dtTileCache> for &TileCache<A, C, P> 
where A: TileCacheAllocator,
    C: TileCacheCompressor,
    P: MeshProcess
{
    fn from(value: &dtTileCache) -> Self { unsafe { std::mem::transmute(value) } }
}

impl<A, C, P> From<&mut dtTileCache> for &mut TileCache<A, C, P>
where A: TileCacheAllocator,
    C: TileCacheCompressor,
    P: MeshProcess
{
    fn from(value: &mut dtTileCache) -> Self { unsafe { std::mem::transmute(value) } }
}

impl<A, C, P> From<&mut dtTileCache> for &TileCache<A, C, P>
where A: TileCacheAllocator,
    C: TileCacheCompressor,
    P: MeshProcess
{
    fn from(value: &mut dtTileCache) -> Self { unsafe { std::mem::transmute(value) } }
}

const QUERY_TILES_MAX_RESULTS: usize = 32;

impl<A, C, P> TileCache<A, C, P>
where A: TileCacheAllocator,
    C: TileCacheCompressor,
    P: MeshProcess
{

    /// `dtTileCache::Init`
    pub unsafe fn new(params: &dtTileCacheParams, allocator: &A, compressor: &C, processor: &P)
    -> Result<Box<Self, DetourAllocator>, RcError> {
        let mut new = Box::new_in(Self {
            data: unsafe { dtTileCache::new() },
            _allocator: PhantomData::<A>,
            _compressor: PhantomData::<C>,
            _processor: PhantomData::<P>,
        }, DetourAllocator);
        match RcError::make_error(unsafe { new.as_mut().data.init(
            params, 
            &raw const *allocator as *mut recast_nav_atl_sys::bindings::root::dtTileCacheAlloc,
            &raw const *compressor as *mut recast_nav_atl_sys::bindings::root::dtTileCacheCompressor,
            &raw const *processor as *mut recast_nav_atl_sys::bindings::root::dtTileCacheMeshProcess
        ) }) {
            Some(v) => Err(v),
            None => Ok(new)
        }
    }

    /// `dtTileCache::getAlloc`
    pub fn get_allocator(&self) -> &A { unsafe { &*(self.data.m_talloc as *const A) } }

    /// `dtTileCache::getAlloc`
    pub fn get_allocator_mut(&mut self) -> &mut A { unsafe { &mut *(self.data.m_talloc as *mut A) } }

    /// `dtTileCache::getCompressor`
    pub fn get_compressor(&self) -> &C { unsafe { &*(self.data.m_tcomp as *const C) } }

    /// `dtTileCache::getCompressor`
    pub fn get_compressor_mut(&mut self) -> &mut C { unsafe { &mut *(self.data.m_tcomp as *mut C) } }

    /// `dtTileCache::getParams`
    pub fn get_params(&self) -> &dtTileCacheParams { &self.data.m_params }

    /// `dtTileCache::getTileCount`
    pub fn get_tile_len(&self) -> usize { self.get_params().maxTiles as usize }

    /// `dtTileCache::getTile`
    pub fn get_tile(&self, index: usize) -> Option<&dtCompressedTile> { 
        if index < self.get_tile_len() {
            Some(unsafe { &*self.data.m_tiles.add(index) })
        } else {
            None
        }
    }

    /// `dtTileCache::getTile`
    pub fn get_tile_mut(&mut self, index: usize) -> Option<&mut dtCompressedTile> { 
        if index < self.get_tile_len() {
            Some(unsafe { &mut *self.data.m_tiles.add(index) })
        } else {
            None
        }
    }

    /// `dtTileCache::getObstacleCount`
    pub fn get_obstacle_len(&self) -> usize { self.get_params().maxObstacles as usize }

    /// `dtTileCache::getTile`
    pub fn get_obstacle(&self, index: usize) -> Option<&Obstacle> { 
        if index < self.get_obstacle_len() {
            Some(unsafe { (&*self.data.m_obstacles.add(index)).into() })
        } else {
            None
        }
    }

    /// `dtTileCache::getObstacleByRef`
    pub fn get_obstacle_by_ref(&self, _ref: u32) -> Option<&Obstacle> {
        // Returned dtTileCacheObstacle* is const so &self should be fine but FFI signature demands &mut self
        let this = unsafe { &mut *(&raw const *self as *mut Self) };
        match unsafe { this.data.getObstacleByRef(_ref) } {
            p if p != std::ptr::null() => Some(unsafe { (&*p).into() }),
            _ => None
        }
    }

    /// `dtTileCache::getTilesAt`
    pub fn get_tiles_at<const N: usize>(&self, pos: UVec2) -> Vec<u32> {
        let mut value = Vec::with_capacity(N);
        let count = unsafe { self.data.getTilesAt(pos.x as i32, pos.y as i32, value.as_mut_ptr(), N as i32) };
        unsafe { value.set_len(count as usize) }
        value
    }

    /// `dtTileCache::getTileAt`
    pub fn get_tile_at(&mut self, pos: UVec2, layer: u32) -> Option<&mut dtCompressedTile> {
        match unsafe { self.data.getTileAt(pos.x as i32, pos.y as i32, layer as i32) } {
            v if v != std::ptr::null_mut() => Some(unsafe { &mut *v }),
            _ => None
        }
    }

    /// `dtTileCache::getTileRef`
    pub fn get_tile_ref(&self, tile: &dtCompressedTile) -> Option<NonZeroU32> {
        match unsafe { self.data.getTileRef(tile) } {
            k if k != 0 => Some(unsafe { NonZeroU32::new_unchecked(k) }),
            _ => None
        }
    }

    /// `dtTileCache::getTileByRef`
    pub fn get_tile_by_ref(&self, _ref: u32) -> Option<&dtCompressedTile> {
        match unsafe { self.data.getTileByRef(_ref) } {
            k if k != std::ptr::null() => Some(unsafe { (&*k).into() }),
            _ => None
        }
    }

    /// `dtTileCache::addTile`
    /// 
    /// Add a new tile into the tile cache. Returns a tile reference ID if it succeeds.
    pub fn add_tile(&mut self, data: &[u8], flags: u8) -> Result<u32, RcError> {
        let mut result: U<_> = U::uninit();
        match RcError::make_error(unsafe { self.data.addTile(
            data.as_ptr() as *mut u8, data.len() as i32, flags, result.as_mut_ptr())}) {
            Some(v) => Err(v),
            None => Ok(unsafe { result.assume_init() })
        }
    }

    /// `dtTileCache::removeTile`
    ///
    /// Removes the tile from the cache with the matching tile reference. Returns a slice containing the
    /// tile's compressed data if the method succeeds
    pub fn remove_tile(&mut self, _ref: u32) -> Result<&[u8], RcError> {
        let mut data: U<_> = U::uninit();
        let mut data_size: U<_> = U::uninit();
        match RcError::make_error(unsafe { self.data.removeTile(_ref, data.as_mut_ptr(), data_size.as_mut_ptr())}) {
            Some(v) => Err(v),
            None => Ok(unsafe { std::slice::from_raw_parts(data.assume_init(), data_size.assume_init() as usize)})
        }
    }

    fn add_cylinder_obstacle_inner(&mut self, pos: Vec3A, radius: f32, height: f32) -> Result<usize, RcError> {
        let pos_raw = &raw const pos as *const f32;
        let mut result: MaybeUninit<u32> = MaybeUninit::uninit();
        match RcError::make_error(unsafe { self.data.addObstacle(pos_raw, radius, height, result.assume_init_mut()) }) {
            Some(v) => Err(v),
            None => Ok(Obstacle::decode_id(unsafe { result.assume_init() }) as usize),
        }
    }

    /// `dtTileCache::addObstacle`
    /// 
	/// Cylinder obstacle.
    pub fn add_cylinder_obstacle(&mut self, pos: Vec3A, radius: f32, height: f32) -> Result<&Obstacle, RcError> {
        self.add_cylinder_obstacle_inner(pos, radius, height).map(|v| unsafe { (&*self.data.m_obstacles.add(v)).into() })
    }

    /// `dtTileCache::addObstacle`
    /// 
	/// Cylinder obstacle.
    pub fn add_cylinder_obstacle_mut(&mut self, pos: Vec3A, radius: f32, height: f32) -> Result<&mut Obstacle, RcError> {
        self.add_cylinder_obstacle_inner(pos, radius, height).map(|v| unsafe { (&mut *self.data.m_obstacles.add(v)).into() })
    }

    fn add_box_obstacle_inner(&mut self, min: Vec3A, max: Vec3A) -> Result<usize, RcError> {
        let min_raw = &raw const min as *const f32;
        let max_raw = &raw const max as *const f32;
        let mut result: MaybeUninit<u32> = MaybeUninit::uninit();
        match RcError::make_error(unsafe { self.data.addBoxObstacle(min_raw, max_raw, result.assume_init_mut()) }) {
            Some(v) => Err(v),
            None => Ok(Obstacle::decode_id(unsafe { result.assume_init() }) as usize),
        }
    }

    /// `dtTileCache::addBoxObstacle`
    /// 
	/// Aabb obstacle.
    pub fn add_box_obstacle(&mut self, min: Vec3A, max: Vec3A) -> Result<&Obstacle, RcError> {
        self.add_box_obstacle_inner(min, max).map(|v| unsafe { (&*self.data.m_obstacles.add(v)).into() })
    }

    /// `dtTileCache::addBoxObstacle`
    /// 
	/// Aabb obstacle.
    pub fn add_box_obstacle_mut(&mut self, min: Vec3A, max: Vec3A) -> Result<&mut Obstacle, RcError> {
        self.add_box_obstacle_inner(min, max).map(|v| unsafe { (&mut *self.data.m_obstacles.add(v)).into() })
    }

    fn add_oriented_box_obstacle_inner(&mut self, min: Vec3A, max: Vec3A, rady: f32) -> Result<usize, RcError> {
        let min_raw = &raw const min as *const f32;
        let max_raw = &raw const max as *const f32;
        let mut result: MaybeUninit<u32> = MaybeUninit::uninit();
        match RcError::make_error(unsafe { self.data.addBoxObstacle1(min_raw, max_raw, rady, result.assume_init_mut()) }) {
            Some(v) => Err(v),
            None => Ok(Obstacle::decode_id(unsafe { result.assume_init() }) as usize),
        }
    }

    /// `dtTileCache::addBoxObstacle`
    /// 
	/// Box obstacle: can be rotated in Y.
    pub fn add_oriented_box_obstacle(&mut self, min: Vec3A, max: Vec3A, rady: f32) -> Result<&Obstacle, RcError> {
        self.add_oriented_box_obstacle_inner(min, max, rady).map(|v| unsafe { (&*self.data.m_obstacles.add(v)).into() })
    }

    /// `dtTileCache::addBoxObstacle`
    /// 
	/// Box obstacle: can be rotated in Y.
    pub fn add_oriented_box_obstacle_mut(&mut self, min: Vec3A, max: Vec3A, rady: f32) -> Result<&mut Obstacle, RcError> {
        self.add_oriented_box_obstacle_inner(min, max, rady).map(|v| unsafe { (&mut *self.data.m_obstacles.add(v)).into() })
    }

    /// `dtTileCache::removeObstacle`
    pub fn remove_obstacle(&mut self, index: usize) -> Result<(), RcError> {
        match RcError::make_error(unsafe { self.data.removeObstacle(index as u32) }) {
            Some(v) => Err(v),
            None => Ok(())
        }
    }

    /// `dtTileCache::queryTiles`
    pub fn query_tiles(&self, min: Vec3A, max: Vec3A) -> Result<Vec<u32>, RcError> {
        let mut results = Vec::with_capacity(QUERY_TILES_MAX_RESULTS);
        let min_raw = &raw const min as *const f32;
        let max_raw = &raw const max as *const f32;
        let mut result_count: MaybeUninit<i32> = MaybeUninit::uninit();
        match RcError::make_error(unsafe { self.data.queryTiles(
            min_raw, max_raw, 
            results.as_mut_ptr(), 
            result_count.as_mut_ptr(), 
            QUERY_TILES_MAX_RESULTS as i32) 
        }) {
            Some(v) => Err(v),
            None => {
                unsafe { results.set_len(result_count.assume_init() as usize) }
                Ok(results)
            }
        }
    }

    /// `dtTileCache::update`
    /// 
    /// Updates the tile cache by rebuilding tiles touched by unfinished obstacle requests.
    /// 
    /// Returns a boolean value that determines if the tile cache is fully up to date with obstacle requests 
    /// and tile rebuilds. Ifthe tile cache is up to date another (immediate) call to update will have no effect.
	/// Otherwise, another call will continue processing obstacle requests and tile rebuilds.
    pub fn update(&mut self, dt: f32, nav_mesh: &mut NavMesh) -> Result<bool, RcError> {
        let mut updated: MaybeUninit<bool> = MaybeUninit::uninit();
        match RcError::make_error(unsafe { self.data.update(dt, &raw mut nav_mesh.0, updated.as_mut_ptr()) }) {
            Some(v) => Err(v),
            None => Ok(unsafe { updated.assume_init() })
        }
    }

    /// `dtTileCache::buildNavMeshTilesAt`
    pub fn build_nav_mesh_tiles_at(&mut self, pos: UVec2, navmesh: &mut NavMesh) -> Result<(), RcError> {
        match RcError::make_error(unsafe { self.data.buildNavMeshTilesAt(pos.x as i32, pos.y as i32, &raw mut navmesh.0)}) {
            Some(v) => Err(v),
            None => Ok(())
        }
    }

    /// `dtTileCache::buildNavMeshTile`
    pub fn build_nav_mesh_tile(&mut self, _ref: u32, navmesh: &mut NavMesh) -> Result<(), RcError> {
        match RcError::make_error(unsafe { self.data.buildNavMeshTile(_ref, &raw mut navmesh.0)}) {
            Some(v) => Err(v),
            None => Ok(())
        }
    }

    /// `dtTileCache::calcTightTileBounds`
    pub fn calculate_tight_tile_bounds(&self, header: &dtTileCacheLayerHeader) -> (Vec3A, Vec3A) {
        let mut bmin: MaybeUninit<Vec3A> = MaybeUninit::uninit();
        let mut bmax: MaybeUninit<Vec3A> = MaybeUninit::uninit();
        unsafe {
            self.data.calcTightTileBounds(header, bmin.as_mut_ptr() as *mut f32, bmax.as_mut_ptr() as *mut f32);
            (bmin.assume_init(), bmax.assume_init())
        }
    }

    /// `dtTileCache::encodeTileId`
    /// 
    /// Encodes a tile id.
    pub fn encode_tile_id(&self, salt: u32, id: u32) -> u32 {
        (salt << self.data.m_tileBits) | id
    }

    /// `dtTileCache::decodeTileIdSalt`
    /// 
    /// Decodes a tile salt.
    pub fn decode_tile_salt(&self, _ref: u32) -> u32 {
        let mask = (1 << self.data.m_saltBits) - 1u32;
        (_ref >> self.data.m_tileBits) & mask
    }
}

/// `dtTileCacheCompressor`
pub trait TileCacheCompressor where Self: Sized {
    fn create_cpp_vtable() -> [usize; 4] {
        [
            drop::<Self> as usize,
            Self::max_compressed_size as usize,
            Self::compress_ffi as usize,
            Self::decompress_ffi as usize
        ]
    }
    fn max_compressed_size(&self, buffer_size: u32) -> u32;

    unsafe fn compress_ffi(&self, buffer: *const u8, buffer_size: u32, 
        compressed: *mut u8, _: u32, compressed_size: *mut u32) -> dtStatus {
        let buffer = unsafe { std::slice::from_raw_parts(buffer, buffer_size as usize) };
        let max_cmp_size = (buffer_size * self.max_compressed_size(buffer_size)) as usize;
        let compressed = unsafe { std::slice::from_raw_parts_mut(compressed, max_cmp_size) };
        let (size, status) = self.compress(buffer, compressed);
        *compressed_size = size;
        status
    }

    unsafe fn decompress_ffi(&self, cmp: *const u8, cmp_size: u32, 
        buffer: *mut u8, max_size: u32, size: *mut u32) -> dtStatus {
        let cmp = unsafe { std::slice::from_raw_parts(cmp, cmp_size as usize) };
        let buffer = unsafe { std::slice::from_raw_parts_mut(buffer, max_size as usize) };
        let (o_size, status) = self.decompress(cmp, buffer);
        *size = o_size;
        status
    }

    fn compress(&self, data: &[u8], compressed: &mut [u8]) -> (u32, dtStatus);
    fn decompress(&self, compressed: &[u8], data: &mut [u8]) -> (u32, dtStatus);
}

/// `dtTileCacheMeshProcess`
pub trait MeshProcess where Self: Sized {
    fn create_cpp_vtable() -> [usize; 2] {
        [
            drop::<Self> as usize,
            Self::process_ffi as usize
        ]
    }

    unsafe fn process_ffi(&self, params: *const dtNavMeshCreateParams, poly_area: *mut u8, poly_flags: *mut u16) {
        let params = std::mem::transmute::<_, &dtNavMeshCreateParams>(params);
        let poly_area = std::slice::from_raw_parts_mut(poly_area, params.polyCount as usize);
        let poly_flags = std::slice::from_raw_parts_mut(poly_flags, params.polyCount as usize);
        self.process(params, poly_area, poly_flags)
    }

    fn process(&self, params: &dtNavMeshCreateParams, poly_area: &mut [u8], poly_flags: &mut [u16]);
}

/// `dtTileCacheAlloc`
pub trait TileCacheAllocator where Self: Sized {
    fn create_cpp_vtable() -> [usize; 4] {
        [
            drop::<Self> as usize,
            Self::reset as usize,
            Self::alloc as usize,
            Self::free as usize
        ]
    }
    fn reset(&mut self);
    fn alloc(&mut self, size: usize) -> *const u8;
    fn free(&mut self, ptr: *const u8);
}