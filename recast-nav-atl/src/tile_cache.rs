use crate::{
    nav_mesh::NavMesh,
    error::Error as RcError
};
use glam::Vec3A;
use std::mem::MaybeUninit;
use recast_nav_atl_sys::bindings::root::{ 
    dtCompressedTile,
    dtTileCache,
    dtTileCacheLayerHeader,
    dtTileCacheObstacle
};

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
}

#[allow(dead_code)]
pub struct TileCache(pub(crate) dtTileCache);

impl From<&dtTileCache> for &TileCache {
    fn from(value: &dtTileCache) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtTileCache> for &mut TileCache {
    fn from(value: &mut dtTileCache) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtTileCache> for &TileCache {
    fn from(value: &mut dtTileCache) -> Self { unsafe { std::mem::transmute(value) } }
}

const QUERY_TILES_MAX_RESULTS: usize = 32;

impl TileCache {

    /// `dtTileCache::encodeTileId`
    /// 
    /// Encodes a tile id.
    pub fn encode_tile_id(&self, salt: u32, id: u32) -> u32 {
        (salt << self.0.m_tileBits) | id
    }

    /// `dtTileCache::decodeTileIdSalt`
    /// 
    /// Decodes a tile salt.
    pub fn decode_tile_salt(&self, _ref: u32) -> u32 {
        let mask = (1 << self.0.m_saltBits) - 1u32;
        (_ref >> self.0.m_tileBits) & mask
    }

    fn add_cylinder_obstacle_inner(&mut self, pos: Vec3A, radius: f32, height: f32) -> Result<usize, RcError> {
        let pos_raw = &raw const pos as *const f32;
        let mut result: MaybeUninit<u32> = MaybeUninit::uninit();
        match RcError::make_error(unsafe { self.0.addObstacle(pos_raw, radius, height, result.assume_init_mut()) }) {
            Some(v) => Err(v),
            None => Ok(Obstacle::decode_id(unsafe { result.assume_init() }) as usize),
        }
    }

    /// `dtTileCache::addObstacle`
    /// 
	/// Cylinder obstacle.
    pub fn add_cylinder_obstacle(&mut self, pos: Vec3A, radius: f32, height: f32) -> Result<&Obstacle, RcError> {
        self.add_cylinder_obstacle_inner(pos, radius, height).map(|v| unsafe { (&*self.0.m_obstacles.add(v)).into() })
    }

    /// `dtTileCache::addObstacle`
    /// 
	/// Cylinder obstacle.
    pub fn add_cylinder_obstacle_mut(&mut self, pos: Vec3A, radius: f32, height: f32) -> Result<&mut Obstacle, RcError> {
        self.add_cylinder_obstacle_inner(pos, radius, height).map(|v| unsafe { (&mut *self.0.m_obstacles.add(v)).into() })
    }

    fn add_box_obstacle_inner(&mut self, min: Vec3A, max: Vec3A) -> Result<usize, RcError> {
        let min_raw = &raw const min as *const f32;
        let max_raw = &raw const max as *const f32;
        let mut result: MaybeUninit<u32> = MaybeUninit::uninit();
        match RcError::make_error(unsafe { self.0.addBoxObstacle(min_raw, max_raw, result.assume_init_mut()) }) {
            Some(v) => Err(v),
            None => Ok(Obstacle::decode_id(unsafe { result.assume_init() }) as usize),
        }
    }

    /// `dtTileCache::addBoxObstacle`
    /// 
	/// Aabb obstacle.
    pub fn add_box_obstacle(&mut self, min: Vec3A, max: Vec3A) -> Result<&Obstacle, RcError> {
        self.add_box_obstacle_inner(min, max).map(|v| unsafe { (&*self.0.m_obstacles.add(v)).into() })
    }

    /// `dtTileCache::addBoxObstacle`
    /// 
	/// Aabb obstacle.
    pub fn add_box_obstacle_mut(&mut self, min: Vec3A, max: Vec3A) -> Result<&mut Obstacle, RcError> {
        self.add_box_obstacle_inner(min, max).map(|v| unsafe { (&mut *self.0.m_obstacles.add(v)).into() })
    }

    fn add_oriented_box_obstacle_inner(&mut self, min: Vec3A, max: Vec3A, rady: f32) -> Result<usize, RcError> {
        let min_raw = &raw const min as *const f32;
        let max_raw = &raw const max as *const f32;
        let mut result: MaybeUninit<u32> = MaybeUninit::uninit();
        match RcError::make_error(unsafe { self.0.addBoxObstacle1(min_raw, max_raw, rady, result.assume_init_mut()) }) {
            Some(v) => Err(v),
            None => Ok(Obstacle::decode_id(unsafe { result.assume_init() }) as usize),
        }
    }

    /// `dtTileCache::addBoxObstacle`
    /// 
	/// Box obstacle: can be rotated in Y.
    pub fn add_oriented_box_obstacle(&mut self, min: Vec3A, max: Vec3A, rady: f32) -> Result<&Obstacle, RcError> {
        self.add_oriented_box_obstacle_inner(min, max, rady).map(|v| unsafe { (&*self.0.m_obstacles.add(v)).into() })
    }

    /// `dtTileCache::addBoxObstacle`
    /// 
	/// Box obstacle: can be rotated in Y.
    pub fn add_oriented_box_obstacle_mut(&mut self, min: Vec3A, max: Vec3A, rady: f32) -> Result<&mut Obstacle, RcError> {
        self.add_oriented_box_obstacle_inner(min, max, rady).map(|v| unsafe { (&mut *self.0.m_obstacles.add(v)).into() })
    }

    /// `dtTileCache::removeObstacle`
    pub fn remove_obstacle(&mut self, index: usize) -> Result<(), RcError> {
        match RcError::make_error(unsafe { self.0.removeObstacle(index as u32) }) {
            Some(v) => Err(v),
            None => Ok(())
        }
    }

    /* 
    pub unsafe fn get_obstacle_id(&self, obstacle: &Obstacle) -> usize {
        (&raw const *obstacle as usize - self.0.m_obstacles as usize) / size_of::<Obstacle>()
    }
    */

    /// `dtTileCache::queryTiles`
    pub fn query_tiles(&self, min: Vec3A, max: Vec3A) -> Result<Vec<u32>, RcError> {
        let mut results = Vec::with_capacity(QUERY_TILES_MAX_RESULTS);
        let min_raw = &raw const min as *const f32;
        let max_raw = &raw const max as *const f32;
        let mut result_count: MaybeUninit<i32> = MaybeUninit::uninit();
        match RcError::make_error(unsafe { self.0.queryTiles(
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
        match RcError::make_error(unsafe { self.0.update(dt, &raw mut nav_mesh.0, updated.as_mut_ptr()) }) {
            Some(v) => Err(v),
            None => Ok(unsafe { updated.assume_init() })
        }
    }

    /// `dtTileCache::calcTightTileBounds`
    pub fn calculate_tight_tile_bounds(&self, header: &dtTileCacheLayerHeader) -> (Vec3A, Vec3A) {
        let mut bmin: MaybeUninit<Vec3A> = MaybeUninit::uninit();
        let mut bmax: MaybeUninit<Vec3A> = MaybeUninit::uninit();
        unsafe {
            self.0.calcTightTileBounds(header, bmin.as_mut_ptr() as *mut f32, bmax.as_mut_ptr() as *mut f32);
            (bmin.assume_init(), bmax.assume_init())
        }
    }

}