use allocator_api2::boxed::Box;
use bitflags::bitflags;
use crate::{
    allocator::DetourAllocator,
    crowd::QueryFilter,
    error::Error as RcError,
};
use glam::{ Vec3, Vec3A, UVec2 };
use std::mem::MaybeUninit;
use recast_nav_atl_sys::bindings::root::{
    dtMeshTile,
    dtNavMesh,
    dtNavMeshParams,
    dtNavMeshQuery
};

type U<T> = MaybeUninit<T>;

/// Provides the ability to perform pathfinding related queries against
/// a navigation mesh.
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

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    pub struct StraightPathFlags : u8 {
	    const Start = 0x01;				// The vertex is the start position in the path.
	    const End = 0x02;					// The vertex is the end position in the path.
	    const OffMeshConnection = 0x04;	// The vertex is the start of an off-mesh connection.
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    pub struct StraightPathOptions : u32 {
	    const AreaCrossing = 0x01;	// Add a vertex at every polygon edge crossing where area changes.
	    const AllCrossings = 0x02;	// Add a vertex at every polygon edge crossing.
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    pub struct TileFlags : u32 {
	    const FreeData = 0x01;				// The navigation mesh owns the tile memory and is responsible for freeing it.
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    pub struct FindPathOptions : u32 {
	    const AnyAngle = 0x02;				// use raycasts during pathfind to "shortcut" (raycast still consider costs)
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    pub struct RaycastOptions : u8 {
	    const UseCosts = 0x01;				// Raycast should calculate movement cost along the ray and fill RaycastHit::cost
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    pub struct TriEdgeFlags : u8 {
	    const DetailEdgeBoundary = 0x01;				// Detail triangle edge is part of the poly boundary
    }
}

// const MESH_QUERY_MAX: usize = 256;

pub struct StraightLineResult<const N: usize> {
    points: [Vec3; N],
    flags: [StraightPathFlags; N],
    refs: [u32; N],
    count: u32,
}

impl<const N: usize> StraightLineResult<N> {
    pub fn get_points(&self) -> &[Vec3] { unsafe { std::slice::from_raw_parts(self.points.as_ptr(), self.count as usize) } }
    pub fn get_points_mut(&mut self) -> &mut [Vec3] { unsafe { std::slice::from_raw_parts_mut(self.points.as_mut_ptr(), self.count as usize) } }
    pub fn get_flags(&self) -> &[StraightPathFlags] { unsafe { std::slice::from_raw_parts(self.flags.as_ptr(), self.count as usize) } }
    pub fn get_flags_mut(&mut self) -> &mut [StraightPathFlags] { unsafe { std::slice::from_raw_parts_mut(self.flags.as_mut_ptr(), self.count as usize) } }
    pub fn get_refs(&self) -> &[u32] { unsafe { std::slice::from_raw_parts(self.refs.as_ptr(), self.count as usize) } }
    pub fn get_refs_mut(&mut self) -> &mut [u32] { unsafe { std::slice::from_raw_parts_mut(self.refs.as_mut_ptr(), self.count as usize) } }
}

/// ## Sliced Pathfinding Functions
/// 
/// Common use case:
///	- Call initSlicedFindPath() to initialize the sliced path query.
///	- Call updateSlicedFindPath() until it returns complete.
///	- Call finalizeSlicedFindPath() to get the path.
pub struct SlicedPathfinder<'a>(&'a mut NavMeshQuery);
impl<'a> SlicedPathfinder<'a> {
    /// `dtNavMeshQuery::updateSlicedFindPath`
    /// 
    /// Updates an in-progress sliced path query.
    /// Returns the actual number of iterations completed
    pub fn update(&mut self, max_iter: usize) -> Result<usize, RcError> {
        let mut out: U<_> = U::uninit();
        match RcError::make_error(unsafe { self.0.0.updateSlicedFindPath(max_iter as i32, out.assume_init_mut()) }) {
            Some(v) => Err(v),
            None => Ok(unsafe { out.assume_init() } as usize)
        }
    }

    /// `dtNavMeshQuery::finalizeSlicedFindPath`
    /// 
    /// Finalizes and returns the results of a sliced path query.
    pub fn finalize<const N: usize>(self) -> Result<Vec<u32>, RcError> {
        let mut path = Vec::with_capacity(N);
        let mut path_count: U<_> = U::uninit();
        match RcError::make_error( unsafe { self.0.0.finalizeSlicedFindPath(
            path.as_mut_ptr(), path_count.assume_init_mut(), N as i32) }
        ) {
            Some(v) => Err(v),
            None => {
                unsafe { path.set_len(path_count.assume_init() as usize) }
                Ok(path)
            }
        }
    }
}

pub struct FindPolygonsResult<const N: usize> {
    polygons: [u32; N],
    parents: [u32; N],
    cost: [f32; N],
    count: u32
}

impl<const N: usize> FindPolygonsResult<N> {
    pub fn get_polygons(&self) -> &[u32] { unsafe { std::slice::from_raw_parts(self.polygons.as_ptr(), self.count as usize) } }
    pub fn get_polygons_mut(&mut self) -> &mut [u32] { unsafe { std::slice::from_raw_parts_mut(self.polygons.as_mut_ptr(), self.count as usize) } }
    pub fn get_parents(&self) -> &[u32] { unsafe { std::slice::from_raw_parts(self.parents.as_ptr(), self.count as usize) } }
    pub fn get_parents_mut(&mut self) -> &mut [u32] { unsafe { std::slice::from_raw_parts_mut(self.parents.as_mut_ptr(), self.count as usize) } }
    pub fn get_cost(&self) -> &[f32] { unsafe { std::slice::from_raw_parts(self.cost.as_ptr(), self.count as usize) } }
    pub fn get_cost_mut(&mut self) -> &mut [f32] { unsafe { std::slice::from_raw_parts_mut(self.cost.as_mut_ptr(), self.count as usize) } }
}

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
    /// Returns an ordered list of polygon references representing the path.
    pub fn find_path<const N: usize>(
        &self, 
        start_ref: usize, 
        end_ref: usize, 
        start_pos: Vec3A,
        end_pos: Vec3A,
        filter: &QueryFilter,
    ) -> Result<Vec<u32>, RcError> {
        let start_raw = &raw const start_pos as *const f32;
        let end_raw = &raw const end_pos as *const f32;
        let mut path = Vec::with_capacity(N);
        let mut path_len  = MaybeUninit::uninit();
        match RcError::make_error(unsafe { self.0.findPath(
            start_ref as u32, end_ref as u32, start_raw, end_raw, &raw const filter.0, 
            path.as_ptr() as *mut u32, path_len.assume_init_mut(), N as i32) }
        ) {
            Some(v) => Err(v),
            None => {
                unsafe { path.set_len(path_len.assume_init() as usize) }
                Ok(path)
            }
        }
    }

    /// `dtNavMeshQuery::findStraightPath`
    /// 
    /// Finds the straight path from the start to the end position within the polygon corridor.
    /// Returns a StraightLineResult structure if it succeeds, consisting of points describing the straight path, 
    /// flags describing each point and the reference of the polygon that is being entered at each point.
    pub fn find_straight_path<const N: usize>(
        &self,
        start_pos: Vec3A,
        end_pos: Vec3A,
        path: &[u32],
        options: StraightPathOptions
    ) -> Result<StraightLineResult<N>, RcError> {
        let start_raw = &raw const start_pos as *const f32;
        let end_raw = &raw const end_pos as *const f32;
        let mut output: MaybeUninit<StraightLineResult<N>> = MaybeUninit::uninit();
        unsafe {
            match RcError::make_error(self.0.findStraightPath(
                start_raw, 
                end_raw, 
                path.as_ptr(), 
                path.len() as i32, 
                output.assume_init_mut().points.as_mut_ptr() as *mut f32, 
                output.assume_init_mut().flags.as_mut_ptr() as *mut u8, 
                output.assume_init_mut().refs.as_mut_ptr(), 
                &raw mut output.assume_init_mut().count as *mut i32, 
                N as i32, 
                options.bits() as i32
            )) {
                Some(v) => Err(v),
                None => Ok(output.assume_init())
            }
        }
    }

    /// `dtNavMeshQuery::initSlicedFindPath`
    /// 
    /// Initializes a sliced path query. 
    /// Returns a SlicedPathfinder object if it succeeds.
    pub fn initialize_sliced_find_path(
        &mut self,
        start_ref: u32, 
        end_ref: u32, 
        start_pos: Vec3A, 
        end_pos: Vec3A,
        filter: &QueryFilter,
        options: FindPathOptions
    ) -> Result<SlicedPathfinder<'_>, RcError> {
        let start_raw = &raw const start_pos as *const f32;
        let end_raw = &raw const end_pos as *const f32;
        match RcError::make_error(unsafe { self.0.initSlicedFindPath(
            start_ref, end_ref, start_raw, 
            end_raw, &raw const filter.0, options.bits() as u32)
        }) {
            Some(v) => Err(v),
            None => Ok(SlicedPathfinder(self))
        }
    }

    /// `dtNavMeshQuery::findPolysAroundCircle`
    /// 
    /// Finds the polygons along the navigation graph that touch the specified circle.
    pub fn find_polygons_around_circle<const N: usize>(
        &self, 
        start_ref: u32,
        center_pos: Vec3A,
        radius: f32,
        filter: &QueryFilter,
    ) -> Result<FindPolygonsResult<N>, RcError> {
        let center_raw = &raw const center_pos as *const f32;
        let mut out: U<FindPolygonsResult<N>> = U::uninit();
        match RcError::make_error(unsafe { self.0.findPolysAroundCircle(
            start_ref, center_raw, radius, &raw const filter.0, 
            out.assume_init_mut().polygons.as_mut_ptr(), 
            out.assume_init_mut().parents.as_mut_ptr(), 
            out.assume_init_mut().cost.as_mut_ptr(), 
            &raw mut out.assume_init_mut().count as *mut i32, 
            N as i32)
        }) {
            Some(v) => Err(v),
            None => Ok(unsafe { out.assume_init() })
        }
    }

    /// `dtNavMeshQuery::findPolyAroundShape`
    /// 
    /// Finds the polygons along the naviation graph that touch the specified convex polygon.
    pub fn find_polygons_around_shape<const N: usize>(
        &self,
        start_ref: u32,
        verts: &[Vec3],
        filter: &QueryFilter
    ) -> Result<FindPolygonsResult<N>, RcError> {
        let mut out: U<FindPolygonsResult<N>> = U::uninit();
        match RcError::make_error(unsafe { self.0.findPolysAroundShape(
            start_ref, 
            verts.as_ptr() as *const f32, 
           verts.len() as i32, 
            &raw const filter.0, 
            out.assume_init_mut().polygons.as_mut_ptr(), 
            out.assume_init_mut().parents.as_mut_ptr(), 
            out.assume_init_mut().cost.as_mut_ptr(), 
            &raw mut out.assume_init_mut().count as *mut i32, 
            N as i32)
        }) {
            Some(v) => Err(v),
            None => Ok(unsafe { out.assume_init() })
        }
    }

    /// `dtNavMeshQuery::getPathFromDijkstraSearch`
    /// 
    /// Gets a path from the explored nodes in the previous search.
    /// The result of this function depends on the state of the query object. For that reason it should only
	/// be used immediately after one of the two Dijkstra searches, findPolysAroundCircle or findPolysAroundShape.
    pub fn get_path_from_dijkstra_search<const N: usize>(
        &self, end_ref: u32,
    ) -> Result<Vec<u32>, RcError> {
        let mut path = Vec::with_capacity(N);
        let mut path_count: U<_> = U::uninit();
        match RcError::make_error( unsafe { self.0.getPathFromDijkstraSearch(
            end_ref, path.as_mut_ptr(), path_count.assume_init_mut(), N as i32) }
        ) {
            Some(v) => Err(v),
            None => {
                unsafe { path.set_len(path_count.assume_init() as usize) }
                Ok(path)
            }
        }
    }

    /// `dtNavMeshQuery::findNearestPoly`
    /// 
    /// Finds the polygon nearest to the specified center point.
    /// Returns the reference ID of the nearest polygon, the nearest point on that polygon
    /// and a boolean that is true if the point's coordinate is inside of the polygon
    pub fn find_nearest_polygon(
        &self, 
        center: Vec3A, 
        half_extent: Vec3A, 
        filter: &QueryFilter
    ) -> Result<(u32, Vec3A, bool), RcError> {
        let center_raw = &raw const center as *const f32;
        let extent_raw = &raw const half_extent as *const f32;
        let mut near_ref: U<_> = U::uninit();
        let mut near_point: U<Vec3A> = U::uninit();
        let mut near_overpoly: U<_> = U::uninit();
        match RcError::make_error( unsafe { self.0.findNearestPoly1(
           center_raw, extent_raw, &raw const filter.0,
           near_ref.assume_init_mut(),
           near_point.as_mut_ptr() as *mut f32,
           near_overpoly.assume_init_mut()
        ) }
        ) {
            Some(v) => Err(v),
            None => Ok(unsafe { (near_ref.assume_init(), near_point.assume_init(), near_overpoly.assume_init() ) })
        }
    }

    /// `dtNavMeshQuery::queryPolygons`
    /// 
    /// Finds polygons that overlap the search box.
    pub fn query_polygons<const N: usize>(
        &self,
        center: Vec3A, 
        half_extent: Vec3A, 
        filter: &QueryFilter,
    ) -> Result<Vec<u32>, RcError> {
        let center_raw = &raw const center as *const f32;
        let extent_raw = &raw const half_extent as *const f32;
        let mut polys = Vec::with_capacity(N);
        let mut polys_len  = MaybeUninit::uninit();
        match RcError::make_error(unsafe { self.0.queryPolygons(
            center_raw as *const f32, extent_raw as *const f32, &raw const filter.0, 
            polys.as_ptr() as *mut u32, polys_len.assume_init_mut(), N as i32) }
        ) {
            Some(v) => Err(v),
            None => {
                unsafe { polys.set_len(polys_len.assume_init() as usize) }
                Ok(polys)
            }
        }
    }

    /// `dtNavMeshQuery::findRandomPoint`
    /// 
    /// Returns random location on navmesh.
    /// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
    pub fn find_random_point(&self, filter: &QueryFilter, rand: fn() -> f32) -> Result<(u32, Vec3A), RcError> {
        let frand = unsafe { std::mem::transmute::<_, Option<unsafe extern "C" fn() -> f32>>(Some(rand)) };
        let mut random_ref: U<_> = U::uninit();
        let mut random_point: U<Vec3A> = U::uninit();
        match RcError::make_error(unsafe { self.0.findRandomPoint(
            &raw const filter.0, frand, 
            random_ref.assume_init_mut(), 
            random_point.as_mut_ptr() as *mut f32
        ) }) {
            Some(v) => Err(v),
            None => Ok(unsafe { (random_ref.assume_init(), random_point.assume_init())})
        }
    }

    /// `dtNavMeshQuery::findRandomPointAroundCircle`
    /// 
    /// Returns random location on navmesh.
    /// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
    pub fn find_random_point_around_circle(
        &self,
        start_ref: u32,
        center: Vec3A,
        radius: f32,
        filter: &QueryFilter, 
        rand: fn() -> f32
    ) -> Result<(u32, Vec3A), RcError> {
        let center_raw = &raw const center as *const f32;
        let frand = unsafe { std::mem::transmute::<_, Option<unsafe extern "C" fn() -> f32>>(Some(rand)) };
        let mut random_ref: U<_> = U::uninit();
        let mut random_point: U<Vec3A> = U::uninit();
        match RcError::make_error(unsafe { self.0.findRandomPointAroundCircle(
            start_ref, center_raw, radius,
            &raw const filter.0, frand, 
            random_ref.assume_init_mut(), 
            random_point.as_mut_ptr() as *mut f32
        ) }) {
            Some(v) => Err(v),
            None => Ok(unsafe { (random_ref.assume_init(), random_point.assume_init())})
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

const MAX_TILE_COUNT: usize = 32;

impl NavMesh {
    /// `dtNavMesh::init`
    /// 
    /// Initializes the navigation mesh for tiled use.
    /// Returns an instance of NavMesh if initialization succeeds.
    pub fn new_tiled(params: &dtNavMeshParams) -> Result<Box<Self, DetourAllocator>, RcError> {
        let mut new = Box::new_in(Self(unsafe { dtNavMesh::new() }), DetourAllocator);
        match RcError::make_error(unsafe { new.as_mut().0.init(params) }) {
            Some(v) => Err(v),
            None => Ok(new)
        }
    }

    /// `dtNavMesh::init`
    /// 
    /// Initializes the navigation mesh for single tile use
    /// Returns an instance of NavMesh if initialization succeeds.
    pub fn new_single(data: &[u8], flags: TileFlags) -> Result<Box<Self, DetourAllocator>, RcError> {
        let mut new = Box::new_in(Self(unsafe { dtNavMesh::new() }), DetourAllocator);
        match RcError::make_error(unsafe { new.as_mut().0.init1(data.as_ptr() as *mut u8, data.len() as i32, flags.bits() as i32) }) {
            Some(v) => Err(v),
            None => Ok(new)
        }
    }

    /// `dtNavMesh::getParams`
    /// 
    /// The navigation mesh initialization params.
    pub fn get_params(&self) -> &dtNavMeshParams { unsafe { &*self.0.getParams() } }

    /// `dtNavMesh::addTile`
    /// 
    /// Adds a tile to the navigation mesh.
    /// Returns the tile reference if the operation succeeded
    pub fn add_tile(&mut self, data: &[u8], flags: TileFlags, last_ref: u32) -> Result<u32, RcError> {
        let mut result: U<u32> = U::uninit();
        match RcError::make_error(unsafe { self.0.addTile(
            data.as_ptr() as *mut u8, data.len() as i32, flags.bits() as i32, 
            last_ref, result.assume_init_mut()) }) {
            Some(v) => Err(v),
            None => Ok(unsafe { result.assume_init() }),
        }
    }

    /// `dtNavMesh::removeTile`
    /// 
    /// Removes the specified tile from the navigation mesh.
    /// Returns the data associated with the deleted tile if the operation succeeded
    pub fn remove_tile(&mut self, _ref: u32) -> Result<&[u8], RcError> {
        let mut data: U<*mut u8> = U::uninit();
        let mut data_size: U<i32> = U::uninit();
        match RcError::make_error(unsafe { self.0.removeTile(_ref, data.assume_init_mut(), data_size.assume_init_mut()) }) {
            Some(v) => Err(v),
            None => Ok(unsafe { std::slice::from_raw_parts(data.assume_init(), data_size.assume_init() as usize) })
        }
    }

    /// `dtNavMesh::calcTileLoc`
    /// Calculates the tile grid location for the specified world position.
    /// Given a world position (xyz), returns the tile's location (xy)
    pub fn calculate_tile_location(&self, pos: Vec3A) -> UVec2 {
        UVec2::new(
            ((pos.x - self.0.m_orig[0]) / self.0.m_tileWidth).floor() as u32,
            ((pos.z - self.0.m_orig[2]) / self.0.m_tileWidth).floor() as u32
        )
    }

    /// `dtNavMesh::getTileAt`
    /// 
    /// Gets the tile at the specified grid location
    /// Returns the tile if it exists
    pub fn get_tile_at(&self, pos: UVec2, layer: u32) -> Option<&dtMeshTile> {
        match unsafe { self.0.getTileAt(pos.x as i32, pos.y as i32, layer as i32) } {
            p if p != std::ptr::null() => Some(unsafe { &*p }),
            _ => None
        }
    }

    /// `dtNavMesh::getTilesAt`
    /// 
    /// Gets all tiles at the specified grid location (All layers)
    /// Returns a Vec containing all matching tiles
    pub fn get_tiles_at(&self, pos: UVec2) -> Vec<&dtMeshTile> {
        let mut result = Vec::with_capacity(MAX_TILE_COUNT);
        let count = unsafe { self.0.getTilesAt(pos.x as i32, pos.y as i32, result.as_mut_ptr(), MAX_TILE_COUNT as i32) } as usize;
        unsafe { 
            result.set_len(count); 
            std::mem::transmute::<_, Vec<&dtMeshTile>>(result)
        }
    }

    /// `dtNavMesh::getTileRefAt`
    /// 
    /// Gets the tile reference for the tile at specified grid location.
    pub fn get_tile_ref_at(&self, pos: UVec2, layer: u32) -> Option<u32> {
        match unsafe { self.0.getTileRefAt(pos.x as i32, pos.y as i32, layer as i32) } {
            0 => None,
            v => Some(v)
        }
    }

    /// `dtNavMesh::getTileRef`
    /// 
    /// Gets the tile reference for the specified tile.
    /// Returns the tile reference if it exists
    pub fn get_tile_ref(&self, tile: &dtMeshTile) -> Option<u32> {
        match unsafe { self.0.getTileRef(tile) } {
            0 => None,
            v => Some(v)
        }
    }

    /// `dtNavMesh::getTileByRef`
    /// 
    /// Gets the tile for the specified tile reference.
    /// Returns the tile if it exists
    pub fn get_tile_by_ref(&self, _ref: u32) -> Option<&dtMeshTile> {
        match unsafe { self.0.getTileByRef(_ref) } {
            p if p != std::ptr::null() => Some(unsafe { &*p }),
            _ => None
        }
    }

    /// `dtNavMesh::getMaxTiles`
    /// 
    /// The maximum number of tiles supported by the navigation mesh.
    pub fn get_max_tiles(&self) -> usize { self.0.m_maxTiles as usize }

    /// `dtNavMesh::getTile`
    /// 
    /// Gets the tile at the specified index.
    pub fn get_tile(&self, index: usize) -> Option<&dtMeshTile> {
        if index < self.get_max_tiles() {
            Some(unsafe { &*self.0.getTile(index as i32) })
        } else {
            None
        }
    }

    /// `dtNavMesh::decodePolyId`
    /// 
    /// Decodes a standard polygon reference.
    /// Returns the tile's salt value, the index of the tile and the index
    /// of the polygon within the tile
    pub fn decode_poly_id(&self, _ref: u32) -> (u32, u32, u32) {
        let salt_mask = (1 << self.0.m_saltBits as u32) - 1;
        let tile_mask = (1 << self.0.m_tileBits as u32) - 1;
        let poly_mask = (1 << self.0.m_polyBits as u32) - 1;
        (
            (_ref >> (self.0.m_polyBits + self.0.m_tileBits) as u32) & salt_mask,
            (_ref >> self.0.m_polyBits as u32) & tile_mask,
            _ref & poly_mask
        )
    }
}