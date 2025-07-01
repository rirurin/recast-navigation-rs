use glam::Vec3;
use recast_nav_atl_sys::bindings::root::{
    dtNavMeshParams,
    dtTileCacheParams,
};

#[cfg(feature = "serialize")]
use rkyv::{ 
    Archive,
    Deserialize,
    Portable,
    Serialize,
    rancor::{ Fallible, Source },
    bytecheck::CheckBytes,
};

pub const TILECACHESET_MAGIC: u32 = ('T' as u32) << 24 | ('S' as u32) << 16 | ('E' as u32) << 8 | 'T' as u32;
pub const TILECACHESET_VERSION: u32 = 1;
pub const TILECACHESET_INCLUDE_OFFMESH: u32 = 2;
pub const TILECACHESET_ATL_VERSION: u32 = 3;

#[derive(Debug, Clone)]
#[repr(C)]
#[cfg_attr(feature = "serialize", derive(Archive, Deserialize, Serialize))]
pub struct TileCacheSetHeader {
    magic: u32,
    version: u32,
    num_tiles: u32,
    mesh_params: dtNavMeshParams,
    cache_params: dtTileCacheParams,
}

impl TileCacheSetHeader {
    pub fn get_tile_len(&self) -> usize { self.num_tiles as usize }
    pub fn get_mesh_params(&self) -> &dtNavMeshParams { &self.mesh_params }
    pub fn get_cache_params(&self) -> &dtTileCacheParams { &self.cache_params }
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct TileCacheSetHeaderBase {
    pub _super: TileCacheSetHeader,
    pub max_bound: Vec3,
    pub min_bound: Vec3
}

#[repr(C)]
#[derive(Portable)]
#[cfg(feature = "serialize")]
pub struct ArchivedTileCacheSetHeaderBase {
    _super: <TileCacheSetHeader as Archive>::Archived,
    max_bound: [<f32 as Archive>::Archived; 3],
    min_bound: [<f32 as Archive>::Archived; 3],
}

#[cfg(feature = "serialize")]
impl Archive for TileCacheSetHeaderBase {
    type Archived = ArchivedTileCacheSetHeaderBase;
    type Resolver = ();
    fn resolve(&self, _: Self::Resolver, _: rkyv::Place<Self::Archived>) {}
}

#[cfg(feature = "serialize")]
impl<S> Serialize<S> for TileCacheSetHeaderBase 
where S: Fallible + ?Sized,
    S::Error: Source
{
    fn serialize(&self, _: &mut S) -> Result<Self::Resolver, S::Error> { Ok(()) }
}

#[cfg(feature = "serialize")]
impl<D> Deserialize<TileCacheSetHeaderBase, D> for <TileCacheSetHeaderBase as Archive>::Archived
where D: Fallible + ?Sized,
    D::Error: Source
{
    fn deserialize(&self, deserializer: &mut D) -> Result<TileCacheSetHeaderBase, <D as Fallible>::Error> {
        Ok(TileCacheSetHeaderBase {
            _super: self._super.deserialize(deserializer)?,
            max_bound: self.max_bound.deserialize(deserializer)?.into(),
            min_bound: self.min_bound.deserialize(deserializer)?.into(),
        })
    }
}

#[cfg(feature = "serialize")]
unsafe impl<C> CheckBytes<C> for ArchivedTileCacheSetHeaderBase
where C: Fallible + ?Sized,
    C::Error: Source
{ unsafe fn check_bytes(_: *const Self, _: &mut C) -> Result<(), <C as Fallible>::Error> { Ok(()) } }

#[derive(Debug, Clone)]
#[repr(C)]
pub struct TileCacheSetHeaderAtl {
    pub _super: TileCacheSetHeader,
    pub agent: AgentSettings,
    pub max_bound: Vec3,
    pub min_bound: Vec3
}

#[repr(C)]
#[derive(Portable)]
#[cfg(feature = "serialize")]
pub struct ArchivedTileCacheSetHeaderAtl {
    _super: <TileCacheSetHeader as Archive>::Archived,
    agent: <AgentSettings as Archive>::Archived,
    max_bound: [<f32 as Archive>::Archived; 3],
    min_bound: [<f32 as Archive>::Archived; 3],
}

#[cfg(feature = "serialize")]
impl Archive for TileCacheSetHeaderAtl {
    type Archived = ArchivedTileCacheSetHeaderAtl;
    type Resolver = ();
    fn resolve(&self, _: Self::Resolver, _: rkyv::Place<Self::Archived>) {}
}

#[cfg(feature = "serialize")]
impl<S> Serialize<S> for TileCacheSetHeaderAtl 
where S: Fallible + ?Sized,
    S::Error: Source
{
    fn serialize(&self, _: &mut S) -> Result<Self::Resolver, S::Error> { Ok(()) }
}

#[cfg(feature = "serialize")]
impl<D> Deserialize<TileCacheSetHeaderAtl, D> for <TileCacheSetHeaderAtl as Archive>::Archived
where D: Fallible + ?Sized,
    D::Error: Source
{
    fn deserialize(&self, deserializer: &mut D) -> Result<TileCacheSetHeaderAtl, <D as Fallible>::Error> {
        Ok(TileCacheSetHeaderAtl {
            _super: self._super.deserialize(deserializer)?,
            agent: self.agent.deserialize(deserializer)?,
            max_bound: self.max_bound.deserialize(deserializer)?.into(),
            min_bound: self.min_bound.deserialize(deserializer)?.into(),
        })
    }
}

#[cfg(feature = "serialize")]
unsafe impl<C> CheckBytes<C> for ArchivedTileCacheSetHeaderAtl
where C: Fallible + ?Sized,
    C::Error: Source
{ unsafe fn check_bytes(_: *const Self, _: &mut C) -> Result<(), <C as Fallible>::Error> { Ok(()) } }

#[derive(Debug, Clone)]
#[repr(C)]
#[cfg_attr(feature = "serialize", derive(Archive, Deserialize, Serialize))]
pub struct AgentSettings {
	// Cell size in world units
	cell_size: f32,
	// Cell height in world units
	cell_height: f32,
	// Agent height in world units
	agent_height: f32,
	// Agent radius in world units
	agent_radius: f32,
	// Agent max climb in world units
	agent_max_climb: f32,
	// Agent max slope in degrees
	agent_max_slope: f32,
	// Region minimum size in voxels.
	// regionMinSize = sqrt(regionMinArea)
	region_min_size: f32,
	// Region merge size in voxels.
	// regionMergeSize = sqrt(regionMergeArea)
	region_merge_size: f32,
	// Edge max length in world units
	edge_max_len: f32,
	// Edge max error in voxels
	edge_max_error: f32,
	verts_per_poly: f32,
	// Detail sample distance in voxels
	detail_sample_dist: f32,
	// Detail sample max error in voxel heights.
	detail_sample_max_error: f32,
	// Partition type, see SamplePartitionType
	partition_type: u32,
    // Size of the tiles in voxels
    tile_size: u32,
}

impl Default for AgentSettings {
    // Original function: 0x1413d6a10 (Metaphor: Refantazio, Steam Prologue Demo 1.01)
    fn default() -> Self {
        Self {
            cell_size: 30.,
            cell_height: 20.,
            agent_height: 80.,
            agent_radius: 20.,
            agent_max_climb: 90.,
            agent_max_slope: 45.,
            region_min_size: 8.,
            region_merge_size: 20.,
            edge_max_len: 12.,
            edge_max_error: 1.3,
            verts_per_poly: 6.,
            detail_sample_dist: 6.,
            detail_sample_max_error: 1.,
            partition_type: 0,
            tile_size: 16
        }
    }
}

impl AgentSettings {
    pub fn get_cell_size(&self) -> f32 {
        self.cell_size
    }
    pub fn set_cell_size(&mut self, value: f32) {
        self.cell_size = value
    }
    pub fn get_cell_height(&self) -> f32 {
        self.cell_height
    }
    pub fn set_cell_height(&mut self, value: f32) {
        self.cell_height = value
    }
    pub fn get_agent_height(&self) -> f32 {
        self.agent_height
    }
    pub fn set_agent_height(&mut self, value: f32) {
        self.agent_height = value
    }
    pub fn get_agent_radius(&self) -> f32 {
        self.agent_radius
    }
    pub fn set_agent_radius(&mut self, value: f32) {
        self.agent_radius = value
    }
    pub fn get_agent_max_climb(&self) -> f32 {
        self.agent_max_climb
    }
    pub fn set_agent_max_climb(&mut self, value: f32) {
        self.agent_max_climb = value
    }
    pub fn get_agent_max_slope(&self) -> f32 {
        self.agent_max_slope
    }
    pub fn set_agent_max_slope(&mut self, value: f32) {
        self.agent_max_slope = value
    }
    pub fn get_region_min_size(&self) -> f32 {
        self.region_min_size
    }
    pub fn set_region_min_size(&mut self, value: f32) {
        self.region_min_size = value
    }
    pub fn get_region_merge_size(&self) -> f32 {
        self.region_merge_size
    }
    pub fn set_region_merge_size(&mut self, value: f32) {
        self.region_merge_size = value
    }
    pub fn get_edge_max_len(&self) -> f32 {
        self.edge_max_len
    }
    pub fn set_edge_max_len(&mut self, value: f32) {
        self.edge_max_len = value
    }
    pub fn get_edge_max_error(&self) -> f32 {
        self.edge_max_error
    }
    pub fn set_edge_max_error(&mut self, value: f32) {
        self.edge_max_error = value
    }
    pub fn get_verts_per_poly(&self) -> f32 {
        self.verts_per_poly
    }
    pub fn set_verts_per_poly(&mut self, value: f32) {
        self.verts_per_poly = value
    }
    pub fn get_detail_sample_dist(&self) -> f32 {
        self.detail_sample_dist
    }
    pub fn set_detail_sample_dist(&mut self, value: f32) {
        self.detail_sample_dist = value
    }
    pub fn get_detail_sample_max_error(&self) -> f32 {
        self.detail_sample_max_error
    }
    pub fn set_detail_sample_max_error(&mut self, value: f32) {
        self.detail_sample_max_error = value
    }
    pub fn get_partition_type(&self) -> u32 {
        self.partition_type
    }
    pub fn set_partition_type(&mut self, value: u32) {
        self.partition_type = value
    }
    pub fn get_tile_size(&self) -> u32 {
        self.tile_size
    }
    pub fn set_tile_size(&mut self, value: u32) {
        self.tile_size = value
    }
}

#[repr(C)]
#[derive(Debug)]
#[cfg_attr(feature = "serialize", derive(Archive, Deserialize, Serialize))]
pub struct TileHeader {
    tile_ref: u32,
    data_size: u32
}

impl TileHeader {
    pub fn get_tile_ref(&self) -> u32 { self.tile_ref }
    pub fn get_data_size(&self) -> u32 { self.data_size }
}