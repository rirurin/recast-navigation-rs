use glam::Vec3;
use recast_nav_atl_sys::bindings::root::{
    dtNavMeshParams,
    dtTileCacheParams,
};

const TILECACHESET_MAGIC: u32 = ('T' as u32) << 24 | ('S' as u32) << 16 | ('E' as u32) << 8 | 'T' as u32;
const TILECACHESET_VERSION: u32 = 1;
const TILECACHESET_ATL_VERSION: u32 = 3;

#[derive(Debug)]
pub struct TileCacheSetHeader {
    magic: u32,
    version: u32,
    num_tiles: u32,
    mesh_params: dtNavMeshParams,
    cache_params: dtTileCacheParams,
}

#[derive(Debug)]
pub struct TileCacheSetHeaderAtl {
    _super: TileCacheSetHeader,
    geometry: GeometrySettings
}

#[derive(Debug)]
pub struct GeometrySettings {
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
    // Maximum bound
    bound_max: Vec3,
    // Minimum bound
    bound_min: Vec3
}

impl GeometrySettings {
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
    pub fn get_bound_max(&self) -> Vec3 { self.bound_max }
    pub fn set_bound_max(&mut self, value: Vec3) { self.bound_max = value }
    pub fn get_bound_min(&self) -> Vec3 { self.bound_min }
    pub fn set_bound_min(&mut self, value: Vec3) { self.bound_min = value }
}