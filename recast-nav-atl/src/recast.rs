use glam::{ UVec2, Vec3 };

pub struct Recast;
impl Recast {
    /// `rcCalcBounds`
    pub fn calc_bounds(verts: &[Vec3]) -> (Vec3, Vec3) {
        let mut min = Vec3::MAX;
        let mut max = Vec3::MIN;
        for v in verts {
            min = v.min(min);
            max = v.min(max);
        }
        (min, max)
    }

    /// `rcCalcGridSize`
    pub fn calc_grid_size(min: Vec3, max: Vec3, cell_size: f32) -> UVec2 {
        UVec2::new(
            ((max.x - min.x) / cell_size + 0.5) as u32,
            ((max.z - min.z) / cell_size + 0.5) as u32,
        )
    }
}