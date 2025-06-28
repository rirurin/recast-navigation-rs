use bitflags::bitflags;
use glam::Vec3A;
use recast_nav_atl_sys::bindings::root::dtCrowdAgent;

#[allow(dead_code)]
pub struct Agent(dtCrowdAgent);

impl From<&dtCrowdAgent> for &Agent {
    fn from(value: &dtCrowdAgent) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtCrowdAgent> for &mut Agent {
    fn from(value: &mut dtCrowdAgent) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtCrowdAgent> for &Agent {
    fn from(value: &mut dtCrowdAgent) -> Self { unsafe { std::mem::transmute(value) } }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AgentState {
    Invalid,
    Walking,
    OffMesh,
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    pub struct UpdateFlags : u8 {
	    const ANTICIPATE_TURNS = 1;
	    const OBSTACLE_AVOIDANCE = 2;
	    const SEPARATION = 4;
	    const OPTIMIZE_VIS = 8;			///< Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
	    const OPTIMIZE_TOPO = 16;		///< Use dtPathCorridor::optimizePathTopology() to optimize the agent path.
        const AGENT_MOVING = 32;
    }
}

impl From<u8> for AgentState {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Invalid,
            1 => Self::Walking,
            2 => Self::OffMesh,
            _ => panic!("Invalid crowd agent state {}", value)
        }
    }
}

impl Agent {
    pub fn is_active(&self) -> bool { self.0.active }
    pub fn set_active(&mut self, value: bool) { self.0.active = value }

    pub fn get_state(&self) -> AgentState { self.0.state.into() }
    pub fn set_state(&mut self, state: AgentState) { self.0.state = state as u8 }

    pub fn get_update_flags(&self) -> UpdateFlags { UpdateFlags::from_bits_retain(self.0.params.updateFlags) }
    pub fn set_update_flags(&mut self, value: UpdateFlags) { self.0.params.updateFlags = value.bits() }

    pub fn get_current_position(&self) -> Vec3A { self.0.npos.into() }
    pub fn set_current_position(&mut self, value: Vec3A) { self.0.npos = value.into() }

    pub fn get_target_velocity(&self) -> Vec3A { self.0.dvel.into() }
    pub fn set_target_velocity(&mut self, value: Vec3A) { self.0.dvel = value.into() }

    pub fn get_current_velocity(&self) -> Vec3A { self.0.vel.into() }
    pub fn set_current_velocity(&mut self, value: Vec3A) { self.0.vel = value.into() }

    pub fn get_target_position(&self) -> Vec3A { self.0.targetPos.into() }
    pub fn set_target_position(&mut self, value: Vec3A) { self.0.targetPos = value.into() }
}