use allocator_api2::boxed::Box;
use crate::{
    allocator::DetourAllocator,
    agent::Agent,
    error::Error as RcError,
    nav_mesh::{
        NavMesh,
        NavMeshQuery,
    }
};
use glam::Vec3A;
use recast_nav_atl_sys::bindings::root::{
    dtCrowd,
    dtCrowdAgentParams,
    dtCrowdAgentDebugInfo,
    dtObstacleAvoidanceParams,
    dtQueryFilter
};

#[allow(dead_code)]
pub struct QueryFilter(pub(crate) dtQueryFilter);

impl From<&dtQueryFilter> for &QueryFilter {
    fn from(value: &dtQueryFilter) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtQueryFilter> for &mut QueryFilter {
    fn from(value: &mut dtQueryFilter) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtQueryFilter> for &QueryFilter {
    fn from(value: &mut dtQueryFilter) -> Self { unsafe { std::mem::transmute(value) } }
}

pub struct Crowd(pub(crate) dtCrowd);

impl From<&dtCrowd> for &Crowd {
    fn from(value: &dtCrowd) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtCrowd> for &mut Crowd {
    fn from(value: &mut dtCrowd) -> Self { unsafe { std::mem::transmute(value) } }
}

impl From<&mut dtCrowd> for &Crowd {
    fn from(value: &mut dtCrowd) -> Self { unsafe { std::mem::transmute(value) } }
}

impl Crowd {

    /// `dtCrowd::init`
    pub fn new(max_agents: usize, max_agent_radius: f32, nav: &NavMesh) -> Result<Box<Self, DetourAllocator>, RcError> {
        let mut new = Box::new_in(Self(unsafe { dtCrowd::new() }), DetourAllocator);
        match unsafe { new.as_mut().0.init(max_agents as i32, max_agent_radius, &raw const nav.0 as *mut _) } {
            true => Ok(new),
            false => Err(RcError::empty())
        }
    }

    /// `dtCrowd::setObstacleAvoidanceParams`
    /// 
    /// Sets the shared avoidance configuration for the specified index.
    pub fn set_obstacle_avoidance_params(&mut self, index: usize, params: &dtObstacleAvoidanceParams) {
        unsafe { self.0.setObstacleAvoidanceParams(index as i32, params) }
    }

    /// `dtCrowd::getObstacleAvoidanceParams`
    /// 
    /// Gets the shared avoidance configuration for the specified index.
    /// Returns the requested configuration
    pub fn get_obstacle_avoidance_params(&self, index: usize) -> Option<&dtObstacleAvoidanceParams> {
        if index < recast_nav_atl_sys::bindings::root::DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS as usize {
            Some(&self.0.m_obstacleQueryParams[index])
        } else {
            None
        }
    }

    /// `dtCrowd::getAgent`
    /// 
    /// Gets the specified agent from the pool.
    /// Returns the requested agent
    pub fn get_agent(&self, index: usize) -> Option<&Agent> {
        if index < self.0.m_maxAgents as usize {
            Some((unsafe { &*self.0.m_agents.add(index) }).into())
        } else {
            None
        }
    }

    /// `dtCrowd::getEditableAgent`
    /// 
    /// Gets the specified agent from the pool.
    /// Returns the requested agent
    pub fn get_agent_mut(&mut self, index: usize) -> Option<&mut Agent> {
        if index < self.0.m_maxAgents as usize {
            Some((unsafe { &mut *self.0.m_agents.add(index) }).into())
        } else {
            None
        }
    }

    /// `dtCrowd::getAgentCount`
    /// 
    /// The maximum number of agents that can be managed by the object.
	/// Returns the maximum number of agents.
    pub fn get_agent_count(&self) -> usize { self.0.m_maxAgents as usize }

    /// `dtCrowd::addAgent`
    /// 
    /// Adds a new agent to the crowd.
	/// Returns a handle to the new agent if it succeeded
    pub fn add_agent(&mut self, pos: Vec3A, params: &dtCrowdAgentParams) -> Option<&mut Agent> {
        let pos_raw = &raw const pos as *const f32;
        
        match unsafe { self.0.addAgent(pos_raw, params) } {
            -1 => None,
            i => self.get_agent_mut(i as usize)
        }
    }

    /// `dtCrowd::updateAgentParameters`
    /// 
    /// Updates the specified agent's configuration.
    pub fn update_agent_parameters(&mut self, index: usize, params: &dtCrowdAgentParams) {
        unsafe { self.0.updateAgentParameters(index as i32, params) }
    }

    /// `dtCrowd::removeAgent`
    /// 
    /// Removes the agent from the crowd.
    pub fn remove_agent(&mut self, index: usize) {
        unsafe { self.0.removeAgent(index as i32) }
    }

    /// `dtCrowd::requestMoveTarget`
    /// 
    /// Submits a new move request for the specified agent.
    /// Returns true if the request was successfully submitted.
    pub fn request_move_target(&mut self, index: usize, poly_ref: usize, pos: Vec3A) -> bool {
        let pos_raw = &raw const pos as *const f32;
        unsafe { self.0.requestMoveTarget(index as i32, poly_ref as u32, pos_raw) }
    }

    /// `dtCrowd::requestMoveVelocity`
    /// 
    /// Submits a new move request for the specified agent.
    /// Returns true if the request was successfully submitted.
    pub fn request_move_velocity(&mut self, index: usize, vel: Vec3A) -> bool {
        let vel_raw = &raw const vel as *const f32;
        unsafe { self.0.requestMoveVelocity(index as i32, vel_raw) }
    }

    /// `dtCrowd::resetMoveTarget`
    /// 
    /// Resets any request for the specified agent.
    pub fn reset_move_target(&mut self, index: usize) -> bool {
        unsafe { self.0.resetMoveTarget(index as i32) }
    }

    /// `dtCrowd::getActiveAgents`
    /// 
    /// Gets the active agents int the agent pool.
    pub fn get_active_agents(&mut self) -> Vec<&mut Agent> {
        let cap = self.0.m_maxAgents as usize;
        let mut values = Vec::with_capacity(cap);
        let len = unsafe { self.0.getActiveAgents(values.as_mut_ptr(), cap as i32) } as usize;
        unsafe { values.set_len(len) }
        unsafe { std::mem::transmute::<_, Vec<&mut Agent>>(values) }
    }

    /// `dtCrowd::update`
    /// 
    /// Updates the steering and positions of all agents.
    pub fn update(&mut self, dt: f32, debug: Option<&dtCrowdAgentDebugInfo>) {
        let value = match debug {
            Some(v) => &raw const *v as *mut dtCrowdAgentDebugInfo,
            None => std::ptr::null_mut()
        };
        unsafe { self.0.update(dt, value) }
    }

    /// `dtCrowd::getFilter`
    /// 
    /// Gets the filter used by the crowd.
	/// Returns the filter used by the crowd.
    pub fn get_filter(&self, index: usize) -> Option<&QueryFilter> {
        if index < recast_nav_atl_sys::bindings::root::DT_CROWD_MAX_QUERY_FILTER_TYPE as usize {
            Some((&self.0.m_filters[index]).into())
        } else {
            None
        }
    }

    /// `dtCrowd::getEditableFilter`
    /// 
	/// Gets the filter used by the crowd.
	/// Returns the filter used by the crowd.
    pub fn get_filter_mut(&mut self, index: usize) -> Option<&mut QueryFilter> {
        if index < recast_nav_atl_sys::bindings::root::DT_CROWD_MAX_QUERY_FILTER_TYPE as usize {
            Some((&mut self.0.m_filters[index]).into())
        } else {
            None
        }
    }

    /// `dtCrowd::getQueryHalfExtents`
    /// 
	/// Gets the search halfExtents [(x, y, z)] used by the crowd for query operations. 
	/// Returns the search halfExtents used by the crowd. [(x, y, z)]
    pub fn get_query_half_extents(&self) -> [f32; 3] { self.0.m_agentPlacementHalfExtents }

    /// `dtCrowd::getQueryExtents`
    /// 
	/// Same as getQueryHalfExtents. Left to maintain backwards compatibility.
	/// Returns the search halfExtents used by the crowd. [(x, y, z)]
    pub fn get_query_extents(&self) -> [f32; 3] { self.0.m_agentPlacementHalfExtents }

    /// `dtCrowd::getVelocitySampleCount`
    /// 
	/// Gets the velocity sample count.
	/// Returns the velocity sample count.
    pub fn get_velocity_sample_count(&self) -> i32 { self.0.m_velocitySampleCount }

    /// `dtCrowd::getNavMeshQuery`
    /// 
    /// Gets the query object used by the crowd.
    pub fn get_nav_mesh_query(&self) -> Option<&NavMeshQuery> {
        if self.0.m_navquery != std::ptr::null_mut() {
            Some(unsafe { (&*self.0.m_navquery).into() })
        } else {
            None
        }
    }
}

impl Drop for Crowd {
    fn drop(&mut self) {
        unsafe { self.0.destruct() }
    }
}