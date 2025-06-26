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