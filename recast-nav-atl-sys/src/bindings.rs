#[cfg(feature = "serialize")]
use rkyv::{ 
    Archive,
    Serialize,
    rancor::Fallible
};

include!("bindings_raw.rs");

#[cfg(feature = "serialize")]
#[allow(non_snake_case, non_camel_case_types, non_upper_case_globals)]
pub mod serialization {
    use rkyv::{ 
        Archive, 
        Deserialize, 
        Portable, 
        rancor::{
            Fallible,
            Source
        }
    };

    // dtNavMeshParams

    #[repr(C)]
    #[derive(Portable)]
    pub struct Archived_dtNavMeshParams {
        orig: [<f32 as Archive>::Archived; 3],
        tileWidth: <f32 as Archive>::Archived,
        tileHeight: <f32 as Archive>::Archived,
        maxTiles: <i32 as Archive>::Archived,
        maxPolys: <i32 as Archive>::Archived,
    }

    impl<D> Deserialize<super::root::dtNavMeshParams, D> for Archived_dtNavMeshParams
    where D: rkyv::rancor::Fallible + ?Sized,
        D::Error: Source
    {
        fn deserialize(&self, deserializer: &mut D) -> Result<super::root::dtNavMeshParams, <D as Fallible>::Error> {
            Ok(super::root::dtNavMeshParams {
                orig: self.orig.deserialize(deserializer)?,
                tileWidth: self.tileWidth.deserialize(deserializer)?,
                tileHeight: self.tileHeight.deserialize(deserializer)?,
                maxTiles: self.maxTiles.deserialize(deserializer)?,
                maxPolys: self.maxPolys.deserialize(deserializer)?,
            })
        }
    }

    // dtTileCacheParams

    #[repr(C)]
    #[derive(Portable)]
    pub struct Archived_dtTileCacheParams {
        orig: [<f32 as Archive>::Archived; 3usize],
        cs: <f32 as Archive>::Archived,
        ch: <f32 as Archive>::Archived,
        width: <i32 as Archive>::Archived,
        height: <i32 as Archive>::Archived,
        walkableHeight: <f32 as Archive>::Archived,
        walkableRadius: <f32 as Archive>::Archived,
        walkableClimb: <f32 as Archive>::Archived,
        maxSimplificationError: <f32 as Archive>::Archived,
        maxTiles: <i32 as Archive>::Archived,
        maxObstacles: <i32 as Archive>::Archived,
    }

    impl<D> Deserialize<super::root::dtTileCacheParams, D> for Archived_dtTileCacheParams
    where D: rkyv::rancor::Fallible + ?Sized,
        D::Error: Source
    {
        fn deserialize(&self, deserializer: &mut D) -> Result<super::root::dtTileCacheParams, <D as Fallible>::Error> {
            Ok(super::root::dtTileCacheParams {
                orig: self.orig.deserialize(deserializer)?,
                cs: self.cs.deserialize(deserializer)?,
                ch: self.ch.deserialize(deserializer)?,
                width: self.width.deserialize(deserializer)?,
                height: self.height.deserialize(deserializer)?,
                walkableHeight: self.walkableHeight.deserialize(deserializer)?,
                walkableRadius: self.walkableRadius.deserialize(deserializer)?,
                walkableClimb: self.walkableClimb.deserialize(deserializer)?,
                maxSimplificationError: self.maxSimplificationError.deserialize(deserializer)?,
                maxTiles: self.maxTiles.deserialize(deserializer)?,
                maxObstacles: self.maxObstacles.deserialize(deserializer)?,
            })
        }
    }
}

// dtNavMeshParams

#[cfg(feature = "serialize")]
impl Archive for root::dtNavMeshParams {
    type Archived = serialization::Archived_dtNavMeshParams;
    type Resolver = ();

    fn resolve(&self, _: Self::Resolver, _: rkyv::Place<Self::Archived>) {}
}

#[cfg(feature = "serialize")]
impl<S> Serialize<S> for root::dtNavMeshParams 
where S: Fallible {
    fn serialize(&self, _: &mut S)
        -> Result<Self::Resolver, <S as Fallible>::Error> { Ok(()) }
}

// dtTileCacheParams

#[cfg(feature = "serialize")]
impl Archive for root::dtTileCacheParams {
    type Archived = serialization::Archived_dtTileCacheParams;
    type Resolver = ();

    fn resolve(&self, _: Self::Resolver, _: rkyv::Place<Self::Archived>) {}
}

#[cfg(feature = "serialize")]
impl<S> Serialize<S> for root::dtTileCacheParams 
where S: Fallible {
    fn serialize(&self, _: &mut S)
        -> Result<Self::Resolver, <S as Fallible>::Error> { Ok(()) }
}