use bindgen::builder;
// use std::path::PathBuf;

fn main() {
    let current_dir = std::env::current_dir().expect("Could not read current directory");
    let native_dir = current_dir.parent().unwrap().join("include/recastnavigation");

    let include_detour = format!("-I{}", native_dir.join("Detour").join("Include").to_str().unwrap());

    let bindings = builder()
        .headers([
            native_dir.join("Detour").join("Include").join("DetourNavMesh.h").to_str().unwrap(),
            native_dir.join("DetourCrowd").join("Include").join("DetourCrowd.h").to_str().unwrap(),
            native_dir.join("DetourTileCache").join("Include").join("DetourTileCache.h").to_str().unwrap(),
        ])
        
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .enable_cxx_namespaces()
        .clang_args([
            "-x", "c++", "-std=c++14", // enable C++ stuff
            &include_detour
        ])
        // .layout_tests(false)
        .generate()
        .expect("Could not generate bindings");
    bindings.write_to_file(current_dir.join("src/bindings_raw.rs")).unwrap();
}