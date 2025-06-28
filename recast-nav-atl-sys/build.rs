use bindgen::builder;
use std::path::Path;
use walkdir::WalkDir;

fn get_files_by_filetype<P>(path: P, ext: &'static str) -> impl Iterator<Item = String> where P: AsRef<Path> {
    WalkDir::new(path.as_ref()).into_iter()
        .filter_map(move |v| v.ok().filter(|p| 
            p.file_type().is_file() && 
            p.path().extension().unwrap().to_str().unwrap() == ext)).map(|p| p.path().to_str().unwrap().to_owned())
}

fn get_source_files_in_directory<P>(path: P) -> impl Iterator<Item = String> where P: AsRef<Path> {
    get_files_by_filetype(path, "cpp")
}

fn main() {
    let current_dir = std::env::current_dir().expect("Could not read current directory");
    let native_dir = current_dir.parent().unwrap().join("include/recastnavigation");

    let include_detour = format!("-I{}", native_dir.join("Detour").join("Include").to_str().unwrap());

    // Create bindgen

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

    // Build recast-navigation (adapted from CMake build script)

    let mut builder = cc::Build::new();
    builder.cpp(true);

    let mut source_files: Vec<String> = vec![];
    source_files.extend(get_source_files_in_directory(native_dir.join("Detour").join("Source").as_path()));
    source_files.extend(get_source_files_in_directory(native_dir.join("DetourCrowd").join("Source").as_path()));
    source_files.extend(get_source_files_in_directory(native_dir.join("DetourTileCache").join("Source").as_path()));
    builder.files(source_files);

    builder.includes([
        native_dir.join("Detour").join("Include").to_str().unwrap(),
        native_dir.join("DetourCrowd").join("Include").to_str().unwrap(),
        native_dir.join("DetourTileCache").join("Include").to_str().unwrap(),
    ]);

    builder.define("SOVERSION", "1");
    builder.define("LIB_VERSION", "1.6.0");

    builder.compile("recast-navigation");
}