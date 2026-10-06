use std::path::{Path, PathBuf};

fn lib_exists(dir: &Path, name: &str) -> bool {
    // Windows static: Foo.lib；Unix static: libFoo.a
    let win = dir.join(format!("{name}.lib"));
    let unix = dir.join(format!("lib{name}.a"));
    win.exists() || unix.exists()
}

fn main() {
    println!("cargo:rerun-if-changed=bullet3/examples/SharedMemory/PhysicsServerCommandProcessor.cpp");
    println!("cargo:rerun-if-changed=build.rs");
    let mut cfg = cmake::Config::new("bullet3");
    cfg.profile("Release")
        .define("BUILD_SHARED_LIBS", "OFF")
        .define("BUILD_BULLET2_DEMOS", "OFF")
        .define("BUILD_CPU_DEMOS", "OFF")
        .define("BUILD_OPENGL3_DEMOS", "OFF")
        .define("BUILD_EXTRAS", "ON")
        .define("BUILD_UNIT_TESTS", "OFF")
        .define("BUILD_PYBULLET", "OFF");

    // cfg.generator("Ninja");

    // cfg.define("CMAKE_MSVC_RUNTIME_LIBRARY", "MultiThreaded$<$<CONFIG:Debug>:Debug>");

    let dst = cfg.build();
    let mut candidates: Vec<PathBuf> = vec![dst.join("lib")];
    candidates.push(dst.join("build").join("lib"));
    candidates.push(dst.join("build").join("lib").join("Release"));
    candidates.push(dst.join("build").join("lib").join("RelWithDebInfo"));
    candidates.push(dst.join("build").join("lib").join("Debug"));

    let libdir = candidates
        .into_iter()
        .find(|dir| lib_exists(dir, "LinearMath"))
        .unwrap_or_else(|| {
            panic!(
                "failed to locate built Bullet libraries under {}",
                dst.display()
            )
        });

    println!("cargo:rustc-link-search=native={}", libdir.display());

    for &lib in &[
        "BulletSoftBody",
        "BulletDynamics",
        "BulletCollision",
        "LinearMath",
        "BulletInverseDynamicsUtils",
        "BulletInverseDynamics",
        "Bullet3Common",
        "Bullet3Collision",
        "Bullet3Dynamics",
        "Bullet3Geometry",
        "BulletFileLoader",
        "BulletWorldImporter",
        "BulletRobotics",
        "Bullet3OpenCL_clew",
        "cbullet",
    ] {
        if lib_exists(&libdir, lib) {
            println!("cargo:rustc-link-lib=static={lib}");
        }
    }

    #[cfg(all(not(target_env = "msvc"), not(target_os = "macos")))]
    println!("cargo:rustc-link-lib=dylib=stdc++");

    #[cfg(target_os = "macos")]
    println!("cargo:rustc-link-lib=c++");

    #[cfg(target_os = "windows")]
    {
        println!("cargo:rustc-link-lib=dylib=User32");
        println!("cargo:rustc-link-lib=dylib=Gdi32");
        println!("cargo:rustc-link-lib=dylib=Opengl32");
        println!("cargo:rustc-link-lib=dylib=Comdlg32");
        println!("cargo:rustc-link-lib=dylib=winmm");
        println!("cargo:rustc-link-lib=dylib=ws2_32");
    }

}
