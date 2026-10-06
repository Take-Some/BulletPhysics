#[derive(Default, Debug)]
pub enum Mode {
    #[default]
    SharedMemory,
    Direct,
    Gui,
    Udp {
        hostname: &'static str,
        port: Option<u16>,
    },
    Tcp {
        hostname: &'static str,
        port: Option<u16>,
    },
    GuiServer,
    GuiMainThread,
    SharedMemoryServer,
    SharedMemoryGui,
    GraphicsClient,
    GraphicsServer {
        port: Option<u16>,
    },
    GraphicsServerTcp {
        hostname: &'static str,
        port: Option<u16>,
    },
    GraphicsServerMainThread {
        port: Option<u16>,
    },

    #[cfg(feature = "dart")]
    Dart,
    #[cfg(feature = "physx")]
    PhysX,
    #[cfg(feature = "mujoco")]
    MuJoCo,
    #[cfg(feature = "grpc")]
    Grpc {
        hostname: &'static str,
        port: Option<i32>,
    },
}
