add_plugin(IsaacSim
  IsaacSimPlugin.cpp
  IsaacSimPlugin.h
  Markers.cpp
  Markers.h
  GuiMirror.cpp
  GuiMirror.h
  Bridge.cpp
  Bridge.h
  Net.cpp
  Net.h
  Sha256.cpp
  Sha256.h
)
# in-process mc_rtc GUI client (3D markers in the Isaac viewport)
target_link_libraries(IsaacSim PUBLIC mc_rtc::mc_control_client)
# default location of Isaac descriptions (ground.yaml and descriptions installed alongside mc_isaac)
target_compile_definitions(IsaacSim PRIVATE MC_ISAAC_SHARE_DIR="${CMAKE_INSTALL_PREFIX}/share/mc_isaac"
                                           MC_RTC_ISAAC_SHARE_DIR="${MC_RTC_INSTALL_PREFIX}/share/mc_isaac"
                                           MC_ISAAC_BIN_DIR="${CMAKE_INSTALL_PREFIX}/bin")
