cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

project(mc_isaac LANGUAGES CXX VERSION 0.1.0)

if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

configure_file(etc/IsaacSim.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/IsaacSim.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/IsaacSim.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

add_subdirectory(src)

install(PROGRAMS scripts/mc_isaac scripts/mc_isaac_server scripts/mc_isaac_mesh_to_usd DESTINATION bin)
# run with the Isaac Sim python, installed as data
install(FILES scripts/mc_isaac_urdf_to_usd.py DESTINATION share/mc_isaac/scripts)
# found by mc_isaac_server through <prefix>/bin/../share/mc_isaac/server
install(FILES server/mc_isaac_server.py DESTINATION share/mc_isaac/server)
# descriptions of the mc_rtc environment modules (env/ground, env/table)
install(FILES descriptions/ground.yaml descriptions/table.yaml DESTINATION share/mc_isaac)
install(FILES descriptions/env/table.usda DESTINATION share/mc_isaac/env)
# embedded JVRC1 description (mc_rtc's default humanoid) and quick-test configurations
install(FILES robots/jvrc_isaac_description/JVRC1.yaml DESTINATION share/mc_isaac)
# mc_rtc reports the module name as jvrc1; keep the module-name lookup case available too.
install(FILES robots/jvrc_isaac_description/JVRC1.yaml DESTINATION share/mc_isaac RENAME jvrc1.yaml)
install(FILES robots/jvrc_isaac_description/jvrc1/jvrc1.usd DESTINATION share/mc_isaac/jvrc1)
install(DIRECTORY robots/jvrc_isaac_description/jvrc1/textures DESTINATION share/mc_isaac/jvrc1)
install(FILES examples/JVRC1_CoM.yaml examples/JVRC1_Posture.yaml DESTINATION share/mc_isaac/examples)
