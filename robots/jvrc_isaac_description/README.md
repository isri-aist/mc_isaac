# jvrc_isaac_description (embedded in mc_isaac)

Isaac Sim description of the mc_rtc robot module `JVRC1` (mc_rtc's default humanoid), for quick tests of mc_isaac
without any other package — the equivalent of `jvrc_mj_description` in mc_mujoco.

```bash
mc_isaac -f <prefix>/share/mc_isaac/examples/JVRC1_CoM.yaml       # CoM task (sample controller CoM)
mc_isaac -f <prefix>/share/mc_isaac/examples/JVRC1_Posture.yaml   # posture task (sample controller Posture)
```

- `JVRC1.yaml` (installed in `<prefix>/share/mc_isaac`): floating base, PD gains of
  `jvrc_mj_description/pdgains/PDgains_sim.dat` (as mc_mujoco), raised effort limits (the URDF has 100 N.m
  placeholders).
- `jvrc1/jvrc1.usd` + `jvrc1/textures/` (the colored textures of the jvrc_description meshes, listed in
  `extra_files`; license: `LICENSE.jvrc_description`): made from `mc_rtc_data/jvrc_description/urdf/jvrc1.urdf`
  (package:// URIs replaced by absolute paths) with the mc_isaac converter, in the Isaac image:

  ```bash
  docker run --rm --gpus all -e ACCEPT_EULA=Y --entrypoint /isaacsim/python.sh -v <dir>:/w <isaac image> \
    /w/mc_isaac_urdf_to_usd.py /w/jvrc1.urdf /w/out/jvrc1.usd
  ```

  The converter keeps the URDF names (no fixed joint merge), uses force drives, keeps the links without
  `<inertial>` (sensor frames) massless (62.4 kg like mc_rtc, not 77.4 kg) and turns the URDF mimic finger joints
  into normal driven joints (mc_rtc computes their targets).
- Sensors: the 4 force sensors (feet, wrists), `Accelerometer` (IMU) and `FloatingBase` are simulated.

Check (standing, half-sitting posture held): base height 0.826 m, feet Fz sum 583 N = robot weight minus the
two feet below the sensors (2 × 1.5 kg), wrist sensors ≈ −16.5 N (hand weight).
