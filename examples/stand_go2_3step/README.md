# stand_go2_3step: keyboard stand-up / sit-down in 3 steps

This was the first working low-level controller. It is a C++ program that talks to the simulator over Unitree SDK2 DDS
(`rt/lowcmd` and `rt/lowstate`). It uses inverse kinematics for the sit and stand poses, cosine interpolation, joint PD,
and a check of the encoders to tell when a movement has finished. `go2_autonomy`'s stand-up sequence is built on this design.

```bash
bash scripts/run_sim.sh -s scene.xml                    # terminal 1 (any Go2 scene)
examples/stand_go2_3step/build/stand_go2                # terminal 2 (built by scripts/build.sh)
```
Press Enter to start, then use these keys, each followed by Enter:

| Key | Action |
|---|---|
| `u` | stand up one step |
| `d` | sit down one step |
| `p` | print the current step |
| `q` | quit |
