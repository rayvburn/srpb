# srpb

**S**ocial **R**obot navigation **P**lanners **B**enchmark

Package developed on top of:
* [`MRPB`: Mobile Robot Local Planning Benchmark](https://github.com/NKU-MobFly-Robotics/local-planning-benchmark).

The corresponding package which allows logging data while the robot is navigating is available at [`srpb_move_base`](https://github.com/rayvburn/srpb_move_base).

A log file is saved once the goal was reached by the `srpb_move_base`. Renewing the goal before reaching the previous one does not cause the files to be divided into parts.
