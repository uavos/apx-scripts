# APX scripts used for onboard virtual machines

This is the archive of scripts, used with [APX Scripting Engine](https://docs.uavos.com/fw/script).

> Scripts published in this repository are not guaranteed to work on particular hardware and with specific firmware, thus, should be considered as examples or templates only.

## Contents

Important root directories:

- `drivers` - published drivers source code;
- `safety` - safety scripts (i.e. when to deploy cancel a mission and how);
- `test` - examples of scripts and functionality tests;

## Contributions

Clone, design, test, and initiate a `Pull Request` to publish your script into archive. The published content should be located in a subfolder with the following contents:

- `README.md` file with the description of how to use the script and what it does;
- script source code file[s];

Examples:

- fuel level sensor driver should be published under `drivers` subfolder, and should be named as `fuel-XXX` folder, where XXX is the part number which distinguish the device.

- safety procedure script should be published under `safety` subfolder, and named accordingly (f.ex. `safety-HiDRON-2`, to distinguish it and to help to find the proper sample for the public. The `README.md` file should contain the full description of the procedure.
