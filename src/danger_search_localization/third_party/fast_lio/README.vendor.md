# Bundled FAST-LIO2 backend

This directory contains the core of HKU-MARS FAST-LIO at commit
`7cc4175de6f8ba2edf34bab02a42195b141027e9`.

Local integration changes remove the unused Livox `CustomMsg` and plotting
dependencies, retain the standard `sensor_msgs/PointCloud2` MARSIM path, make
frames configurable, and disable the backend's TF broadcaster. The original
GPL-2.0 license is retained in `LICENSE`.

Upstream: <https://github.com/hku-mars/FAST_LIO>
