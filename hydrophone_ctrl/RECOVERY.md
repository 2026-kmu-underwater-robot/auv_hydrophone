# RC controller recovery

The controller, estimator, visualizer, launch, and RViz files in this package
were recovered from Git tree:

`66a03ac0cbeaf3f0849c55e7ab6028532df70bec`

Recovered source blobs:

- `near_zone_line_search_controller.cpp`: `c20c1b9e6850d82c0bac81b15ae0682c964e922e`
- `region_local_gradient_estimator.cpp`: `5ea72e866283e0e65f8c6a23e589eaa9a190b6e9`
- `region_local_gradient_rviz_visualizer.cpp`: `aab9e56af4ce9676dece8cb95ff19705a6b8ae91`
- `waypoint_homing_controller.cpp`: `0af3fc9f4b3541237bc126527f23acc37c91a933`

The recovered C++ source files are preserved byte-for-byte. Launch files only
change package references where required to run the recovered nodes from
`hydrophone_ctrl`; simulation and audio nodes remain supplied by
`audio_capture`.
