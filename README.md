# ActiveVaR
to test on Gemini:

- `roslaunch ar_track_alvar indiv.launch`

- `roslaunch hlpr_single_plane_segmentation collect_demo.launch`
- `rosrun hlpr_single_plane_segmentation tf_broadcaster.py`

- `rosrun active_var active_var_server`
- `rosrun active_var learing_agent.py`
