#!/usr/bin/env bash
LOCALIZATION_MAP_OUTPUT=$1
ROSBAG=$2
REST=$@

rosrun rovioli rovioli \
  --alsologtostderr=1 \
  --v=2 \
  --ncamera_calibration=ncamera-rotors-stereo.yaml  \
  --imu_parameters_maplab=imu-adis16488.yaml \
  --imu_parameters_rovio=imu-sigmas-rovio.yaml \
  --publish_debug_markers  \
  --datasource_type="rosbag" \
  --optimize_map_to_localization_map=true \
  --save_map_folder=$LOCALIZATION_MAP_OUTPUT \
  --map_builder_save_image_as_resources=false \
  --datasource_rosbag=$ROSBAG $REST
