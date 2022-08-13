include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame  = os.getenv("tf_prefix") ..     -- Frame id to track from map
                    "/base_footprint",
  published_frame = os.getenv("tf_prefix") ..     -- Odom frame to publish if `provide_odom_frame = true`. Even though
                    "/odom",                      -- are not used, all the options' parameter keys are mandatory
  odom_frame =      os.getenv("tf_prefix") ..     -- Frame where publish the odometry to if `provide_odom_frame=true`
                    "/odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = false

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 4.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.0

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 70
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 300

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.035
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120
POSE_GRAPH.constraint_builder.min_score = 0.82
POSE_GRAPH.constraint_builder.sampling_ratio = 1.

POSE_GRAPH.optimization_problem.huber_scale = 1e2

TRAJECTORY_BUILDER.pure_localization = true     -- This makes cartographer to start a new trajectory on top of the
                                                -- previous witout writing the new data to the actual map
POSE_GRAPH.optimize_every_n_nodes = 20          -- While doing pure localization, optimization can be done more frequently

return options
