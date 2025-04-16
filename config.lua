-- Include default configurations for map building and trajectory processing
include "map_builder.lua"
include "trajectory_builder.lua"

-- Options table holding the main configuration
options = {
  -- Use the pre-defined configurations from included files
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- Frame IDs used by Cartographer
  map_frame = "map",                      -- The frame ID for the final map
  tracking_frame = "base_link",           -- The frame ID attached to the robot's base that moves with it (LiDAR is often relative to this)
                                          -- Note: The OCR shows "imu_link" commented out, base_link is likely correct for typical setups
  published_frame = "odom",               -- The frame ID for the published pose, relative to the map frame (often the odom frame)
                                          -- Note: OCR shows "odom" commented out for the first odom, but uses "odom" for the second. Assuming "odom" is correct.
  odom_frame = "odom",                    -- The frame ID of the input odometry data
                                          -- Note: OCR shows "odom" commented out. Using "odom" as it's standard practice when use_odometry = true.

  -- Input data configuration
  provide_odom_frame = false,             -- Set to true if you DON'T have external odometry on odom_frame
  publish_frame_projected_to_2d = true,   -- Publish the tracked pose projected onto the 2D plane (required for Navigation2)
  use_odometry = true,                    -- Use external odometry data (e.g., from wheel encoders or rf2o)
  use_nav_sat = false,                    -- Use NavSatFix messages (GPS) - Typically false for indoor
  use_landmarks = false,                  -- Use landmark data - Typically false unless using specific landmark features
  num_laser_scans = 1,                    -- Number of laser scan topics to subscribe to
  num_multi_echo_laser_scans = 0,         -- Number of multi-echo laser scan topics
  num_subdivisions_per_laser_scan = 1,    -- Split each laser scan into N smaller scans (rarely needed)
  num_point_clouds = 0,                   -- Number of point cloud topics to subscribe to

  -- Timing and rate control
  lookup_transform_timeout_sec = 0.2,     -- Max time to wait for TF transforms
  submap_publish_period_sec = 0.3,        -- How often to publish submap data
  pose_publish_period_sec = 5e-3,         -- How often to publish the robot's pose (e.g., 200Hz) - Note: 5e-3 = 0.005 seconds
  trajectory_publish_period_sec = 30e-3,  -- How often to publish trajectory data (e.g., ~33Hz) - Note: 30e-3 = 0.03 seconds

  -- Sampling ratios for input data (1.0 = use all data)
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- Enable 2D SLAM mode in the map builder
MAP_BUILDER.use_trajectory_builder_2d = true

-- Configure the 2D Trajectory Builder (Laser Scan Processing)
TRAJECTORY_BUILDER_2D.min_range = 0.15      -- Minimum valid laser scan range (meters) - Adjust based on LiDAR specs
TRAJECTORY_BUILDER_2D.max_range = 3.5       -- Maximum valid laser scan range (meters) - Adjust based on LiDAR specs
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0 -- How far to trace rays for points marked as missing data
TRAJECTORY_BUILDER_2D.use_imu_data = false   -- Set to true if you have an IMU providing orientation data
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true -- Use real-time scan matching for local adjustments
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) -- Filter out scans if rotation exceeds this threshold (0.1 degrees)

-- Configure the Pose Graph (Loop Closure and Optimization)
POSE_GRAPH.constraint_builder.min_score = 0.65 -- Minimum score for a scan matching result to be considered a loop closure constraint
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7 -- Minimum score required for global localization (relocalization)

-- Return the final options table
return options