
# A camera used for tensorflow object detection
# TODO - these values are all guesses
define normal_camera fiducial
(
	range_min 0.0
	range_max 10.0
	range_max_id 10.0
	fov 359
	ignore_zloc 1
	size [ 0.2 0.2 0.2 ]
	color "purple"
)

# terabee evo 3m
define terabee_distance sensor
(
  # define the size of each transducer [xsize ysize zsize] in meters
  size [0.01 0.05 0.01 ]
  # define the range bounds [min max]
  range [0.01 3.0]
  # define the angular field of view in degrees
  fov 2
  # define the number of samples spread over the fov
  samples 1
)

define single_terrabee ranger
(
  terabee_distance(pose [ 0 0 0 0 ] )
)

define omnidir_robot position
(
  update_interval 5
  pose [ 0.0 0.0 0.0 0.0 ]

  localization "odom"
  odom_error [ 0.02 0.02 0.0 0.002 ]

  size [ 0.889 0.889 0.30 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  color "red"

  # kinematics
  drive "omni"

  # [ xmin xmax ymin ymax zmin zmax amin amax ]
  velocity_bounds [-5.5 5.5 -5.5 5.5 0 0 -90 90]
  acceleration_bounds [-4.5 4.5 -4.5 4.5 0 0 -90 90]

  # spawn sensors
  normal_camera(pose [ 0.1 -0.1 0.65 0.0 ] )

  single_terrabee(pose [0.0 0.4445 0.1 90 ] )

  single_terrabee(pose [0.889 0.4445 0.1 90 ] )

	camera
(
  # laser properties
  resolution [ 32 32 ]
  range [ 0.2 8.0 ]
  fov [ 70.0 40.0 ]
  pantilt [ 0.0 0.0 ]

  # model properties
  size [ 0.66675 0.2286 0.15 ]
	pose [0.0 0.4445 -0.3 0 ]
  color "purple"
  watts 100.0
)
)
