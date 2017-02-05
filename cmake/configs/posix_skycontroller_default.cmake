include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf.cmake)

add_definitions(
	-D__PX4_POSIX_SKYCONTROLLER
	-D__DF_LINUX # Define needed DriverFramework
	)

set(CMAKE_PROGRAM_PATH
	"${PX4_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf/bin"
	${CMAKE_PROGRAM_PATH}
	)

set(config_module_list

  # examples/px4_simple_app

	#
	# Board support modules
	#
	drivers/device
	drivers/skycontroller_interface

	#
	# System commands
	#
	systemcmds/param
	systemcmds/ver

	#
	# Library modules
	#
	modules/param
	modules/systemlib
	modules/uORB

	#
	# Libraries
	#
	lib/ecl
	lib/mathlib
	lib/mathlib/math/filter
	lib/geo
	lib/geo_lookup
	lib/conversion
	lib/version
	lib/DriverFramework/framework

	#
	# POSIX
	#
	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
)

set(config_df_driver_list
)
