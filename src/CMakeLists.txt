file(GLOB files "nodes/*.cpp")

foreach(file ${files})
	get_filename_component(tempName ${file} NAME_WE)
	add_executable(${tempName} ${file} configurationManager.cpp)
	add_dependencies(${tempName} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(${tempName}
		${catkin_LIBRARIES}
		${EXTERNAL_LIBS}
	)
endforeach()

