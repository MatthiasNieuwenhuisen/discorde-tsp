cmake_minimum_required( VERSION 2.8.3 )
project( discorde_tsp )

find_package( catkin REQUIRED mod_concorde ) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS mod_concorde
)

include_directories( include ${catkin_INCLUDE_DIRS} )
#link_directories( /home/nieuwenh/catkin_workspace/devel/lib )

add_library( ${PROJECT_NAME} STATIC src/discorde_cpp.cpp )
target_link_libraries( ${PROJECT_NAME} concorde )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
