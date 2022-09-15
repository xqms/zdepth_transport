
cmake_minimum_required(VERSION 3.10)
project(zdepth_transport)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
    rosfmt
)

catkin_package()

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
  zdepth
  GIT_REPOSITORY https://github.com/catid/Zdepth.git
  GIT_TAG        master
  GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(zdepth)

add_library(zdepth_transport
    src/publisher.cpp
    src/subscriber.cpp
)
target_include_directories(zdepth_transport PRIVATE
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(zdepth_transport
    ${catkin_LIBRARIES}
    zdepth::zdepth
)

install(TARGETS zdepth_transport
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
