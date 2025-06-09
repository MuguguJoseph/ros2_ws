cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#provide the instruction to install the launch folder
#we will not install each launch file one by one,we 
#will  directly install the folder with all its contents
install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
)
#what we are telling the compiler here is to install
# the directory launch inside the share my_robot_bringup 
#folder which will be inside the install folder 
#of your ROS2 workspace
#after this,the set up is finished
 
ament_package()
