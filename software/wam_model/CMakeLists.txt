cmake_minimum_required(VERSION 2.8.3)
project(wam_model)

find_package(catkin
        REQUIRED COMPONENTS
        xacro
)


catkin_package()

file(GLOB xacro_files
        robots/wam7.urdf.xacro
        )
xacro_add_files(${xacro_files} TARGET xacro_files)
