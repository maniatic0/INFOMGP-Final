cmake_minimum_required(VERSION 3.1)
project(PracticalFinal)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         	ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           	ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  	ON)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"  			ON)
option(LIBIGL_WITH_CGAL           	 "Use CGAL"  			ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI  "Use OpenGL viewer with ImGui"  ON)


find_package(LIBIGL REQUIRED QUIET)

# Add your project files
file(GLOB SRCFILES *.cpp *.h *.c)
add_executable(${PROJECT_NAME}_bin ${SRCFILES})
target_link_libraries(${PROJECT_NAME}_bin igl::core igl::opengl_glfw igl::tetgen igl::cgal igl::opengl_glfw_imgui ${GMP_LIBRARIES} ${MPFR_LIBRARIES})

add_custom_command(TARGET ${PROJECT_NAME}_bin POST_BUILD 
               COMMAND ${CMAKE_COMMAND} -E copy_if_different
               ${GMP_LIBRARIES_DIR}/libgmp-10.dll            
                $<TARGET_FILE_DIR:${PROJECT_NAME}_bin>)

add_custom_command(TARGET ${PROJECT_NAME}_bin POST_BUILD 
               COMMAND ${CMAKE_COMMAND} -E copy_if_different
               ${MPFR_LIBRARIES_DIR}/libmpfr-4.dll          
                $<TARGET_FILE_DIR:${PROJECT_NAME}_bin>)