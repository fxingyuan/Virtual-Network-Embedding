cmake_minimum_required (VERSION 2.8)
project(VirtualNetworkEmbedding)

# Boost
set(BOOST_PATH ${PROJECT_SOURCE_DIR}/thirdparty/boost_1_55_0)
include_directories(${BOOST_PATH})
link_directories(${BOOST_PATH}/libs)

# k-shortest-paths
set(KSP_PATH ${PROJECT_SOURCE_DIR}/thirdparty/k-shortest-paths)
include_directories(${KSP_PATH})

# vne-main
set(sim_incs				def.h
							simulator.h
							utility.h
)
set(sim_srcs				main.cpp
							simulator.cpp
							utility.cpp
)

# k-short-path
set(ksp_incs				./thirdparty/k-shortest-paths/QYConfigCenter.h
							./thirdparty/k-shortest-paths/QYDirectedGraph.h
							./thirdparty/k-shortest-paths/QYDirectedPath.h
							./thirdparty/k-shortest-paths/QYKShortestPaths.h
							./thirdparty/k-shortest-paths/QYShortestPath.h
)
set(ksp_srcs				./thirdparty/k-shortest-paths/QYDirectedGraph.cpp
							./thirdparty/k-shortest-paths/QYKShortestPaths.cpp
							./thirdparty/k-shortest-paths/QYShortestPath.cpp
)

SOURCE_GROUP("vne-incs" FILES ${sim_incs})
SOURCE_GROUP("vne-srcs" FILES ${sim_srcs})
SOURCE_GROUP("k-shortest-paths-incs" FILES ${ksp_incs})
SOURCE_GROUP("k-shortest-paths-srcs" FILES ${ksp_srcs})

add_executable(VNE ${sim_incs} ${sim_srcs} ${ksp_incs} ${ksp_srcs})
  
#target_link_libraries(VNE 	debug OpenMeshCored OpenMeshToolsd optimized OpenMeshCore OpenMeshTools					# OpenMesh
#										freeglut						 														# freeglut
#)