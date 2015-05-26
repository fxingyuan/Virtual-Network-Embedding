cmake_minimum_required (VERSION 2.8)
project(Analysis)

set(analysis_incs	def.h
					In_Event.h)
set(analysis_srcs	In_Event.cpp
					main.cpp)
#main
source_group("analysis_incs" FILES ${analysis_incs})
source_group("analysis_srcs" FILES ${analysis_srcs})
add_executable(analysis ${analysis_incs} ${analysis_srcs})
