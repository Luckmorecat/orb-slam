cmake_minimum_required(VERSION 2.8)
project(DBoW2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

if (EMSCRIPTEN)

#   include_directories(${OpenCV_INCLUDE_DIRS})
   include_directories(${PROJECT_SOURCE_DIR}/../opencv-3.4/include)
   include_directories(${PROJECT_SOURCE_DIR}/../opencv-3.4/build_wasm)
   file(GLOB opencv_include_modules "${PROJECT_SOURCE_DIR}/../opencv-3.4/modules/*/include")
   include_directories(${opencv_include_modules})

else(EMSCRIPTEN)

   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      find_package(OpenCV 2.4.3 QUIET)
      if(NOT OpenCV_FOUND)
         message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
      endif()
   endif()

   include_directories(${OpenCV_INCLUDE_DIRS})

endif ()


set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})

if (EMSCRIPTEN)

   file(GLOB opencv_js "${PROJECT_SOURCE_DIR}/../opencv-3.4/build_wasm/lib/*.a")

   target_link_libraries(DBoW2
           ${opencv_js}
           )
else(EMSCRIPTEN)
   target_link_libraries(DBoW2 ${OpenCV_LIBS})
endif ()


