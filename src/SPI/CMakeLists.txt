add_library(SPIDMA)

target_include_directories(SPIDMA PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory("utility")

target_sources(SPIDMA
  PUBLIC
	SPI.h
  	SPIClass.h
  	SingleBufferTransfer.h
  	SingleBufTransferCopy.h
  	SingleBufTransferInplace.h
	SPIDMA.h
	SPIDMA_F1XX.h		
	SPIDMA_F4XX.h
	SPIDMA_G4XX.h
	SPIDMA_H5XX.h
	SPIDMA_H7XX.h
  PRIVATE
  	SPIClass.cpp
  	SingleBufferTransfer.cpp
  	SingleBufTransferCopy.cpp
  	SingleBufTransferInplace.cpp  	
  	SPIDMA.cpp
  	SPIDMA_F1XX.cpp
  	SPIDMA_F4XX.cpp
  	SPIDMA_G4XX.cpp
  	SPIDMA_H5XX.cpp
  	SPIDMA_H7XX.cpp  	
	utility/spi_com.h
	utility/spi_com.c	
  )

target_link_libraries(SPIDMA PUBLIC
  base_config
  core_usage
)  

#include(CMakePrintHelpers)
#cmake_print_variables(CORE_PATH)
#message("CM5_PATH=" "${CORE_PATH}")

# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#   message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

#include(print_target_properties)
#print_target_properties(SPIDMA)

