# http://www.cmake.org/Wiki/CMakeMacroParseArguments
macro(isae_parse_arguments prefix arg_names option_names)
  set(DEFAULT_ARGS)
  foreach(arg_name ${arg_names})    
    set(${prefix}_${arg_name})
  endforeach(arg_name)
  foreach(option ${option_names})
    set(${prefix}_${option} FALSE)
  endforeach(option)

  set(current_arg_name DEFAULT_ARGS)
  set(current_arg_list)
  foreach(arg ${ARGN})            
    set(larg_names ${arg_names})    
    list(FIND larg_names "${arg}" is_arg_name)                   
    if (is_arg_name GREATER -1)
      set(${prefix}_${current_arg_name} ${current_arg_list})
      set(current_arg_name ${arg})
      set(current_arg_list)
    else()
      set(loption_names ${option_names})    
      list(FIND loption_names "${arg}" is_option)            
      if (is_option GREATER -1)
	     set(${prefix}_${arg} TRUE)
      else ()
	     set(current_arg_list ${current_arg_list} ${arg})
      endif()
    endif()
  endforeach()
  set(${prefix}_${current_arg_name} ${current_arg_list})
endmacro()

macro(isae_generate_package)

  oro_parse_arguments(ORO_CREATE_PC
    "VERSION;DEPENDS;DEPENDS_TARGETS"
    ""
    ${ARGN}
    )

  # Check version
  if (NOT ORO_CREATE_PC_VERSION)
    if (COMPONENT_VERSION)
      set( ORO_CREATE_PC_VERSION ${COMPONENT_VERSION})
      message("[UseOrocos] Generating package version ${ORO_CREATE_PC_VERSION} from COMPONENT_VERSION.")
    else (COMPONENT_VERSION)
      set( ORO_CREATE_PC_VERSION "1.0")
      message("[UseOrocos] Generating package version ${ORO_CREATE_PC_VERSION} (default version).")
    endif (COMPONENT_VERSION)
  else (NOT ORO_CREATE_PC_VERSION)
    message("[UseOrocos] Generating package version ${ORO_CREATE_PC_VERSION}.")
  endif (NOT ORO_CREATE_PC_VERSION)

  # Create filename
  if ( ORO_CREATE_PC_DEFAULT_ARGS )
    set(PC_NAME ${ORO_CREATE_PC_DEFAULT_ARGS})
  else ( ORO_CREATE_PC_DEFAULT_ARGS )
    set(PC_NAME ${PROJECT_NAME} )
    if ( NOT ${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${${PROJECT_NAME}_SOURCE_DIR} )
      # Append -subdir-subdir-... to pc name:
      file(RELATIVE_PATH RELPATH ${${PROJECT_NAME}_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} )
      string(REPLACE "/" "-" PC_NAME_SUFFIX ${RELPATH} )
      set(PC_NAME ${PC_NAME}-${PC_NAME_SUFFIX})
    endif ( NOT ${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${${PROJECT_NAME}_SOURCE_DIR} )
    set(PC_NAME ${PC_NAME}-${OROCOS_TARGET})
  endif ( ORO_CREATE_PC_DEFAULT_ARGS )

  # Create dependency list
  foreach( DEP ${ORO_CREATE_PC_DEPENDS_TARGETS})
     list(APPEND ${ORO_CREATE_PC_DEPENDS} ${DEP}-${OROCOS_TARGET})
  endforeach()

  # Create lib-path list
  set(PC_LIBS "Libs: ")
  if (OROCOS_DEFINED_LIBS)
    set(PC_LIBS "${PC_LIBS} -L\${libdir} ${OROCOS_DEFINED_LIBS}")
  endif (OROCOS_DEFINED_LIBS)
  if (OROCOS_DEFINED_COMPS)
    set(PC_LIBS "${PC_LIBS} -L\${orocos_libdir} ${OROCOS_DEFINED_COMPS}")
  endif (OROCOS_DEFINED_COMPS)
  if (OROCOS_DEFINED_PLUGINS)
    set(PC_LIBS "${PC_LIBS} -L\${orocos_libdir}/plugins ${OROCOS_DEFINED_PLUGINS}")
  endif (OROCOS_DEFINED_PLUGINS)
  if (OROCOS_DEFINED_TYPES)
    set(PC_LIBS "${PC_LIBS} -L\${orocos_libdir}/types ${OROCOS_DEFINED_TYPES}")
  endif (OROCOS_DEFINED_TYPES)

  if(CMAKE_SYSROOT)
	  string(REGEX REPLACE "^${CMAKE_SYSROOT}" "" PC_PREFIX ${CMAKE_INSTALL_PREFIX})
  else()
	  set(PC_PREFIX ${CMAKE_INSTALL_PREFIX})
  endif()
  set(PC_LIB_DIR "\${libdir}/orocos${OROCOS_SUFFIX}/${PROJECT_NAME}")
  set(PC_CONTENTS "prefix=@PC_PREFIX@
libdir=\${prefix}/lib
includedir=\${prefix}/include/orocos/${PROJECT_NAME}
orocos_libdir=${PC_LIB_DIR}

Name: ${PC_NAME}
Description: ${PC_NAME} package for Orocos
Requires: orocos-rtt-${OROCOS_TARGET} ${ORO_CREATE_PC_DEPENDS}
Version: ${ORO_CREATE_PC_VERSION}
${PC_LIBS}
Cflags: -I\${includedir}
")
  string(CONFIGURE "${PC_CONTENTS}" INSTALLED_PC_CONTENTS @ONLY)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${PC_NAME}.pc ${INSTALLED_PC_CONTENTS})

  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PC_NAME}.pc DESTINATION lib/pkgconfig )

  # Also set the uninstall target:
  orocos_uninstall_target()

endmacro()

macro(isae_search_yarp)
	find_package(YARP REQUIRED)
	include_directories(${YARP_INCLUDE_DIRS})
	link_libraries(${YARP_LIBRARIES})
endmacro()

macro(isae_base_orocos name)
	if (NOT isae_meta_project)
		project(${name})

		set(ORO_USE_ROSBUILD OFF)
		set(ORO_USE_CATKIN OFF)

		# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
		# for helping these find commands find RTT.
		find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
		 
		# Defines the orocos_* cmake macros. See that file for additional
		# documentation.
		include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

		enable_testing()
		find_package(Boost REQUIRED COMPONENTS unit_test_framework)

		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
		if("${CMAKE_CXX_COMPILER}" MATCHES "clang\\+\\+.*")
			set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-return-type-c-linkage -Wno-deprecated-register")
		endif()

		option(ENABLE_CLANG_SANITIZER "Will compile with clang sanitizers" OFF)
		if (ENABLE_CLANG_SANITIZER)
			option(ENABLE_CLANG_SANITIZER_ADDRESS "Enable Address sanitizer" ON)
			option(ENABLE_CLANG_SANITIZER_MEMORY "Enable Memory sanitizer" OFF)
		endif()

		if("${CMAKE_C_COMPILER}" MATCHES "clang.*")
			if (ENABLE_CLANG_SANITIZER_ADDRESS)
				set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fPIE -fno-omit-frame-pointer") 
				set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie -fsanitize=address")
			endif()
			if (ENABLE_CLANG_SANITIZER_MEMORY)
				set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=memory -fsanitize-memory-track-origins -fPIE -fno-omit-frame-pointer")
				set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie -fsanitize=memory")
			endif()
		endif()

		if("${CMAKE_CXX_COMPILER}" MATCHES "clang\\+\\+.*")
			if (ENABLE_CLANG_SANITIZER_ADDRESS)
				set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fPIE -fno-omit-frame-pointer")
				set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie -fsanitize=address")
			endif()
			if (ENABLE_CLANG_SANITIZER_MEMORY)
				set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=memory -fsanitize-memory-track-origins -fPIE -fno-omit-frame-pointer")
				set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie -fsanitize=memory")
			endif()
		endif()


	endif()
endmacro()

macro(_isae_search_type_header name)
	if (EXISTS ${CMAKE_SOURCE_DIR}/ORO_TYPE_${name})
		include_directories(${CMAKE_SOURCE_DIR}/ORO_TYPE_${name})
	else()
		string(TOUPPER "${name}" NAME)
		pkg_check_modules(PC_${NAME} REQUIRED ${name}-${OROCOS_TARGET})
		include_directories(${PC_${NAME}_INCLUDEDIR})
	endif()
endmacro()
 
macro(isae_orocos_type name)
	isae_parse_arguments(COMP
		"INTERNAL_LIB_DEPENDS;DEPENDS;TYPE_DEPENDS"
		""
		${ARGN}
	 )
	isae_base_orocos(${name})

	foreach(type ${COMP_TYPE_DEPENDS})
		message(STATUS "Component ${name} depending of type in ${type}")
		_isae_search_type_header(${type})
	endforeach()

	foreach(depends ${COMP_DEPENDS})
		orocos_use_package(${depends})
	endforeach()
 
        file(GLOB EXTRA_TYPEKIT_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

        orocos_typekit(${name} typekit.cpp ${EXTRA_TYPEKIT_SRC})

	file(GLOB headers ${CMAKE_CURRENT_SOURCE_DIR}/*h)
	file(GLOB headers2 ${CMAKE_CURRENT_SOURCE_DIR}/*hpp)
	add_custom_target(${name}-type-headers SOURCES ${headers} ${headers2})

	orocos_install_headers(${headers})
	orocos_install_headers(${headers2})

	isae_generate_package(${name}-${OROCOS_TARGET})
endmacro()

macro(_isae_search_internal_libs name)
	if (EXISTS ${CMAKE_SOURCE_DIR}/ORO_${name})
		include_directories(${CMAKE_SOURCE_DIR}/ORO_${name})
		link_libraries(${name})
	else()
		string(TOUPPER "${name}" NAME)
		pkg_check_modules(PC_${NAME} REQUIRED ${name}-${OROCOS_TARGET})
		include_directories(${PC_${NAME}_INCLUDEDIR}/${name})

		orocos_use_package(${name})
	endif()
endmacro()

macro(isae_orocos_comp name)
	isae_parse_arguments(COMP
		"TYPE_DEPENDS;INTERNAL_LIB_DEPENDS;DEPENDS;EXTRA_INCLUDE_DIRS;EXTRA_DIR_SRCS"
		""
		${ARGN}
	 )

	isae_base_orocos(${name})

	foreach(type ${COMP_TYPE_DEPENDS})
		message(STATUS "Component ${name} depending of type in ${type}")
		_isae_search_type_header(${type})
	endforeach()

	foreach(internal_lib ${COMP_INTERNAL_LIB_DEPENDS})
		message(STATUS "Component ${name} depending of library ${internal_lib}")
		_isae_search_internal_libs(${internal_lib})
	endforeach()

	foreach(depends ${COMP_DEPENDS})
		message(STATUS "Component ${name} depending of external library ${depends}")
		orocos_use_package(${depends})
	endforeach()

	foreach(includedir ${COMP_EXTRA_INCLUDE_DIRS})
		include_directories(${includedir})
	endforeach()

	file(GLOB SRCS *.cpp)
	file(GLOB SRCS2 *.c)

	set(EXTRA_SRCS "")
	foreach(dir ${COMP_EXTRA_DIR_SRCS})
		file(GLOB _SRCS ${dir}/*.cpp)
		list(APPEND EXTRA_SRCS ${_SRCS})
		file(GLOB _SRCS ${dir}/*.c)
		list(APPEND EXTRA_SRCS ${_SRCS})
	endforeach()

	orocos_component(${name} ${SRCS} ${SRCS2} ${EXTRA_SRCS})

	isae_generate_package(${name}-${OROCOS_TARGET})
endmacro()

macro(isae_internal_lib name)
	isae_parse_arguments(COMP
		"TYPE_DEPENDS;INTERNAL_LIB_DEPENDS;DEPENDS;EXTRA_DIR_SRCS"
		""
		${ARGN}
	 )

	isae_base_orocos(${name})

	foreach(internal_lib ${COMP_INTERNAL_LIB_DEPENDS})
		message(STATUS "Component ${name} depending of library ${internal_lib}")
		_isae_search_internal_libs(${internal_lib})
	endforeach()

	foreach(type ${COMP_TYPE_DEPENDS})
		message(STATUS "Component ${name} depending of type in ${type}")
		_isae_search_type_header(${type})
	endforeach()

	foreach(depends ${COMP_DEPENDS})
		message(STATUS "Component ${name} depending of external library ${depends}")
		orocos_use_package(${depends})
	endforeach()

	file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
	file(GLOB SRCS2 ${CMAKE_CURRENT_SOURCE_DIR}/*.c)

	set(EXTRA_SRCS "")
	set(HEADER_INSTALL_DIR include/orocos/${PROJECT_NAME} ) # XXX
	foreach(dir ${COMP_EXTRA_DIR_SRCS})
		file(GLOB _SRCS ${dir}/*.cpp)
		list(APPEND EXTRA_SRCS ${_SRCS})
		file(GLOB _SRCS ${dir}/*.c)
		list(APPEND EXTRA_SRCS ${_SRCS})
		set(EXTRA_HEADERS "")
		file(GLOB _HEADERS ${dir}/*.hpp)
		list(APPEND EXTRA_HEADERS ${_HEADERS})
		file(GLOB _HEADERS ${dir}/*.h)
		list(APPEND EXTRA_HEADERS ${_HEADERS})
		install(FILES ${EXTRA_HEADERS} DESTINATION ${HEADER_INSTALL_DIR}/${dir})
	endforeach()

	orocos_library(${name} ${SRCS} ${SRCS2} ${EXTRA_SRCS})

	string(TOUPPER "test_${name}_sources" THIS_LIB_TEST_SOURCES)
	file(GLOB  ${THIS_LIB_TEST_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_*.cpp) 
	foreach(file ${${THIS_LIB_TEST_SOURCES}})
		get_filename_component(basename ${file} NAME_WE)
		string(TOLOWER "${name}_${basename}" THIS_TEST_EXECUTABLE)
		add_executable(${THIS_TEST_EXECUTABLE} ${file})
		target_include_directories(${THIS_TEST_EXECUTABLE} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
		target_link_libraries(${THIS_TEST_EXECUTABLE} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} ${name})
		add_test(${THIS_TEST_EXECUTABLE}  ${THIS_TEST_EXECUTABLE})
	endforeach()

	file(GLOB headers ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
	file(GLOB headers1 ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
	add_custom_target(${name}-internal-headers SOURCES ${headers} ${headers2})

	orocos_install_headers(${headers} ${headers1})

	isae_generate_package(${name}-${OROCOS_TARGET})
endmacro()

macro(isae_orocos_yarp_plugin name)
	isae_parse_arguments(COMP
		"TYPE_DEPENDS"
		""
		${ARGN}
	 )

	isae_base_orocos(yarp-${name})

	orocos_use_package(yarp)

	isae_search_yarp()

	foreach(type ${COMP_TYPE_DEPENDS})
		_isae_search_type_header(${type})
	endforeach()

	file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

	orocos_plugin(yarp-${name} ${SRCS})

	isae_generate_package(${name}-${OROCOS_TARGET})
endmacro()
