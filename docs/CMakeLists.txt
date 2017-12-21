find_package(Doxygen)
option(BUILD_DOCS "Create Doxygen API Documentation" ${DOXYGEN_FOUND})

if(BUILD_DOCS)
    if(NOT DOXYGEN_FOUND)
		message(FATAL_ERROR "Doxygen required to build documentation")
    endif()

	set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
	set(DOXYFILE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

	configure_file(${DOXYFILE_IN} ${DOXYFILE} @ONLY)

    add_custom_target(doc
		ALL
		COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE}
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating API documentation with Doxygen"
		VERBATIM)
endif()
