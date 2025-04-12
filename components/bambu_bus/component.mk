COMPONENT_ADD_INCLUDEDIRS := .
COMPONENT_SRCDIRS := .
target_link_libraries(${COMPONENT_LIB} PRIVATE esphome::core esphome::logger)
