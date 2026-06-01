
file(GLOB SOURCES_LIST "*.cpp")


idf_component_register(
    SRCS 
        ${SOURCES_LIST}
    INCLUDE_DIRS 
        "."
)