file(GLOB_RECURSE QP_SRCS
    c/*.c
)

idf_component_register(
    SRCS ${QP_SRCS}
    INCLUDE_DIRS
        c/
        c/inc/public
        c/inc/private
    PRIV_INCLUDE_DIRS
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC DFLOAT)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=parentheses
    -Wno-parentheses
)