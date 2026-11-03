file(GLOB_RECURSE QP_SRCS
    c/src/*.c
    c/solver_code/*.c
)

idf_component_register(
    SRCS ${QP_SRCS}
    INCLUDE_DIRS
        c/include
        c/solver_code
        c/solver_code/inc/public
        c/solver_code/inc/private
    PRIV_INCLUDE_DIRS
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC DFLOAT)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=parentheses
    -Wno-parentheses
)