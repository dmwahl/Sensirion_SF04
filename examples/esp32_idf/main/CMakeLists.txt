# Component CMakeLists.txt for SF04 example

set(COMPONENT_SRCS
    "sf04_example.cpp"
    "../../../sensirion_sf04_v2.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "../../.."
    "../../../hal"
)

register_component()
