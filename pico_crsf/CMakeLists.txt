add_library(pico_crsf
        crsf_parser.h
        crsf_packets.h
        crsf_parser.c
    )

add_subdirectory(ring_buf)

target_link_libraries(pico_crsf
        ring_buffer
        pico_stdlib
    )


target_include_directories (pico_crsf PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})