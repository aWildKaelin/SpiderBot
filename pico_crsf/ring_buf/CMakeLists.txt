add_library(ring_buffer
        ring_buffer.h
        ring_buffer.c
    )



target_include_directories (ring_buffer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})