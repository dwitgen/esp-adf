
set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_SRCS "audio_element.c"
                    "audio_event_iface.c"
                    "audio_pipeline.c"
                    "ringbuf.c"
                    "i2s_debug.c")

set(COMPONENT_REQUIRES audio_sal esp-adf-libs driver)

register_component()
