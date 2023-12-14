
set(COMPONENT_SRCS "app_main.c"
                   "libs/sbc.c"
                   "libs/mqtt.c"
                   )
set(COMPONENT_ADD_INCLUDEDIRS ".")
# Embed the server root certificate into the final binary
set(COMPONENT_EMBED_TXTFILES ${IDF_PROJECT_PATH}server_certs/github.pem)

register_component()
