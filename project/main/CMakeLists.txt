
set(COMPONENT_SRCS "app_main.c"
                   "libs/sbc.c"
                   "libs/mqtt.c"
                   "libs/bme280.c"
                   "libs/wifi_manager.c"
                   "libs/dns_server.c"
                   )
set(COMPONENT_ADD_INCLUDEDIRS ".")
# Embed the server root certificate into the final binary
set(COMPONENT_EMBED_TXTFILES ${IDF_PROJECT_PATH}server_certs/github.pem)

set(COMPONENT_EMBED_FILES src/style.css src/code.js src/index.html)

register_component()
