# No REQUIRES/PRIV_REQUIRES here on purpose: ESP-IDF gives the "main" component
# an implicit dependency on every other component in the build, and declaring
# any requirement replaces that - which loses esp_matter, nvs_flash and the
# device_hal headers.
idf_component_register(SRC_DIRS          "."
                       PRIV_INCLUDE_DIRS  "." "${ESP_MATTER_PATH}/examples/common/utils")

if (CONFIG_ENABLE_SET_CERT_DECLARATION_API)
    target_add_binary_data(${COMPONENT_TARGET} "certification_declaration/certification_declaration.der" BINARY)
endif()

target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")
