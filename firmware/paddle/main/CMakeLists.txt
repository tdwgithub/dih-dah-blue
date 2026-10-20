# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

#idf_component_register(
#    SRCS main.c         # list the source files of this component
#    INCLUDE_DIRS        # optional, add here public include directories
#    PRIV_INCLUDE_DIRS   # optional, add here private include directories
#    REQUIRES            # optional, list the public requirements (component names)
#    PRIV_REQUIRES       # optional, list the private requirements
#)



# Set usual component variables
set(app_sources "main.c")

idf_component_register(SRCS ${app_sources}
                       REQUIRES
                               # Core components
                                driver
                                esp_system
                                esp_timer
                                esp_pm
                                esp_hw_support

                                # Storage
                                nvs_flash

                                # Bluetooth
                                bt

                                # ULP
                                ulp

                                # FreeRTOS
                                freertos

                                # Utilities
                                esp_common
                                log

                            PRIV_REQUIRES
                                # Private requirements
                                esp_phy
                                WHOLE_ARCHIVE)
#
# ULP support additions to component CMakeLists.txt.
#
# 1. The ULP app name must be unique (if multiple components use ULP).
set(ulp_app_name ulp_${COMPONENT_NAME})


#
# 2. Specify all C and Assembly source files.
#    Files should be placed into a separate directory (in this case, ulp/),
#    which should not be added to COMPONENT_SRCS.
set(ulp_sources "ulp/main.c")

#
# 3. List all the component source files which include automatically
#    generated ULP export file, ${ulp_app_name}.h:
set(ulp_exp_dep_srcs ${app_sources})

#
# 4. Call function to build ULP binary and embed in project using the argument
#    values above.
ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")

target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--undefined=ulp_lp_core_memory_shared_cfg")
