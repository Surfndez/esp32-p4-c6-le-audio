idf_build_get_property(target IDF_TARGET)

set(srcs)
set(public_include "include")
if(CONFIG_SOC_PPA_SUPPORTED)
    list(APPEND srcs "src/ppa_core.c"
                     "src/ppa_srm.c"
                     "src/ppa_blend.c"
                     "src/ppa_fill.c")
endif()

if(${target} STREQUAL "linux")
    set(priv_requires "")
else()
    set(priv_requires esp_mm esp_pm)
endif()


# Cyberdeck: copy of the driver from ESP-IDF v5.5.5 with a fix for a hung SRM engine. ppa_core.c reads the
# 2D-DMA channel owner from the private esp_hw_support header (dma2d_priv.h) - hence the directory from IDF.
idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${public_include}
                       PRIV_INCLUDE_DIRS "${IDF_PATH}/components/esp_hw_support/dma"
                       PRIV_REQUIRES "${priv_requires}"
                      )
