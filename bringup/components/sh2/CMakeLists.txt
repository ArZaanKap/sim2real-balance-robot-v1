# CEVA's official SH-2 host driver, vendored unchanged from:
# https://github.com/ceva-dsp/sh2 commit b514b1e2586ddc195e553dac89fc94c637b25298
idf_component_register(SRCS "sh2.c"
                            "shtp.c"
                            "sh2_SensorValue.c"
                            "sh2_util.c"
                       INCLUDE_DIRS ".")
