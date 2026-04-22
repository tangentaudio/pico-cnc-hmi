# LVGL + Pico SDK Upgrade Notes (firmware/)

## SDK 2.1.1 → 2.2.0
- New module `pico_platform_common` added in 2.2.0
- Must add to LVGL include paths in CMakeLists.txt:
  `${PICO_SDK_PATH}/src/rp2_common/pico_platform_common/include/`

## LVGL 9.3.0 → 9.5.0
- `lv_conf.h` discovery changed — use `set(LV_BUILD_CONF_DIR <dir> CACHE PATH "" FORCE)` before `add_subdirectory(lvgl)`
- NXP VGLite backend removed — requires clean build wipe after LVGL version change (stale glob)
- `lvgl/env_support/cmake/custom.cmake` must exist as an empty file (user hook); create it with `touch`, then re-run cmake and `touch build/build.ninja` to fix timestamps
- cmake re-run loop root cause: custom.cmake listed as phony output but missing → ninja always dirty
