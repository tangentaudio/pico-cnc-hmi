# generate_fonts.cmake — Build-time font generation using lv_font_conv
#
# Generates LVGL font .c files from TTF sources in fonts/src/.
# Requires: npx (Node.js) with lv_font_conv available.

set(FONT_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)
set(FONT_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/fonts)
file(MAKE_DIRECTORY ${FONT_GEN_DIR})

# Find npx (required for lv_font_conv)
find_program(NPX_EXECUTABLE npx REQUIRED)

# Helper: add a custom command to generate a single LVGL font .c file.
#
# Usage:
#   add_lvgl_font(
#     OUTPUT_NAME  roboto_14           # Output filename (without .c) and C symbol name
#     FONT_FILE    Roboto-Variable...  # TTF filename inside FONT_SRC_DIR
#     SIZE         14                  # Pixel size
#     BPP          4                   # Bits per pixel
#     RANGE        "32-127"            # Unicode range(s), comma-separated
#     FALLBACK     fa_solid_18         # Optional: fallback font symbol name (baked into const struct)
#   )
function(add_lvgl_font)
    cmake_parse_arguments(FONT "" "OUTPUT_NAME;FONT_FILE;SIZE;BPP;RANGE;FALLBACK" "" ${ARGN})

    set(OUTPUT_FILE ${FONT_GEN_DIR}/${FONT_OUTPUT_NAME}.c)
    set(INPUT_FILE  ${FONT_SRC_DIR}/${FONT_FONT_FILE})

    if(FONT_FALLBACK)
        # Generate, then patch .fallback = NULL to point to the fallback font.
        # This keeps the font struct const (in flash) with the fallback baked in.
        add_custom_command(
            OUTPUT  ${OUTPUT_FILE}
            COMMAND ${NPX_EXECUTABLE} -y lv_font_conv
                    --bpp ${FONT_BPP}
                    --size ${FONT_SIZE}
                    --no-compress
                    --font "${INPUT_FILE}"
                    --range "${FONT_RANGE}"
                    --format lvgl
                    --lv-include "lvgl.h"
                    -o "${OUTPUT_FILE}"
            # Insert extern declaration for fallback font after the lvgl.h include
            COMMAND sed -i
                    "s|#include \"lvgl.h\"|#include \"lvgl.h\"\\nextern const lv_font_t ${FONT_FALLBACK}\\;|"
                    "${OUTPUT_FILE}"
            # Patch fallback pointer from NULL to the fallback font
            COMMAND sed -i
                    "s|\\.fallback = NULL|\\.fallback = \\&${FONT_FALLBACK}|"
                    "${OUTPUT_FILE}"
            DEPENDS ${INPUT_FILE}
            COMMENT "Generating LVGL font: ${FONT_OUTPUT_NAME} (${FONT_SIZE}px, ${FONT_BPP}bpp, fallback=${FONT_FALLBACK})"
            VERBATIM
        )
    else()
        add_custom_command(
            OUTPUT  ${OUTPUT_FILE}
            COMMAND ${NPX_EXECUTABLE} -y lv_font_conv
                    --bpp ${FONT_BPP}
                    --size ${FONT_SIZE}
                    --no-compress
                    --font "${INPUT_FILE}"
                    --range "${FONT_RANGE}"
                    --format lvgl
                    --lv-include "lvgl.h"
                    -o "${OUTPUT_FILE}"
            DEPENDS ${INPUT_FILE}
            COMMENT "Generating LVGL font: ${FONT_OUTPUT_NAME} (${FONT_SIZE}px, ${FONT_BPP}bpp)"
            VERBATIM
        )
    endif()

    # Accumulate generated sources into parent scope list
    set(GENERATED_FONT_SOURCES ${GENERATED_FONT_SOURCES} ${OUTPUT_FILE} PARENT_SCOPE)
endfunction()

# -----------------------------------------------------------------------
# FontAwesome Solid — icon glyphs for LV_SYMBOL replacements
# (declared FIRST so the Roboto fonts below can reference them as fallback)
#
# Codepoints:
#   0xF015 = HOME (house)
#   0xF071 = WARNING (triangle-exclamation)
#   0xF1E6 = PLUG (replaces FA5 USB 0xF287, not present in FA7)
#   0xF11C = KEYBOARD
# -----------------------------------------------------------------------
set(FA_SOLID_TTF "fa-solid-900.ttf")
set(FA_RANGE "0xF015,0xF071,0xF1E6,0xF11C")

add_lvgl_font(OUTPUT_NAME fa_solid_18  FONT_FILE ${FA_SOLID_TTF}  SIZE 18  BPP 4  RANGE ${FA_RANGE})
add_lvgl_font(OUTPUT_NAME fa_solid_20  FONT_FILE ${FA_SOLID_TTF}  SIZE 20  BPP 4  RANGE ${FA_RANGE})
add_lvgl_font(OUTPUT_NAME fa_solid_28  FONT_FILE ${FA_SOLID_TTF}  SIZE 28  BPP 4  RANGE ${FA_RANGE})

# -----------------------------------------------------------------------
# Roboto — proportional UI font (replaces Montserrat)
# Each size that displays LV_SYMBOL glyphs gets the matching FA fallback.
# -----------------------------------------------------------------------
set(ROBOTO_TTF "Roboto-VariableFont.ttf")

add_lvgl_font(OUTPUT_NAME roboto_14  FONT_FILE ${ROBOTO_TTF}  SIZE 14  BPP 4  RANGE "32-127")
add_lvgl_font(OUTPUT_NAME roboto_18  FONT_FILE ${ROBOTO_TTF}  SIZE 18  BPP 4  RANGE "32-127"  FALLBACK fa_solid_18)
add_lvgl_font(OUTPUT_NAME roboto_20  FONT_FILE ${ROBOTO_TTF}  SIZE 20  BPP 4  RANGE "32-127"  FALLBACK fa_solid_20)
add_lvgl_font(OUTPUT_NAME roboto_28  FONT_FILE ${ROBOTO_TTF}  SIZE 28  BPP 4  RANGE "32-127"  FALLBACK fa_solid_28)

# -----------------------------------------------------------------------
# JetBrains Mono SemiBold — monospaced readout font
# -----------------------------------------------------------------------
set(JBMONO_TTF "JetBrainsMono-SemiBold.ttf")

add_lvgl_font(OUTPUT_NAME jetbrainsmono_20_4bpp  FONT_FILE ${JBMONO_TTF}  SIZE 20  BPP 4  RANGE "32-126")
add_lvgl_font(OUTPUT_NAME jetbrainsmono_24       FONT_FILE ${JBMONO_TTF}  SIZE 24  BPP 4  RANGE "32-126")
add_lvgl_font(OUTPUT_NAME jetbrainsmono_48       FONT_FILE ${JBMONO_TTF}  SIZE 48  BPP 4  RANGE "32-126")
