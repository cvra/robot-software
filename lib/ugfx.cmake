add_library(ugfx
    ugfx/src/gfx.c
    ugfx/src/gos/gos_chibios.c
    ugfx/src/gos/gos_linux.c
    ugfx/src/gos/gos_osx.c
    ugfx/src/gdriver/gdriver.c
    ugfx/src/gqueue/gqueue.c
    ugfx/src/gdisp/gdisp.c
    ugfx/src/gdisp/gdisp_fonts.c
    ugfx/src/gdisp/gdisp_pixmap.c
    ugfx/src/gdisp/gdisp_image.c
    ugfx/src/gdisp/gdisp_image_native.c
    ugfx/src/gdisp/gdisp_image_gif.c
    ugfx/src/gdisp/gdisp_image_bmp.c
    ugfx/src/gdisp/gdisp_image_jpg.c
    ugfx/src/gdisp/gdisp_image_png.c
    ugfx/src/gdisp/mcufont/mf_encoding.c
    ugfx/src/gdisp/mcufont/mf_font.c
    ugfx/src/gdisp/mcufont/mf_justify.c
    ugfx/src/gdisp/mcufont/mf_kerning.c
    ugfx/src/gdisp/mcufont/mf_rlefont.c
    ugfx/src/gdisp/mcufont/mf_bwfont.c
    ugfx/src/gdisp/mcufont/mf_scaledfont.c
    ugfx/src/gdisp/mcufont/mf_wordwrap.c
    ugfx/src/gevent/gevent.c
    ugfx/src/gtimer/gtimer.c
    ugfx/src/gwin/gwin.c
    ugfx/src/gwin/gwin_widget.c
    ugfx/src/gwin/gwin_wm.c
    ugfx/src/gwin/gwin_console.c
    ugfx/src/gwin/gwin_graph.c
    ugfx/src/gwin/gwin_button.c
    ugfx/src/gwin/gwin_slider.c
    ugfx/src/gwin/gwin_checkbox.c
    ugfx/src/gwin/gwin_image.c
    ugfx/src/gwin/gwin_label.c
    ugfx/src/gwin/gwin_radio.c
    ugfx/src/gwin/gwin_list.c
    ugfx/src/gwin/gwin_progressbar.c
    ugfx/src/gwin/gwin_container.c
    ugfx/src/gwin/gwin_frame.c
    ugfx/src/gwin/gwin_tabset.c
    ugfx/src/gwin/gwin_gl3d.c
    ugfx/src/gwin/gwin_keyboard.c
    ugfx/src/gwin/gwin_keyboard_layout.c
    ugfx/src/gwin/gwin_textedit.c

    ugfx/src/ginput/ginput.c
    ugfx/src/ginput/ginput_mouse.c
    ugfx/src/ginput/ginput_keyboard.c
    ugfx/src/ginput/ginput_keyboard_microcode.c
    ugfx/src/ginput/ginput_toggle.c
    ugfx/src/ginput/ginput_dial.c
    ugfx/src/gadc/gadc.c
    ugfx/src/gaudio/gaudio.c
    ugfx/src/gmisc/gmisc.c
    ugfx/src/gmisc/gmisc_arrayops.c
    ugfx/src/gmisc/gmisc_matrix2d.c
    ugfx/src/gmisc/gmisc_trig.c
    ugfx/src/gmisc/gmisc_hittest.c
    ugfx/src/gfile/gfile.c
    ugfx/src/gfile/gfile_fs_native.c
    ugfx/src/gfile/gfile_fs_ram.c
    ugfx/src/gfile/gfile_fs_rom.c
    ugfx/src/gfile/gfile_fs_fatfs.c
    ugfx/src/gfile/gfile_fs_petitfs.c
    ugfx/src/gfile/gfile_fs_mem.c
    ugfx/src/gfile/gfile_fs_chibios.c
    ugfx/src/gfile/gfile_fs_strings.c
    ugfx/src/gfile/gfile_printg.c
    ugfx/src/gfile/gfile_scang.c
    ugfx/src/gfile/gfile_stdio.c
    ugfx/src/gfile/gfile_fatfs_wrapper.c
    ugfx/src/gfile/gfile_fatfs_diskio_chibios.c
    ugfx/src/gfile/gfile_petitfs_wrapper.c
    ugfx/src/gfile/gfile_petitfs_diskio_chibios.c
    ugfx/src/gtrans/gtrans.c
)

target_include_directories(ugfx PUBLIC ugfx)

# Defined in other projects
target_link_libraries(ugfx gfxconf)

if (${CMAKE_CROSSCOMPILING})
    target_compile_definitions(ugfx PUBLIC "GFX_USE_OS_CHIBIOS=GFXON")
    target_link_libraries(ugfx chibios)
elseif (APPLE)
    # TODO: Test this
    target_compile_definitions(ugfx PUBLIC "GFX_USE_OS_OSX=GFXON")
elseif (UNIX) # Linux
    target_compile_definitions(ugfx PUBLIC "GFX_USE_OS_LINUX=GFXON")
else()
    message(FATAL_ERROR "Supported platforms: Linux and MacOS and ChibiOS")
endif()

add_library(stmpe610_driver
    ugfx/drivers/ginput/touch/STMPE610/gmouse_lld_STMPE610.c
)

target_link_libraries(stmpe610_driver ugfx)
