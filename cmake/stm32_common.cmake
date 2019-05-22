function(stm32_set_linker_script TARGET SCRIPT_FILE)
    get_target_property(TARGET_LD_FLAGS ${TARGET} LINK_FLAGS)
    if(TARGET_LD_FLAGS)
        set(TARGET_LD_FLAGS "\"-T${CMAKE_CURRENT_SOURCE_DIR}/${SCRIPT_FILE}\" -L${PROJECT_SOURCE_DIR}/lib/ChibiOS/os/common/startup/ARMCMx/compilers/GCC/ld/ -Wl,--as-needed,--gc-sections ${TARGET_LD_FLAGS} -nostartfiles")
    else()
        set(TARGET_LD_FLAGS "\"-T${CMAKE_CURRENT_SOURCE_DIR}/${SCRIPT_FILE}\" -L${PROJECT_SOURCE_DIR}/lib/ChibiOS/os/common/startup/ARMCMx/compilers/GCC/ld/ -Wl,--as-needed,-gc-sections -nostartfiles")
    ENDIF()
    set_target_properties(${TARGET} PROPERTIES LINK_FLAGS ${TARGET_LD_FLAGS})
    set_target_properties(${TARGET} PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${SCRIPT_FILE})
endfunction()

function(stm32_make_bin ELF_FILE)
    get_filename_component(BASENAME ${ELF_FILE} NAME_WE)
    set(BIN_FILE ${BASENAME}.bin)
    add_custom_command(OUTPUT ${BIN_FILE}
        COMMAND ${CMAKE_OBJCOPY} ARGS -O binary ${ELF_FILE} ${BIN_FILE}
        DEPENDS ${ELF_FILE}
        COMMENT "Generating ${BIN_FILE}"
    )

endfunction()

function(stm32_dfu_upload ELF_FILE)
    stm32_make_bin(${ELF_FILE})
    get_filename_component(BASENAME ${ELF_FILE} NAME_WE)
    set(BIN_FILE ${BASENAME}.bin)
    add_custom_target(dfu 
        dfu-util -a 0 -d 0483:df11 --dfuse-address 0x08000000 -D ${BIN_FILE}
    )
endfunction()

