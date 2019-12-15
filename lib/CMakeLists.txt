add_subdirectory(abseil)

# UAVCAN stuff

set(DSDLC_INPUTS uavcan/dsdl/uavcan ../uavcan_data_types/cvra)
set(DSDLC_OUTPUT ${CMAKE_BINARY_DIR}/dsdlc_generated)

# Build list of UAVCAN inputs and outputs
execute_process(
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/list_uavcan_dsdl.py ${DSDLC_INPUTS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE UAVCAN_FILES
    RESULT_VARIABLE RETURN_VALUE
)
if (NOT RETURN_VALUE EQUAL 0)
    message(FATAL_ERROR "Failed to get the dependencies (error ${RETURN_VALUE})")
endif()

execute_process(
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/list_uavcan_dsdl.py ${DSDLC_INPUTS} --output-dir ${DSDLC_OUTPUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE UAVCAN_HEADERS
    RESULT_VARIABLE RETURN_VALUE
)
if (NOT RETURN_VALUE EQUAL 0)
    message(FATAL_ERROR "Failed to get the dependencies (error ${RETURN_VALUE})")
endif()

message(STATUS "Found the following UAVCAN files...")
foreach(F ${UAVCAN_FILES})
    message(STATUS ${F})
endforeach()

message(STATUS "Which generate the following headers...")
foreach(F ${UAVCAN_HEADERS})
    message(STATUS ${F})
endforeach()


add_custom_command(COMMAND ${PYTHON} uavcan/libuavcan/dsdl_compiler/libuavcan_dsdlc ${DSDLC_INPUTS} -O${DSDLC_OUTPUT}
    OUTPUT ${UAVCAN_HEADERS}
    DEPENDS ${UAVCAN_FILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Running dsdl compiler")

add_library(uavcan
    uavcan/libuavcan/src/driver/uc_can.cpp
    uavcan/libuavcan/src/marshal/uc_bit_array_copy.cpp
    uavcan/libuavcan/src/marshal/uc_bit_stream.cpp
    uavcan/libuavcan/src/marshal/uc_float_spec.cpp
    uavcan/libuavcan/src/marshal/uc_scalar_codec.cpp
    uavcan/libuavcan/src/node/uc_generic_publisher.cpp
    uavcan/libuavcan/src/node/uc_generic_subscriber.cpp
    uavcan/libuavcan/src/node/uc_global_data_type_registry.cpp
    uavcan/libuavcan/src/node/uc_scheduler.cpp
    uavcan/libuavcan/src/node/uc_service_client.cpp
    uavcan/libuavcan/src/node/uc_timer.cpp
    uavcan/libuavcan/src/protocol/uc_dynamic_node_id_client.cpp
    uavcan/libuavcan/src/protocol/uc_node_status_provider.cpp
    uavcan/libuavcan/src/transport/uc_can_acceptance_filter_configurator.cpp
    uavcan/libuavcan/src/transport/uc_can_io.cpp
    uavcan/libuavcan/src/transport/uc_crc.cpp
    uavcan/libuavcan/src/transport/uc_dispatcher.cpp
    uavcan/libuavcan/src/transport/uc_frame.cpp
    uavcan/libuavcan/src/transport/uc_outgoing_transfer_registry.cpp
    uavcan/libuavcan/src/transport/uc_transfer.cpp
    uavcan/libuavcan/src/transport/uc_transfer_buffer.cpp
    uavcan/libuavcan/src/transport/uc_transfer_listener.cpp
    uavcan/libuavcan/src/transport/uc_transfer_receiver.cpp
    uavcan/libuavcan/src/transport/uc_transfer_sender.cpp
    uavcan/libuavcan/src/uc_data_type.cpp
    uavcan/libuavcan/src/uc_dynamic_memory.cpp
    uavcan/libuavcan/src/uc_error.cpp
    ${UAVCAN_HEADERS}
)

target_include_directories(uavcan PUBLIC uavcan/libuavcan/include)
target_compile_definitions(uavcan PUBLIC "UAVCAN_CPP_VERSION=UAVCAN_CPP11;UAVCAN_TOSTRING=0;UAVCAN_DEBUG=0")
target_include_directories(uavcan PUBLIC ${DSDLC_OUTPUT})

add_library(uavcan_linux INTERFACE)
target_include_directories(uavcan_linux INTERFACE uavcan/libuavcan_drivers/linux/include/)

if (${CMAKE_CROSSCOMPILING})
    add_library(uavcan_stm32
        uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_clock.cpp
        uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_can.cpp
        uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_thread.cpp
    )
    target_include_directories(uavcan_stm32 PUBLIC uavcan/libuavcan_drivers/stm32/driver/include)
    target_compile_definitions(uavcan_stm32 PUBLIC "UAVCAN_STM32_CHIBIOS=1;UAVCAN_STM32_NUM_IFACES=1")

    target_link_libraries(uavcan_stm32 uavcan chibios)
endif()

add_subdirectory(arm-cortex-tools)
add_subdirectory(aversive)
add_subdirectory(chibios-syscalls)
add_subdirectory(cmp)
add_subdirectory(cmp_mem_access)
add_subdirectory(crc)
add_subdirectory(error)
add_subdirectory(fatfs)
add_subdirectory(filter)
add_subdirectory(goap)
add_subdirectory(lwip)
add_subdirectory(msgbus)
add_subdirectory(nanopb)
add_subdirectory(parameter)
add_subdirectory(parameter_flash_storage)
add_subdirectory(pid)
add_subdirectory(quadramp)
add_subdirectory(test-runner)
add_subdirectory(timestamp)
add_subdirectory(trace)

add_library(Eigen INTERFACE)
target_include_directories(Eigen INTERFACE eigen/)

