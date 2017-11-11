/*
 * generic_device.c
 *
 *  Created on: Oct 25, 2017
 *      Author: Konrad Traczyk
 */

#include <stdint-gcc.h>
#include <stdbool.h>
#include "generic_device.h"
#include "qom/object.h"
#include "hw/cortexm/peripheral.h"
#include "hw/cortexm/nvic.h"
#include "hw/cortexm/stm32/capabilities.h"
#include "hw/cortexm/stm32/mcu.h"
#include "hw/cortexm/mcu.h"
#include "hw/cortexm/svd.h"
#include <sys/socket.h>
#include <errno.h>
#include "qemu/thread.h"
#include "exec/memory.h"
#include <pthread.h>
#include <sys/syscall.h>

void generic_debug_device_instance_init_callback(Object *obj);
void generic_debug_device_class_init_callback(ObjectClass *klass, void *data);

uint16_t createdPeripheralCnt = 0;
char peripheralNames[GENERIC_PERIPHERALS_COUNT][16] =
{
        "DBG_DEV",
        "QEMU_SPI1",
        "QEMU_SPI2",
        "QEMU_SPI3",
        "QEMU_UART1",
        "QEMU_I2S",
        "QEMU_GPIO",
        "QEMU_EXTI",
        "QEMU_ADC"
};

bool genericPeripheralServerUsed = false;

GenericDeviceState_t* peripheralArray[GENERIC_PERIPHERALS_COUNT];

static const TypeInfo generic_debug_device_type_info =
{
    .name = TYPE_STM32_GENERIC_DEBUG_DEVICE,
    .parent = TYPE_PERIPHERAL,
    .instance_init = generic_debug_device_instance_init_callback,
    .instance_size = sizeof(GenericDeviceState_t),
    .class_init = generic_debug_device_class_init_callback,
    .class_size = sizeof(STM32GenericDebugDeviceClass)
/**/
};

// Thread used to trigger IRQs on the CPU
static QemuThread               qemu_irq_thread;
int                             qemuTcpConnFd;
static CortexMNVICState*        _nvic;

void tcp_thread_init()
{
    struct sockaddr_in serv_addr;
    int socketFd = socket(AF_INET, SOCK_STREAM, 0);

    if (socketFd < 0)
    {
        printf("QEMU Log: Error during socket opening. Errno: %d\n", errno);
        exit(1);
        return;
    }

    memset(&serv_addr, 0 , sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(IRQ_LISTEN_PORT_NUM);

    if (bind(socketFd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("QEMU Log: !ERROR! Could not bind to the socket\n");
        exit(1);

        return;
    }

    if (listen(socketFd, MAX_PERIPH_SERVER_CONN_NUM) < 0)
    {
        printf("QEMU Log: !ERROR! Could not listen on the socket\n");
        exit(1);

        return;
    }

    qemuTcpConnFd = accept(socketFd, (struct sockaddr*)NULL, NULL);
    if (qemuTcpConnFd < 0)
    {
        printf("QEMU Log: Error during connection accept. Errno: %d\n", errno);
        exit(1);

        return;
    }

    qemu_thread_create(&qemu_irq_thread, "GenDbgTCP",
            (void *(*)(void*)) tcp_worker_function, NULL, 0);
    pthread_setname_np(qemu_irq_thread.thread, "GenDbgDevTCP");
}



void tcp_worker_function()
{
    char readBuffer[READ_BUFFER_SIZE];
    memset(readBuffer, 0, sizeof(readBuffer));

    peripheral_response_header_t response;

    while (1)
    {
        memset(&response, 0, sizeof(peripheral_response_header_t));

        int readBytesCount = read(qemuTcpConnFd, readBuffer, READ_BUFFER_SIZE);
        if (readBytesCount == 0)
        {
            printf("QEMU Log: Peripheral server disconnected. Exiting...");
            exit(1);
        }

        readBuffer[readBytesCount] = '\0';
        printf("QEMU Log: Received data: %s\n", readBuffer);

        memcpy(&response, readBuffer, sizeof(peripheral_response_header_t));

        if (peripheralArray[response.peripheralIndex]->isWaitingForDeviceRead)
        {
            if (peripheralArray[response.peripheralIndex]->cpuAddressRegister == response.address)
            {
                cpu_physical_memory_write(response.address, readBuffer + sizeof(peripheral_response_header_t), response.wordSize*response.wordCount);
                peripheralArray[response.peripheralIndex]->isWaitingForDeviceRead = false;
            }
        }
        else
        {
            // Trigger interrupt
            cortexm_nvic_set_pending_interrupt(_nvic, response.irqNum);
        }
    }
}

void tcp_write_to_peripheral_server(void* data, uint32_t dataSize)
{
    pid_t tid = syscall(SYS_gettid);
    printf("QEMU Log: Thread ID: %u has sent data via TCP to Peripheral Server\n", tid);

    write(qemuTcpConnFd, data, dataSize);
    fsync(qemuTcpConnFd);
}

void generic_debug_device_write_callback(Object *reg, Object *periph,
        uint32_t addr, uint32_t offset, unsigned size,
        peripheral_register_t value, peripheral_register_t full_value)
{
    buffer_header_t header;

    GenericDeviceState_t *state = GENERIC_DEVICE_STATE(periph);
    PeripheralRegisterState *regState = PERIPHERAL_REGISTER_STATE(reg);
    printf("QEMU Log: Write callback for \"%s->%s\"\n", state->deviceName, regState->name);

    uint32_t sr = peripheral_register_get_raw_value(state->cpuSendRegister);
    header.peripheralIndex = state->peripheralIndex;
    header.isReadRegister = peripheral_register_get_raw_value(state->cpuIsReadRegister);
    header.address = peripheral_register_get_raw_value(state->cpuAddressRegister);
    header.wordSize = peripheral_register_get_raw_value(state->cpuWordSizeRegister);
    header.wordCount = peripheral_register_get_raw_value(state->cpuWordCountRegister);
    header.dataPtr = peripheral_register_get_raw_value(state->cpuDataPtrRegister);

    // Mark the device to be waiting for the response;
    if(header.isReadRegister)
    {
        state->isWaitingForDeviceRead = true;
    }else
    {
        state->isWaitingForDeviceRead = false;
    }

    uint16_t dataSize = header.wordSize*header.wordCount + sizeof(header) + 1;
    char* data = (char*)malloc(dataSize);
    memset(data, 0 , dataSize);

    memcpy(data, &header, sizeof(header));


    // Get data from the CPU's RAM memory
    cpu_physical_memory_read(header.dataPtr, data + sizeof(header), header.wordSize*header.wordCount);

    tcp_write_to_peripheral_server(data, sizeof(header) + header.wordCount*header.wordSize);
    free(data);

    while(state->isWaitingForDeviceRead)
    {

    }

}

void generic_debug_device_read_callback(Object *reg, Object *periph,
        uint32_t addr, uint32_t offset, unsigned size)
{
    GenericDeviceState_t *state = GENERIC_DEVICE_STATE(periph);
}

void generic_debug_device_realize_callback(DeviceState *dev, Error **errp)
{
    // Call parent realize().
    if (!cm_device_parent_realize(dev, errp, TYPE_STM32_GENERIC_DEBUG_DEVICE)) {
        return;
    }
    const char* jsonName = qemu_find_file(QEMU_FILE_TYPE_DEVICES, "generic_device_description.json");
    JSON_Value *value = json_parse_file(jsonName);
    JSON_Object *svd_json = json_value_get_object(value);

    STM32MCUState *mcu = stm32_mcu_get();
    CortexMState *cm_state = CORTEXM_MCU_STATE(mcu);

    GenericDeviceState_t *state = GENERIC_DEVICE_STATE(dev);

    // First thing first: get capabilities from MCU, needed everywhere.
    state->capabilities = mcu->capabilities;

    Object *obj = OBJECT(dev);

    state->nvic = CORTEXM_NVIC_STATE(cm_state->nvic);

    state->peripheralIndex = createdPeripheralCnt;
    const char* periphName = peripheralNames[createdPeripheralCnt];
    printf("QEMU Log: Creating Device: %s\n", periphName);

    svd_set_peripheral_address_block(svd_json, periphName, obj);
    peripheral_create_memory_region(obj);

    // Must be defined before creating registers.
    cm_object_property_set_int(obj, 4, "register-size-bytes");
    // TODO: get it from MCU
    cm_object_property_set_bool(obj, true, "is-little-endian");

    JSON_Object *periph = svd_get_peripheral_by_name(svd_json, periphName);
    svd_add_peripheral_properties_and_children(obj, periph, svd_json);

    peripheral_prepare_registers(obj);

    strcpy(state->deviceName, periphName);

    state->cpuSendRegister = cm_object_get_child_by_name(obj, "SEND");
    state->cpuIsReadRegister = cm_object_get_child_by_name(obj, "READ_REGISTER");
    state->cpuAddressRegister = cm_object_get_child_by_name(obj, "ADDRESS");
    state->cpuWordSizeRegister = cm_object_get_child_by_name(obj, "WORD_SIZE");
    state->cpuWordCountRegister = cm_object_get_child_by_name(obj, "WORD_COUNT");
    state->cpuDataPtrRegister = cm_object_get_child_by_name(obj, "DATA_PTR");

    // Register callbacks.
    peripheral_register_set_post_read(state->cpuSendRegister,
            &generic_debug_device_read_callback);

    peripheral_register_set_post_write(state->cpuSendRegister,
            &generic_debug_device_write_callback);

    peripheralArray[createdPeripheralCnt++] = state;
    _nvic = state->nvic;
}

void generic_debug_device_instance_init_callback(Object *obj)
{
    GenericDeviceState_t *state = GENERIC_DEVICE_STATE(obj);

    state->cpuSendRegister = NULL;
}

void generic_debug_device_class_init_callback(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

//    dc->reset = generic_debug_device_reset_callback;
    dc->realize = generic_debug_device_realize_callback;


    PeripheralClass *per_class = PERIPHERAL_CLASS(klass);
//    per_class->is_enabled = generic_debug_device_is_enabled;
}

Object* generic_debug_device_create(Object *parent)
{
    char child_name[16];
    snprintf(child_name, sizeof(child_name) - 1, peripheralNames[createdPeripheralCnt]);

    // Passing a local string is ok.
    Object *genDbgDev = cm_object_new(parent, child_name, TYPE_STM32_GENERIC_DEBUG_DEVICE);

    cm_object_realize(genDbgDev);

    return genDbgDev;
}

void register_debug_device_type()
{
    type_register_static(&generic_debug_device_type_info);
}

type_init(register_debug_device_type)
