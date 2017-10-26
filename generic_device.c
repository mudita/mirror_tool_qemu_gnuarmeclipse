/*
 * generic_device.c
 *
 *  Created on: Oct 25, 2017
 *      Author: Konrad Traczyk
 */

#include "qom/object.h"
#include "hw/cortexm/peripheral.h"
#include "hw/cortexm/nvic.h"
#include "hw/cortexm/stm32/capabilities.h"
#include "hw/cortexm/stm32/mcu.h"
#include "hw/cortexm/mcu.h"
#include "generic_device.h"
#include <sys/socket.h>
#include <errno.h>
#include "qemu/thread.h"

void generic_debug_device_instance_init_callback(Object *obj);
void generic_debug_device_class_init_callback(ObjectClass *klass, void *data);

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
static QemuThread qemu_irq_thread;
int               qemuTcpConnFd;
static qemu_irq   _generalIrq;

static void tcp_thread_init()
{
    // Get the IRQ
    _generalIrq = qdev_get_gpio_in_named(cm_device_by_name("/machine/mcu/stm32/GPIOA"), "idr-in", 0);

    qemu_thread_create(&qemu_irq_thread, "app-irq-thread",
            (void *(*)(void*)) tcp_worker_function, NULL, 0);
}

void tcp_worker_function()
{
    char readBuffer[READ_BUFFER_SIZE];
    memset(readBuffer, 0, sizeof(readBuffer));

    struct sockaddr_in serv_addr;
    int socketFd = socket(AF_INET, SOCK_STREAM, 0);

    if (socketFd < 0)
    {
        printf("Error during socket opening. Errno: %d\n", errno);
        return;
    }

    memset(&serv_addr, 0 , sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(IRQ_LISTEN_PORT_NUM);

    bind(socketFd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

    listen(socketFd, IRQ_MAX_CONNECTIONS_NUN);

    while (1)
    {
        qemuTcpConnFd = accept(socketFd, (struct sockaddr*)NULL, NULL);

        if (connFd < 0)
        {
            printf("Error during connection accept. Errno: %d\n", errno);
            return;
        }

        while (1)
        {
            int readBytesCount = read(qemuTcpConnFd, readBuffer, READ_BUFFER_SIZE);
            readBuffer[readBytesCount] = '\0';
            printf("Received data: %s\n", readBuffer);

            // Trigger interrupt
            qemu_set_irq(_generalIrq, 1);
        }
    }
}

void generic_debug_device_write_callback()
{
    buffer_header_t header;
    uint8_t headerSize = sizeof(header.address) + sizeof(header.wordCount) + sizeof(header.wordSize);
    memcpy((void*)(&header), (void*)GENERIC_DEBUG_DEVICE_BUFFER_ADDRESS, headerSize);
    write(qemuTcpConnFd, GENERIC_DEBUG_DEVICE_BUFFER_ADDRESS, headerSize + header.wordCount*header.wordSize);
}

void generic_debug_device_read_callback()
{

}

void generic_debug_device_realize_callback(DeviceState *dev, Error **errp)
{
    // Call parent realize().
    if (!cm_device_parent_realize(dev, errp, TYPE_STM32_GENERIC_DEBUG_DEVICE)) {
        return;
    }

    STM32MCUState *mcu = stm32_mcu_get();
    CortexMState *cm_state = CORTEXM_MCU_STATE(mcu);

    GenericDeviceState_t *state = TYPE_STM32_GENERIC_DEBUG_DEVICE(dev);
    // First thing first: get capabilities from MCU, needed everywhere.
    state->capabilities = mcu->capabilities;

    Object *obj = OBJECT(dev);

    state->nvic = CORTEXM_NVIC_STATE(cm_state->nvic);

    cm_object_property_set_int(obj, GENERIC_DEBUG_DEVICE_BUFFER_ADDRESS, "mmio-address");
    cm_object_property_set_int(obj, GENERIC_DEBUG_DEVICE_BUFFER_SIZE, "mmio-size-bytes");
    // TODO: get it from MCU
    cm_object_property_set_bool(obj, true, "is-little-endian");

    peripheral_create_memory_regions(obj);

    const char* regi_name = "GEN_DEV";
    Object *reg = cm_object_new(obj, regi_name, TYPE_PERIPHERAL_REGISTER);


    // Store a local copy of the node name, for easier access.
    // Passing a parsed string is ok, it is copied.
    cm_object_property_set_str(reg, regi_name, "name");

    cm_object_property_set_int(reg, 0x00, "offset-bytes");
    cm_object_property_set_bool(obj, true, "is-readable");
    cm_object_property_set_bool(obj, true, "is-writable");

    cm_object_realize(reg);

    // Register callbacks.
    peripheral_register_set_post_read(state->cpuSideBuffer,
            &generic_debug_device_read_callback);
    peripheral_register_set_post_write(state->cpuSideBuffer,
            &generic_debug_device_write_callback);

    tcp_thread_init();
}

void generic_debug_device_instance_init_callback(Object *obj)
{
    GenericDeviceState_t *state = GENERIC_DEVICE_STATE(obj);

    state->cpuSideBuffer = GENERIC_DEBUG_DEVICE_BUFFER_ADDRESS;
    state->cpuSideBufferSize = GENERIC_DEBUG_DEVICE_BUFFER_SIZE;
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
    char child_name[10];
    snprintf(child_name, sizeof(child_name) - 1, "GenDebDev");
    // Passing a local string is ok.
    Object *genDbgDev = cm_object_new(parent, child_name, TYPE_STM32_GENERIC_DEBUG_DEVICE);
//    int i;


    cm_object_realize(genDbgDev);

//    for (i = 0; i < STM32_GPIO_PIN_COUNT; ++i) {
//        /* Connect GPIO outgoing to EXTI incoming. */
//        cm_irq_connect(DEVICE(gpio), STM32_IRQ_GPIO_EXTI_OUT, i,
//                cm_device_by_name(DEVICE_PATH_STM32_EXTI), STM32_IRQ_EXTI_IN,
//                i);
//    }

    return genDbgDev;
}

void register_debug_device_type()
{
    type_register_static(&generic_debug_device_type_info);
}

type_init(register_debug_device_type)
