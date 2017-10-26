/*
 * generic_device.c
 *
 *  Created on: Oct 25, 2017
 *      Author: Konrad Traczyk
 */

#include "qom/object.h"
#include "hw/cortexm/peripheral.h"
#include "generic_device.h"

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

void register_debug_device_type()
{
    type_register_static(&generic_debug_device_type_info);
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

    peripheral_create_memory_region(obj);

}

void generic_debug_device_instance_init_callback(Object *obj)
{
    GenericDeviceState_t *state = GENERIC_DEVICE_STATE(obj);

    state->bareMetalSideBuffer = GENERIC_DEBUG_DEVICE_BUFFER_ADDRESS;
    state->bareMetalSideBufferSize = GENERIC_DEBUG_DEVICE_BUFFER_SIZE;
}

void generic_debug_device_class_init_callback(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = generic_debug_device_reset_callback;
    dc->realize = generic_debug_device_realize_callback;


    PeripheralClass *per_class = PERIPHERAL_CLASS(klass);
    per_class->is_enabled = generic_debug_device_is_enabled;
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

type_init(register_debug_device_type)
