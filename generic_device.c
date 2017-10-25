/*
 * generic_device.c
 *
 *  Created on: Oct 25, 2017
 *      Author: Konrad Traczyk
 */

#include "qom/object.h"

#define TYPE_STM32_GENERIC_DEBUG_DEVICE     TYPE_STM32_PREFIX "gen-deb-dev" TYPE_PERIPHERAL_SUFFIX

static const TypeInfo generic_debug_device_type_info =
{
    .name = TYPE_STM32_GENERIC_DEBUG_DEVICE,
    .parent = TYPE_STM32_GPIO_PARENT,
    .instance_init = generic_debug_device_instance_init_callback,
    .instance_size = sizeof(STM32GPIOState),
    .class_init = generic_debug_device_class_init_callback,
    .class_size = sizeof(STM32GPIOClass)
/**/
};

void register_debug_device_type()
{
    type_register_static(&generic_debug_device_type_info);
}

void generic_debug_device_instance_init_callback()
{

}

void generic_debug_device_class_init_callback()
{

}

Object* generic_debug_device_create(Object *parent)
{
    char child_name[10];
    snprintf(child_name, sizeof(child_name) - 1, "GenDebDev");
    // Passing a local string is ok.
    Object *gpio = cm_object_new(parent, child_name, TYPE_STM32_GPIO);
    int i;

    object_property_set_int(gpio, index, "port-index", NULL);

    cm_object_realize(gpio);

    for (i = 0; i < STM32_GPIO_PIN_COUNT; ++i) {
        /* Connect GPIO outgoing to EXTI incoming. */
        cm_irq_connect(DEVICE(gpio), STM32_IRQ_GPIO_EXTI_OUT, i,
                cm_device_by_name(DEVICE_PATH_STM32_EXTI), STM32_IRQ_EXTI_IN,
                i);
    }

    return gpio;
}

type_init(register_debug_device_type)
