/*
 * generic_device.h
 *
 *  Created on: Oct 25, 2017
 *      Author: Konrad Traczyk
 */

#ifndef PULL_GENERIC_DEVICE_H_
#define PULL_GENERIC_DEVICE_H_

#include <hw/cortexm/peripheral.h>
#include <hw/cortexm/stm32/capabilities.h>

#define GENERIC_DEBUG_DEVICE_BUFFER_ADDRESS         (0x50000000)
#define GENERIC_DEBUG_DEVICE_BUFFER_SIZE            (64*1024)

#define IRQ_LISTEN_PORT_NUM                         (7924)
#define IRQ_MAX_CONNECTIONS_NUN                     (1)
#define READ_BUFFER_SIZE                            (256)


#define TYPE_STM32_GENERIC_DEBUG_DEVICE     TYPE_STM32_PREFIX "gen-deb-dev" TYPE_PERIPHERAL_SUFFIX

typedef struct {
    // private:
    PeripheralClass parent_class;
    // public:

    // None, so far.
} STM32GenericDebugDeviceClass;

// Class definitions.
#define STM32_GENERIC_DEVICE_GET_CLASS(obj) \
    OBJECT_GET_CLASS(STM32GenericDebugDeviceClass, (obj), TYPE_STM32_GENERIC_DEBUG_DEVICE)
#define STM32_GENERIC_DEVICE_CLASS(klass) \
    OBJECT_CLASS_CHECK(STM32GenericDebugDeviceClass, (klass), TYPE_STM32_GENERIC_DEBUG_DEVICE)

// ----------------------------------------------------------------------------


// Instance definitions.
#define GENERIC_DEVICE_STATE(obj) \
    OBJECT_CHECK(GenericDeviceState_t, (obj), TYPE_STM32_GENERIC_DEBUG_DEVICE)

typedef struct
{
    // private:
    PeripheralState parent_obj;
    // public:

    const STM32Capabilities *capabilities;

    CortexMNVICState *nvic;

    Object*     cpuSideBuffer;
    uint32_t    cpuSideBufferSize;

}GenericDeviceState_t;


typedef struct
{
    uint32_t address;
    uint32_t wordCount;
    uint8_t  wordSize;
    void*    data;
}buffer_header_t;
#endif /* PULL_GENERIC_DEVICE_H_ */
