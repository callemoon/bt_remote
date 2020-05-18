/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/DiscoveredCharacteristic.h"
#include "ble/DiscoveredService.h"
#include "ble/gap/Gap.h"
#include "ble/gap/AdvertisingDataParser.h"
//#include "pretty_printer.h"


unsigned char address[6] = {0xaf, 0x82, 0x01, 0xa9, 0x52, 0xb4};    // mac address of unit to connect to

static EventQueue event_queue(/* event count */ 10 * EVENTS_EVENT_SIZE);

static DiscoveredCharacteristic joystick_characteristic;
static bool trigger_led_characteristic = false;

Serial pc(p35, p42);

AnalogIn   ain0(p4);
AnalogIn   ain1(p5);

void print_error(int code, const char *str)
{
    pc.printf(str);
}

void print_mac_address()
{
   pc.printf("mac");  
}

void service_discovery(const DiscoveredService *service) {    
}

void update_led_characteristic(void) {
    //if (!BLE::Instance().gattClient().isServiceDiscoveryActive()) {
    //    led_characteristic.read();
    //}

    char buf[16];

    int a0 = ain0.read_u16();
    int a1 = ain1.read_u16();

    sprintf(buf, "%4x %4x\r\n", a0, a1);
    joystick_characteristic.write(strlen(buf), (const uint8_t *)buf);
}

void characteristic_discovery(const DiscoveredCharacteristic *characteristicP) {
    if (characteristicP->getUUID().getShortUUID() == 0xffe1)
    {
        joystick_characteristic = *characteristicP;
    }
}

void discovery_termination(ble::connection_handle_t connectionHandle) {
    printf("terminated SD for handle %u\r\n", connectionHandle);
    if (trigger_led_characteristic) {
        trigger_led_characteristic = false;
        event_queue.call(update_led_characteristic);
    }

    event_queue.call(update_led_characteristic);
}

void trigger_toggled_write(const GattReadCallbackParams *response) {
}

void trigger_read(const GattWriteCallbackParams *response) {
}

class LEDBlinkerDemo : ble::Gap::EventHandler {
public:
    LEDBlinkerDemo(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),
        _alive_led(LED1, 1),
        _actuated_led(LED2, 0),
        _is_connecting(false) { }

    ~LEDBlinkerDemo() { }

    void start() {
        _ble.gap().setEventHandler(this);

        _ble.init(this, &LEDBlinkerDemo::on_init_complete);

        _event_queue.call_every(50, this, &LEDBlinkerDemo::blink);

        _event_queue.dispatch_forever();
    }

private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        if (params->error != BLE_ERROR_NONE) {
            printf("Ble initialization failed.");
            return;
        }

        print_mac_address();

        _ble.gattClient().onDataRead(trigger_toggled_write);
        _ble.gattClient().onDataWritten(trigger_read);

        print_error(0, "starting scan\r\n");

        ble::ScanParameters scan_params;

        _ble.gap().setScanParameters(scan_params);
        _ble.gap().startScan();
    }

    void blink() {
        _alive_led = !_alive_led;

        event_queue.call(update_led_characteristic);
    }

private:
    /* Event handler */

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
        print_error(0, "disconnect complete\r\n");
        
        _ble.gap().startScan();
        _is_connecting = false;
    }

    void onConnectionComplete(const ble::ConnectionCompleteEvent& event) {
        if (event.getOwnRole() == ble::connection_role_t::CENTRAL) {
            _ble.gattClient().onServiceDiscoveryTermination(discovery_termination);
            _ble.gattClient().launchServiceDiscovery(
                event.getConnectionHandle(),
                service_discovery,
                characteristic_discovery
            );
        } else {
            _ble.gap().startScan();
        }
        _is_connecting = false;
    }

    void onAdvertisingReport(const ble::AdvertisingReportEvent &event) {
        /* don't bother with analysing scan result if we're already connecting */
        if (_is_connecting) {
            return;
        }

        ble::AdvertisingDataParser adv_data(event.getPayload());

        /* parse the advertising payload, looking for a discoverable device */
        while (adv_data.hasNext()) {
            ble::AdvertisingDataParser::element_t field = adv_data.next();


            if(!memcmp(event.getPeerAddress().data(), address, 6))
            /* connect to a discoverable device */
            {
                ble_error_t error = _ble.gap().stopScan();

                if (error) {
                    print_error(error, "Error caused by Gap::stopScan");
                    return;
                }

                const ble::ConnectionParameters connection_params;
            
                error = _ble.gap().connect(
                    event.getPeerAddressType(),
                    event.getPeerAddress(),
                    connection_params
                );

                if (error) {
                    _ble.gap().startScan();
                    return;
                }

                /* we may have already scan events waiting
                 * to be processed so we need to remember
                 * that we are already connecting and ignore them */
                _is_connecting = true;

                return;
            }
        }
    }

private:
    BLE &_ble;
    events::EventQueue &_event_queue;
    DigitalOut _alive_led;
    DigitalOut _actuated_led;
    bool _is_connecting;
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

int main()
{
    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);

    LEDBlinkerDemo demo(ble, event_queue);
    demo.start();

    return 0;
}
