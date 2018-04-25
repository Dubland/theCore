/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

//! \file
//! \brief HM-10 usage example

#include <aux/generated.hpp>
#include <ecl/iostream.hpp>

#include <dev/serial.hpp>
#include <dev/hm10.hpp>

using hm10 = ecl::hm10_sync<ecl::hm10_uart>;

// Basic, low-level initialization
extern "C" void board_init()
{
    // Here, we only initialize GPIOs.
    gpio_init_generated();
}

int main()
{
    // Greeting.
    ecl::cout << "Starting HM-10 Bluetooth example" << ecl::endl;

    while (1) {
        // Data to send and its size.
        uint8_t data[] = "Hello, World!";
        size_t data_size = sizeof(data);

        // Initialize HM10 driver.
        hm10::init();

        // Set PIN to 000000 during pairing.
        hm10::set_pin(0);

        // Send data via Bluetooth.
        // When smartphone connected, data will be displayed as plain string.
        hm10::data_send(data, data_size);

        // Display how much data was sent.
        ecl::cout << "Bytes sent: " << data_size << ecl::endl;

        // Sleep some time.
        ecl::wait_for(std::chrono::seconds(1));
    }

    return 0;
}
