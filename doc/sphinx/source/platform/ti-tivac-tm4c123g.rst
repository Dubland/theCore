
Texas Instruments Tiva C TM4C123G
---------------------------------

Tiva C TM4C123G is a Cortex-M4 based microcontroller from Texas Instruments.

:Module location:
    ``platform/tm4c``
:Supported devices:
    TM4C123GH6PM
:Data sheets/Reference manuals:
    `TM4C123GH6PM <TM4C123GH6PM datasheet>`_

.. _`TM4C123GH6PM datasheet`: http://www.ti.com/lit/ds/spms376e/spms376e.pdf

.. important:: The platform is configured by theCore configurator.
    To make sure you are familiar with it, check the :ref:`theCore_Configurator`
    section before going any further.

Minimal configuration JSON file for the TM4C MCU must contain the platform
object with following fields:

  * ``name`` - platform name (simply ``tm4c``)
  * ``device`` - desired device ID. Depends on MCU.

.. important:: Currently, only ``TM4C123GH6PM`` device is supported.

For example, the basic configuration can look like that:

.. code-block:: json
   :linenos:

    {
        "platform": {
            "name": "tm4c",
            "device": "TM4C123GH6PM"
        }
    }

To import all generated definitions into the application code, simply add following
line to your source:

.. code-block:: cpp

    #include <aux/generated.hpp>

For more JSON configuration examples for TM4C platform, refer to the
:ref:`theCore_Examples` page.

Periphery overview
~~~~~~~~~~~~~~~~~~

TM4C MCU peripheries can be configured within the same JSON file.
Layout of the platform configuration object is described by JSON schema,
placed under ``platform/tm4c/schemas/tm4c.schema.json``:

.. literalinclude:: ../../../../platform/tm4c/schemas/tm4c.schema.json
    :language: json
    :linenos:

Each periphery configuration placed under a property with a relevant name.
The example below illustrates UART periphery being added to JSON config:

.. code-block:: json
   :linenos:

    {
        "platform": {
            "name": "tm4c",
            "device": "TM4C123GH6PM",
            "uart": [
                {
                    "id": "UART0",
                    "comment": "UART-over-USB output",
                    "alias": "my_uart"
                }
            ]
        }
    }

Note that to use any of periphery, corresponding pins must be configured too.
**Do not forget to include pin multiplexing configuration for each desired periphery.**
Proceed to the `TM4C Multiplexing`_ section for more details.

System timer
~~~~~~~~~~~~

:Driver sources:    ``platform/tm4c/export/aux/execution.hpp``
                    ``platform/tm4c/export/aux/execution.hpp``

JSON schema
+++++++++++

.. literalinclude:: ../../../../platform/tm4c/schemas/systmr.schema.json
    :language: json
    :linenos:

Example configuration
+++++++++++++++++++++

.. code-block:: json
   :linenos:

    {
        "platform": {
            "name": "tm4c",
            "systmr_cfg": {
                "source": "systick",
                "freq_hz": 50,
                "owner": "user"
            }
        }
    }

Properties:
.. note:: This section is under construction

Usage
+++++

.. note:: This section is under construction

UART
~~~~

:Driver sources:    ``platform/tm4c/export/aux/uart_bus.hpp``
:Template file:     ``platform/tm4c/templates/uart_cfg.in.hpp``

JSON schema
+++++++++++

.. literalinclude:: ../../../../platform/tm4c/schemas/uart.schema.json
    :language: json
    :linenos:

Example configuration
+++++++++++++++++++++

.. literalinclude:: tm4c/uart_example.json
    :language: json
    :linenos:

Example output
++++++++++++++

.. literalinclude:: ../_static/generated/tm4c/uart_example.hpp
    :language: cpp
    :linenos:
    :lines: 16-31

`Full TM4C UART example header <_static/generated/tm4c/uart_example.hpp>`_

Console
+++++++

To enable console in theCore, set ``console`` field to desired UART instance
and enable that instance, all via JSON:

.. code-block:: json
    :linenos:

    {
        "platform": {
            "name": "tm4c",
            "console": "UART0",
            "uart": [
                {
                    "id": "UART0",
                    "comment": "UART-over-USB console output"
                }
            ]
        }
    }

Check the :ref:`theCore_Console` section for more details about theCore console
library.

Usage
+++++

.. note:: This section is under construction

.. _TM4C Multiplexing:

Pin multiplexing
~~~~~~~~~~~~~~~~

:Driver sources:    ``platform/tm4c/export/platform/pin_cfg.hpp``
:Template file:     ``platform/tm4c/templates/pin_mux.in.hpp``

JSON schema
+++++++++++

.. literalinclude:: ../../../../platform/tm4c/schemas/pinmux.schema.json
    :language: json
    :linenos:

Example configuration
+++++++++++++++++++++

.. literalinclude:: tm4c/pin_mux_example.json
    :language: json
    :linenos:

Example output
++++++++++++++

.. literalinclude:: ../_static/generated/tm4c/pin_mux_example.cpp
    :language: cpp
    :linenos:
    :lines: 13-54

`Full TM4C GPIO pin multiplexing source file <_static/generated/tm4c/pin_mux_example.cpp>`_

Usage
+++++

.. note:: This section is under construction

GPIO aliases
~~~~~~~~~~~~

:Driver sources:    ``platform/tm4c/export/platform/gpio_device.hpp``
:Template file:     ``platform/tm4c/templates/gpio_cfg.in.hpp``

JSON schema
+++++++++++

.. literalinclude:: ../../../../platform/tm4c/schemas/gpio_alias.schema.json
    :language: json
    :linenos:

Example configuration
+++++++++++++++++++++

.. literalinclude:: tm4c/gpio_alias_example.json
    :language: json
    :linenos:

Example output
++++++++++++++

.. literalinclude:: ../_static/generated/tm4c/gpio_alias_example.hpp
    :language: cpp
    :linenos:
    :lines: 14-26

`Full TM4C GPIO alias example header <_static/generated/tm4c/gpio_alias_example.hpp>`_

Usage
+++++

.. note:: This section is under construction


External interrupts
~~~~~~~~~~~~~~~~~~~

.. note:: This section is under construction
