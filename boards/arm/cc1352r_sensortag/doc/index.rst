.. _cc1352r_sesortag:

CC1352R SensorTag
##################

Overview
********

The Texas Instruments CC1352R SensorTag |trade| (SENSORTAG-CC1352R) is a
development kit for the SimpleLink |trade| multi-Standard CC1352R wireless MCU.

See the `TI CC1352R SensorTag Product Page`_ for details.

.. figure:: img/cc1352r_sensortag.jpg
   :width: 400px
   :align: center
   :alt: TI CC1352R SensorTag

   Texas Instruments CC1352R SensorTag |trade|

Hardware
********

The CC1352R SensorTag |trade| development kit features the CC1352R wireless MCU.
The board is equipped with two LEDs, two push buttons and BoosterPack connectors
for expansion. It also includes an integrated (XDS110) debugger.

The CC13522 wireless MCU has a 48 MHz Arm |reg| Cortex |reg|-M4F SoC and an
integrated Sub-1 and 2.4 GHz transceiver supporting multiple protocols including
Bluetooth |reg| Low Energy and IEEE |reg| 802.15.4.

See the `TI CC1352R Product Page`_ for additional details.

Supported Features
==================

The CC1352R SensorTag board configuration supports the following hardware
features:

+-------------+------------+----------------------+
| Interface   | Controller | Driver/Component     |
+=============+============+======================+
| GPIO        | on-chip    | gpio                 |
+-------------+------------+----------------------+
| NVIC        | on-chip    | arch/arm             |
+-------------+------------+----------------------+
| PINMUX      | on-chip    | pinmux               |
+-------------+------------+----------------------+
| UART        | on-chip    | serial               |
+-------------+------------+----------------------+
| I2C         | on-chip    | i2c                  |
+-------------+------------+----------------------+
| SPI         | on-chip    | spi                  |
+-------------+------------+----------------------+
| DIO23       | off-chip   | DRV5032              |
+-------------+------------+----------------------+
| I2C         | off-chip   | HDC2080              |
+-------------+------------+----------------------+
| I2C         | off-chip   | OPT3001              |
+-------------+------------+----------------------+
| SPI         | off-chip   | ADXL362              |
+-------------+------------+----------------------+

Other hardware features are not supported by the Zephyr kernel.

Connections and IOs
===================

All I/O signals are accessible from the BoosterPack connectors. Pin function
aligns with the SensorTag standard.

+-------+-----------+---------------------+
| Pin   | Function  | Usage               |
+=======+===========+=====================+
| DIO3  | GPIO      | GPIO / PWM1         |
+-------+-----------+---------------------+
| DIO4  | I2C_MSSCL | I2C SCL             |
+-------+-----------+---------------------+
| DIO5  | I2C_MSSDA | I2C SDA             |
+-------+-----------+---------------------+
| DIO6  | GPIO      | Red LED             |
+-------+-----------+---------------------+
| DIO7  | GPIO      | Green LED           |
+-------+-----------+---------------------+
| DIO8  | SSI0_RX   | SPI MISO            |
+-------+-----------+---------------------+
| DIO9  | SSI0_TX   | SPI MOSI            |
+-------+-----------+---------------------+
| DIO10 | SSI0_CLK  | SPI CLK             |
+-------+-----------+---------------------+
| DIO11 | SSIO_CS   | SPI CS              |
+-------+-----------+---------------------+
| DIO12 | UART0_RX  | UART RXD            |
+-------+-----------+---------------------+
| DIO13 | UART0_TX  | UART TXD            |
+-------+-----------+---------------------+
| DIO14 | GPIO      | Button 2            |
+-------+-----------+---------------------+
| DIO15 | GPIO      | Button 1            |
+-------+-----------+---------------------+
| DIO16 |           | JTAG TDO            |
+-------+-----------+---------------------+
| DIO17 |           | JTAG TDI            |
+-------+-----------+---------------------+
| DIO18 | UART0_RTS | UART RTS / JTAG SWO |
+-------+-----------+---------------------+
| DIO19 | UART0_CTS | UART CTS            |
+-------+-----------+---------------------+
| DIO20 | GPIO      | Flash CS            |
+-------+-----------+---------------------+
| DIO21 | GPIO      | Blue LED            |
+-------+-----------+---------------------+
| DIO22 | GPIO      |                     |
+-------+-----------+---------------------+
| DIO23 | AUX_IO    | A0 (DRV5032)        |
+-------+-----------+---------------------+
| DIO24 | AUX_IO    | A1                  |
+-------+-----------+---------------------+
| DIO25 | GPIO      | HDC2080 INT         |
+-------+-----------+---------------------+
| DIO26 | AUX_IO    | A3                  |
+-------+-----------+---------------------+
| DIO27 | GPIO      | OPT3001 INT         |
+-------+-----------+---------------------+
| DIO28 | AUX_IO    | A5                  |
+-------+-----------+---------------------+
| DIO29 | AUX_IO    | A6                  |
+-------+-----------+---------------------+
| DIO30 | AUX_IO    | ADXL362 INT         |
+-------+-----------+---------------------+

Programming and Debugging
*************************

Flashing
========

Applications for the ``CC1352R SensorTag`` board configuration can be built and
flashed in the usual way (see :ref:`build_an_application` and
:ref:`application_run` for more details).

Here is an example for the :ref:`hello_world` application.

First, run your favorite terminal program to listen for output.

.. code-block:: console

   $ screen <tty_device> 115200

Replace :code:`<tty_device>` with the port where the XDS110 application
serial device can be found. For example, :code:`/dev/ttyACM0`.

Then build and flash the application in the usual way.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: cc1352r_sensortag
   :goals: build flash

Debugging
=========

You can debug an application in the usual way.  Here is an example for the
:ref:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: cc1352r_sensortag
   :maybe-skip-config:
   :goals: debug

Bootloader
==========

The ROM bootloader on CC13x2 and CC26x2 devices is enabled by default. The
bootloader will start if there is no valid application image in flash or the
so-called backdoor is enabled (via option
:option:`CONFIG_CC13X2_CC26X2_BOOTLOADER_BACKDOOR_ENABLE`) and BTN-1 is held
down during reset. See the bootloader documentation in chapter 10 of the `TI
CC13x2 / CC26x2 Technical Reference Manual`_ for additional information.

Power Management and UART
=========================

System and device power management are supported on this platform, and
can be enabled via the standard Kconfig options in Zephyr, such as
:option:`CONFIG_SYS_POWER_MANAGEMENT`, :option:`CONFIG_DEVICE_POWER_MANAGEMENT`,
:option:`CONFIG_SYS_POWER_SLEEP_STATES`, and
:option:`CONFIG_SYS_POWER_DEEP_SLEEP_STATES`.

When system power management is turned on (CONFIG_SYS_POWER_MANAGEMENT=y),
sleep state 2 (standby mode) is allowed, and polling is used to retrieve input
by calling uart_poll_in(), it is possible for characters to be missed if the
system enters standby mode between calls to uart_poll_in(). This is because
the UART is inactive while the system is in standby mode. The workaround is to
disable sleep state 2 while polling:

.. code-block:: c

    sys_pm_ctrl_disable_state(SYS_POWER_STATE_SLEEP_2);
    <code that calls uart_poll_in() and expects input at any point in time>
    sys_pm_ctrl_enable_state(SYS_POWER_STATE_SLEEP_2);


References
**********

CC1352R1 SensorTag Quick Start Guide:
  https://www.ti.com/lit/pdf/swau127

.. _TI CC1352R SensorTag Product Page:
   http://www.ti.com/tool/lpstk-cc1352r

.. _TI CC1352R Product Page:
   http://www.ti.com/product/cc1352r

.. _TI CC13x2 / CC26x2 Technical Reference Manual:
   http://www.ti.com/lit/pdf/swcu185
