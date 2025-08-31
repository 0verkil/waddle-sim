/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2014-2020 Gary Rowe
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

package org.necessaryevil.waddle;

import org.necessaryevil.waddle.ftc.XboxOneControllerInputReport;

import org.hid4java.*;

/**
 * Demonstrate the USB HID interface using a Satoshi Labs Trezor
 *
 * @since 0.0.1
 */
public class UsbHidEnumerationExample extends BaseExample {

    public static void main(String[] args) throws HidException {

        UsbHidEnumerationExample example = new UsbHidEnumerationExample();
        example.executeExample();

    }

    public static boolean bitAtIndex(byte num, int index) {
        return ((num >> (index-1)) & 1) > 0;
    }

    private void executeExample() throws HidException {

        printPlatform();

        // Configure to use custom specification
        HidServicesSpecification hidServicesSpecification = new HidServicesSpecification();
        // Use the v0.7.0 manual start feature to get immediate attach events
        hidServicesSpecification.setAutoStart(false);

        // Get HID services using custom specification
        HidServices hidServices = HidManager.getHidServices(hidServicesSpecification);
        hidServices.addHidServicesListener(this);

        // Manually start the services to get attachment event
        System.out.println(ANSI_GREEN + "Manually starting HID services." + ANSI_RESET);
        hidServices.start();

        System.out.println(ANSI_GREEN + "Enumerating attached devices..." + ANSI_RESET);

        // Provide a list of attached devices

        HidDevice device = hidServices.getAttachedHidDevices().get(0);
        for (HidDevice hidDevice : hidServices.getAttachedHidDevices()) {
            System.out.println(hidDevice);
            if (hidDevice.getProduct().startsWith("Xbox")) {
                device = hidDevice;
            }
        }

        while (device.isClosed()) {
            device.open();
        }

        device.setNonBlocking(true);
        System.out.println(device);

        while (true) {
            Byte[] report = device.read();
            byte[] reportbyte = new byte[report.length];
            if (report.length > 0) {
                StringBuilder message = new StringBuilder();
                int i = 0;
                for (byte a : report) {
                    message.append(Byte.toUnsignedInt(a)).append(" ");
                    reportbyte[i] = a;
                    i++;
                }

                XboxOneControllerInputReport thing = XboxOneControllerInputReport.Companion.fromHidData(reportbyte);

                if (thing != null) {
                    System.out.println(thing.getDPad().name());
                }
                //System.out.println(message);
            }
        }


    }

}