package com.necessaryevil.simulatedsdk.ftc

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Represents the state of an Xbox One Controller, parsed from its HID input report.
 *
 * The HID report structure is based on the provided descriptor:
 * - X, Y, Rx, Ry axes are 16-bit unsigned values (0-65535).
 * - Z, Rz axes (triggers) are 10-bit unsigned values (0-1023).
 * - Buttons are 1-bit boolean values.
 * - Hat switch is a 4-bit value (1-8 for directions, 0 for centered).
 * - System Main Menu button is a 1-bit boolean.
 * - Battery Strength is an 8-bit unsigned value (0-255).
 */
data class XboxOneControllerInputReport(
    val lx: Int, // Left Stick X (0-65535)
    val ly: Int, // Left Stick Y (0-65535)
    val rx: Int, // Right Stick X (0-65535)
    val ry: Int, // Right Stick Y (0-65535)
    val lt: Int, // Left Trigger (0-1023)
    val rt: Int, // Right Trigger (0-1023)
    val dPad: DPadDirection,
    val buttonA: Boolean,
    val buttonB: Boolean,
    val buttonX: Boolean,
    val buttonY: Boolean,
    val buttonLB: Boolean, // Left Bumper
    val buttonRB: Boolean, // Right Bumper
    val buttonBack: Boolean,
    val buttonStart: Boolean,
    val buttonLS: Boolean, // Left Stick Click
    val buttonRS: Boolean, // Right Stick Click
    val buttonGuide: Boolean, // System Main Menu button
    val batteryStrength: Int // 0-255
) {

    enum class DPadDirection {
        CENTERED,
        UP,
        UP_RIGHT,
        RIGHT,
        DOWN_RIGHT,
        DOWN,
        DOWN_LEFT,
        LEFT,
        UP_LEFT,
        UNKNOWN; // Should not happen with a well-formed report

        companion object {
            fun fromHatSwitchValue(value: Int): DPadDirection {
                return when (value) {
                    0 -> CENTERED
                    1 -> UP
                    2 -> UP_RIGHT
                    3 -> RIGHT
                    4 -> DOWN_RIGHT
                    5 -> DOWN
                    6 -> DOWN_LEFT
                    7 -> LEFT
                    8 -> UP_LEFT
                    else -> UNKNOWN
                }
            }
        }
    }

    companion object {
        private const val REPORT_SIZE_BYTES = 17 // Based on the parsed descriptor structure

        /**
         * Converts a raw HID input report byte array into an [XboxOneControllerInputReport] object.
         *
         * @param hidData The byte array representing the HID input report.
         * @return An [XboxOneControllerInputReport] object, or null if the data is malformed.
         */
        fun fromHidData(hidData: ByteArray): XboxOneControllerInputReport? {

            if (hidData.size < REPORT_SIZE_BYTES) {
                println("HID data too short. Expected at least $REPORT_SIZE_BYTES bytes, got ${hidData.size}")
                return null
            }

            // Wrap the byte array in a ByteBuffer for easier parsing of multi-byte values and bit manipulation
            val buffer = ByteBuffer.wrap(hidData).order(ByteOrder.LITTLE_ENDIAN)

            // Joysticks (X, Y, Rx, Ry) - 16 bits each
            val lx = buffer.getShort(1).toUShort().toInt()
            val ly = buffer.getShort(3).toUShort().toInt()
            val rx = buffer.getShort(5).toUShort().toInt()
            val ry = buffer.getShort(7).toUShort().toInt()

            // Triggers (Z, Rz) - 10 bits each. These are packed into 16-bit words with 6 padding bits.
            // Z is at byte 8, bits 0-9
            val zCombined = buffer.getShort(9).toUShort().toInt()
            val lt = zCombined and 0x3FF // Mask for the first 10 bits

            // Rz is at byte 10, bits 0-9
            val rzCombined = buffer.getShort(11).toUShort().toInt()
            val rt = rzCombined and 0x3FF // Mask for the first 10 bits


            // Buttons - 10 bits packed into 2 bytes, followed by 6 padding bits.
            // Buttons start at byte 12
            val buttonBytes = buffer.getShort(14).toUShort().toInt()
            val buttonA = (buttonBytes and (1 shl 0)) != 0
            val buttonB = (buttonBytes and (1 shl 1)) != 0
            val buttonX = (buttonBytes and (1 shl 2)) != 0
            val buttonY = (buttonBytes and (1 shl 3)) != 0
            val buttonLB = (buttonBytes and (1 shl 4)) != 0
            val buttonRB = (buttonBytes and (1 shl 5)) != 0
            val buttonBack = (buttonBytes and (1 shl 6)) != 0
            val buttonStart = (buttonBytes and (1 shl 7)) != 0
            val buttonLS = (buttonBytes and (1 shl 8)) != 0
            val buttonRS = (buttonBytes and (1 shl 9)) != 0

            // Hat Switch - 4 bits, followed by 4 padding bits.
            // Hat switch is at byte 14, upper 4 bits of the byte are padding.
            val hatSwitchValue = (buffer.get(13).toUByte().toInt() and 0x0F) // Mask for the lower 4 bits
            val dPad = DPadDirection.fromHatSwitchValue(hatSwitchValue)

            // System Main Menu Button - 1 bit, followed by 7 padding bits.
            // System Main Menu button is at byte 15, bit 0
            val systemControlByte = buffer.get(15).toUByte().toInt()
            val buttonGuide = (systemControlByte and (1 shl 0)) != 0

            // Battery Strength - 8 bits
            // Battery strength is at byte 16
            val batteryStrength = buffer.get(16).toUByte().toInt()

            return XboxOneControllerInputReport(
                lx = lx,
                ly = ly,
                rx = rx,
                ry = ry,
                lt = lt,
                rt = rt,
                dPad = dPad,
                buttonA = buttonA,
                buttonB = buttonB,
                buttonX = buttonX,
                buttonY = buttonY,
                buttonLB = buttonLB,
                buttonRB = buttonRB,
                buttonBack = buttonBack,
                buttonStart = buttonStart,
                buttonLS = buttonLS,
                buttonRS = buttonRS,
                buttonGuide = buttonGuide,
                batteryStrength = batteryStrength
            )
        }
    }
}