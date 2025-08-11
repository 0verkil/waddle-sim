package com.necessaryevil.waddle.ftc

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.Func
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.psilynx.psikit.Logger

import kotlin.reflect.KClass
import kotlin.reflect.KFunction
import kotlin.reflect.full.staticFunctions

/**
 * Attempts to record an output using `Logger`, dynamically selecting the correct type.
 *
 * @param key The message string.
 * @param value The Any? value to be recorded.
 * @return True if a suitable type was found and invoked, false otherwise.
 */
// thanks gemini
fun recordOutputDynamic(key: String?, value: Any?): Boolean {
    val recordOutputFunctions = Logger::class.staticFunctions.filter { it.name == "recordOutput" }

    var bestMatch: KFunction<*>? = null
    var bestMatchScore = -1 // Higher score means a better match (closer in inheritance tree)

    for (function in recordOutputFunctions) {
        val parameters = function.parameters
        // We expect 2 parameters: message (String), value (Any?)
        if (parameters.size == 2 && parameters[1].type.classifier == String::class) {
            val valueParam = parameters[1] // This is the 'value' parameter
            val expectedKType = valueParam.type
            val expectedKClass = expectedKType.classifier as? KClass<*>

            if (expectedKClass == null) {
                // Cannot determine target KClass (e.g., if it's a type parameter with no concrete type)
                continue
            }

            // --- Type Compatibility Check ---
            val isCompatible = when {
                value == null -> expectedKType.isMarkedNullable // Null value is compatible with any nullable type
                expectedKClass.java.isAssignableFrom(value::class.java) -> true // Value's class is assignable to parameter's class
                else -> false
            }

            if (isCompatible) {
                // Basic scoring: exact match is better, then assignable.
                var currentScore = 0
                if (value != null && value::class == expectedKClass) {
                    currentScore = 2 // Exact match
                } else if (value != null && expectedKClass.java.isAssignableFrom(value::class.java)) {
                    currentScore = 1 // Subtype match
                } else if (value == null && expectedKType.isMarkedNullable) {
                    currentScore = 1 // Nullable parameter match
                }

                // If this is a better match or the first match
                if (currentScore > bestMatchScore) {
                    bestMatchScore = currentScore
                    bestMatch = function
                }
            }
        }
    }

    if (bestMatch != null) {
        try {
            // The first parameter is the 'key'
            // The second parameter is the 'value'
            // Since methods are static, we don't have to worry about a receiver
            bestMatch.call(key ?: "", value)
            return true
        } catch (e: Exception) {
            // can be enabled for verbose debugging
            // System.err.println("Error invoking recordOutput for value $value: ${e.message}")
            return false
        }
    } else {
        // println("No suitable recordOutput overload found for value: $value (Type: ${value?.let { it::class.simpleName } ?: "null"})")
        return false
    }
}

class PsikitTelemetry(opMode: OpMode) : Telemetry {
    override fun addData(
        caption: String?,
        format: String?,
        vararg args: Any?
    ): Telemetry.Item? {
        return addData(caption, args)
    }

    override fun addData(
        caption: String?,
        value: Any?
    ): Telemetry.Item? {
        recordOutputDynamic(caption, value)
        return PsikitTelemetryItem()
    }

    override fun <T : Any?> addData(
        caption: String?,
        valueProducer: Func<T?>?
    ): Telemetry.Item? {
        return addData(caption, valueProducer?.value())
    }

    override fun <T : Any?> addData(
        caption: String?,
        format: String?,
        valueProducer: Func<T?>?
    ): Telemetry.Item? {
        return addData(caption, valueProducer)
    }

    override fun removeItem(item: Telemetry.Item?): Boolean {
        return true
    }

    override fun clear() {

    }

    override fun clearAll() {

    }

    override fun addAction(action: Runnable?): Any? {
        TODO("Not yet implemented")
    }

    override fun removeAction(token: Any?): Boolean {
        TODO("Not yet implemented")
    }

    override fun speak(text: String?) {
        TODO("Not yet implemented")
    }

    override fun speak(
        text: String?,
        languageCode: String?,
        countryCode: String?
    ) {
        TODO("Not yet implemented")
    }

    override fun update(): Boolean {
        return true
    }

    override fun addLine(): Telemetry.Line? {
        return PsikitTelemetryLine()
    }

    override fun addLine(lineCaption: String?): Telemetry.Line? {
        return PsikitTelemetryLine()
    }

    override fun removeLine(line: Telemetry.Line?): Boolean {
        return true
    }

    override fun isAutoClear(): Boolean {
        TODO("Not yet implemented")
    }

    override fun setAutoClear(autoClear: Boolean) {

    }

    override fun getMsTransmissionInterval(): Int {
        TODO("Not yet implemented")
    }

    override fun setMsTransmissionInterval(msTransmissionInterval: Int) {

    }

    override fun getItemSeparator(): String? {
        TODO("Not yet implemented")
    }

    override fun setItemSeparator(itemSeparator: String?) {

    }

    override fun getCaptionValueSeparator(): String? {
        TODO("Not yet implemented")
    }

    override fun setCaptionValueSeparator(captionValueSeparator: String?) {

    }

    override fun setDisplayFormat(displayFormat: Telemetry.DisplayFormat?) {

    }

    override fun log(): Telemetry.Log? {
        TODO("Not yet implemented")
    }

    class PsikitTelemetryLine : Telemetry.Line {
        override fun addData(
            caption: String?,
            format: String?,
            vararg args: Any?
        ): Telemetry.Item? {
            return PsikitTelemetryItem()
        }

        override fun addData(
            caption: String?,
            value: Any?
        ): Telemetry.Item? {
            return PsikitTelemetryItem()
        }

        override fun <T : Any?> addData(
            caption: String?,
            valueProducer: Func<T?>?
        ): Telemetry.Item? {
            return PsikitTelemetryItem()
        }

        override fun <T : Any?> addData(
            caption: String?,
            format: String?,
            valueProducer: Func<T?>?
        ): Telemetry.Item? {
            return PsikitTelemetryItem()
        }

    }

    /**
     * Dummy class. Do not try calling any of the methods LOL
     */
    class PsikitTelemetryItem : Telemetry.Item {
        override fun getCaption(): String? {
            TODO("Not yet implemented")
        }

        override fun setCaption(caption: String?): Telemetry.Item? {
            TODO("Not yet implemented")
        }

        override fun setValue(
            format: String?,
            vararg args: Any?
        ): Telemetry.Item? {
            TODO("Not yet implemented")
        }

        override fun setValue(value: Any?): Telemetry.Item? {
            TODO("Not yet implemented")
        }

        override fun <T : Any?> setValue(valueProducer: Func<T?>?): Telemetry.Item? {
            TODO("Not yet implemented")
        }

        override fun <T : Any?> setValue(
            format: String?,
            valueProducer: Func<T?>?
        ): Telemetry.Item? {
            TODO("Not yet implemented")
        }

        override fun setRetained(retained: Boolean?): Telemetry.Item? {
            TODO("Not yet implemented")
        }

        override fun isRetained(): Boolean {
            TODO("Not yet implemented")
        }

        override fun addData(
            caption: String?,
            format: String?,
            vararg args: Any?
        ): Telemetry.Item? {
            TODO("Not yet implemented")
        }

        override fun addData(
            caption: String?,
            value: Any?
        ): Telemetry.Item? {
            TODO("Not yet implemented")
        }

        override fun <T : Any?> addData(
            caption: String?,
            valueProducer: Func<T?>?
        ): Telemetry.Item? {
            TODO("Not yet implemented")
        }

        override fun <T : Any?> addData(
            caption: String?,
            format: String?,
            valueProducer: Func<T?>?
        ): Telemetry.Item? {
            TODO("Not yet implemented")
        }

    }
    
}