package com.necessaryevil.simulatedsdk.physics.chassis

import com.necessaryevil.simulatedsdk.physics.common.PhysicsLigament
import org.psilynx.psikit.wpi.Color8Bit
import org.psilynx.psikit.wpi.Translation2d
import kotlin.math.hypot
import kotlin.math.sign

abstract class Wheel(
    name: String,
    mass: Double,
    val x: Double,
    val y: Double,
    wheelRadiusMM: Double,
    var chassis: Chassis? = null,
    lineWidth: Double = 10.0,
    color: Color8Bit = Color8Bit(0, 0, 0)
) :
    PhysicsLigament(
        name,
        mass,
        wheelRadiusMM / 1000.0,
        lineWidth = lineWidth,
        color = color,
        isCircle = true
    ) {

    init {
        constrainLengthByConstant(wheelRadiusMM / 1000.0)
    }

    abstract val directionVector: Translation2d

}

const val MEC_RR = 0.3

class MecanumWheel(
    name: String,
    mass: Double,
    x: Double,
    y: Double,
    wheelRadiusMM: Double = 104.0,
    chassis: Chassis? = null,
    reversed: Boolean = false
) :
    Wheel(name, mass, x, y, wheelRadiusMM, chassis, 10.0, Color8Bit(0, 155, 155)) {

    override val directionVector: Translation2d =
        Translation2d(1.0, 0.8 * if (reversed) -1.0 else 1.0).div(hypot(1.0, 0.8))

    override val angularLoad: Double
        get() {
            return (MEC_RR * (chassis?.mass ?: 0.0) / ((chassis?.numWheels?.toDouble()
                ?: 1.0) * (chassis?.efficiency ?: 1.0)) + ((chassis?.aerodynamicDrag ?: 0.0) / (chassis?.numWheels?.toDouble()
                ?: 1.0))) * sign(this.angularVelocity)

        }
}