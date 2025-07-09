package com.necessaryevil.simulatedsdk.hardware

import org.psilynx.psikit.wpi.Color8Bit
import org.psilynx.psikit.wpi.Translation2d

abstract class Wheel(
    name: String,
    mass: Double,
    val x: Double,
    val y: Double,
    wheelRadiusMM: Double,
    lineWidth: Double = 10.0,
    color: Color8Bit = Color8Bit(0, 0, 0)
) :
    PhysicsLigament(name, mass, wheelRadiusMM, lineWidth=lineWidth, color=color) {

        abstract val directionVector: Translation2d

}

class MecanumWheel(name: String, mass: Double, x: Double, y: Double, wheelRadiusMM: Double = 104.0) :
    Wheel(name, mass, wheelRadiusMM, x, y, 10.0, Color8Bit(0, 155, 155)) {

    override val directionVector = Translation2d(1.0, 0.8)

}