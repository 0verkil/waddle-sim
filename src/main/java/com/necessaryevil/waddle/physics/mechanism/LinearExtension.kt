package com.necessaryevil.waddle.physics.mechanism

import com.necessaryevil.waddle.physics.common.PhysicsLigament
import com.necessaryevil.waddle.physics.common.SimulatedMotor
import com.necessaryevil.waddle.physics.common.SimulationObject
import com.necessaryevil.waddle.physics.common.degrees
import org.psilynx.psikit.Logger
import org.psilynx.psikit.mechanism.LoggedMechanism2d
import kotlin.math.PI
import kotlin.math.min

/**
 * @param massKilograms In kilograms.
 * @param angleDegrees In degrees. 0 is parallel to the tile floor.
 */
class LinearExtension(
    val name: String,
    massKilograms: Double,
    private val minInches: Double = 0.0,
    maxInches: Double,
    val angleDegrees: Double,
    spoolRadiusMillimeters: Double,
    vararg motors: SimulatedMotor,
    efficiency: Double = 1.0
) : SimulationObject {

    private val mechanism = LoggedMechanism2d(10.0, 10.0)
    private val root by lazy { mechanism.getRoot("root", 0.0, 0.0) }
    private val spool by lazy {
        root.append(
            PhysicsLigament(
                "spool",
                0.0,
                spoolRadiusMillimeters / 1000.0,
                lineWidth = 0.0
            )
        )
    }
    private val extension by lazy {
        spool.simAppend(
            PhysicsLigament(
                name,
                massKilograms,
                minInches,
                angleDegrees,
                efficiency = efficiency
            )
        )
    }


    init {
        extension.constrainAngleByConstant(angleDegrees)
        extension.constrainLengthByAngle(spool, 0.0, minInches, maxInches)

        // find min and max angles
        val minMeters = minInches * 0.0254
        val maxMeters = minInches * 0.0254

        val radiusMeters = spoolRadiusMillimeters / 1000.0

        val minRad = minMeters / radiusMeters // theta = L / R
        val maxRad = maxMeters / radiusMeters

        spool.constrainLengthByConstant(radiusMeters)
        spool.constrainAngleByMotors(motors, 0.0, minRad.degrees, maxRad.degrees)
    }

    override fun update(dt: Double) {
        spool.update(dt)
        extension.update(dt)

        Logger.recordOutput("Simulation/$name", mechanism)
    }


}