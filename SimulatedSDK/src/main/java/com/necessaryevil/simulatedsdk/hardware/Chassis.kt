package com.necessaryevil.simulatedsdk.hardware

import com.necessaryevil.simulatedsdk.hardware.SimulationObject
import org.psilynx.psikit.Logger
import org.psilynx.psikit.wpi.Pose2d
import org.psilynx.psikit.wpi.Rotation2d
import org.psilynx.psikit.wpi.Translation2d
import kotlin.math.pow
import kotlin.math.sin

/**
 * Represents the robot itself. All coordinates, besides the actual robot pose, are robot-centric. +X is forward, +Y is left.
 */
class Chassis(
    val mass: Double,
    vararg val wheels: Wheel,
    val centerOfRotation: Translation2d = Translation2d(),
    val frontalArea: Double,
    val trackwidth: Double,
    val efficiency: Double = 0.7
) : SimulationObject {
    val numWheels: Int = wheels.size
    var pose: Pose2d = Pose2d()
    var velocity: Pose2d = Pose2d()
    val aerodynamicDrag
        get() = 0.5 * 1.225 * (velocity.translation.norm + velocity.rotation.radians * trackwidth / 2.0).pow(
            2
        ) * frontalArea * 1.2


    init {
        // add correct mass
        for (wheel in wheels) {
            wheel.simAppend(
                PhysicsLigament(
                    "chassis",
                    mass / numWheels.toDouble(),
                    length = 0.0,
                    lineWidth = 0.0
                )
            )
            wheel.chassis = this
        }
    }

    override fun update(dt: Double) {
        for (wheel in wheels) {
            wheel.update(dt)

            Logger.recordOutput("Wheel angle", wheel.angle)
            Logger.recordOutput("Expected wheel angle", wheel.angleSupplier.get())
            Logger.recordOutput("Is this thing on?", System.nanoTime())
        }

        // sum velocities
        var angvel = 0.0
        var xvel = 0.0
        var yvel = 0.0
        var i = 0
        for (wheel in wheels) {

            // add vel
            val movementVector =
                wheel.directionVector.times(wheel.angularVelocity.radians * wheel.length)
            Logger.recordOutput("Drivetrain/$i/Length", wheel.length)
            Logger.recordOutput("Drivetrain/$i/Angvel", wheel.angularVelocity.radians)
            Logger.recordOutput("Drivetrain/$i/DeltaAngle", wheel.deltaAngle)
            //Logger.recordOutput("Movement $i", movementVector)
            Logger.recordOutput("Wheel Angvel", wheel.angularVelocity)
            xvel += movementVector.x
            yvel += movementVector.y

            // compute vector length perpendicular to center of rotation, then divide by radius to get theta
            val toWheel = Translation2d(wheel.x, wheel.y).minus(centerOfRotation)
            val deltaAngle = toWheel.angle - movementVector.angle
            angvel += movementVector.norm * sin(deltaAngle.radians) / toWheel.norm
            i++
        }

        xvel /= numWheels.toDouble()
        yvel /= numWheels.toDouble()
        angvel /= numWheels.toDouble()
        velocity = Pose2d(xvel, yvel, Rotation2d(angvel))
        Logger.recordOutput("Drivetrain/X Velocity", xvel)
        Logger.recordOutput("Drivetrain/Y Velocity", yvel)
        Logger.recordOutput("Drivetrain/Ang Velocity", angvel)


        // forward euler cuz i'm lazy
        pose =
            Pose2d(pose.x + xvel * dt, pose.y + yvel * dt, pose.rotation + Rotation2d(angvel * dt))
    }

}