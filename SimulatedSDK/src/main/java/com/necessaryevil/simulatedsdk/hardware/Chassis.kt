package com.necessaryevil.simulatedsdk.hardware

import org.psilynx.psikit.Logger
import org.psilynx.psikit.wpi.Pose2d
import org.psilynx.psikit.wpi.Rotation2d
import org.psilynx.psikit.wpi.Translation2d
import kotlin.math.sin

/**
 * Represents the robot itself. All coordinates, besides the actual robot pose, are robot-centric. +X is forward, +Y is left.
 */
class Chassis(mass: Double, vararg val wheels: Wheel, val centerOfRotation: Translation2d = Translation2d(),) {
    var numWheels: Int = wheels.size
    var pose: Pose2d = Pose2d()

    init {
        // add correct mass
        for (wheel in wheels) {
            wheel.simAppend(PhysicsLigament("chassis", mass / numWheels.toDouble(), length=Translation2d(wheel.x, wheel.y).norm ,lineWidth = 0.0))
        }
    }

    fun update(dt: Double) {
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
            val movementVector = wheel.directionVector.times(wheel.angularVelocity.radians * wheel.length)
            /*Logger.recordOutput("Length $i", wheel.length)
            Logger.recordOutput("Angvel $i", wheel.angularVelocity.radians)
            Logger.recordOutput("DeltaAngle $i", wheel.deltaAngle)
            Logger.recordOutput("Movement $i", movementVector)*/
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
        Logger.recordOutput("X Velocity", xvel)
        Logger.recordOutput("Y Velocity", yvel)
        Logger.recordOutput("Ang Velocity", angvel)


        // forward euler cuz i'm lazy
        pose = Pose2d(pose.x + xvel * dt, pose.y + yvel * dt, pose.rotation + Rotation2d(angvel * dt))
    }

}