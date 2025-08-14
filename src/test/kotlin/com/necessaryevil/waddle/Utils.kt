package com.necessaryevil.waddle

import com.necessaryevil.waddle.ftc.time.SimulatableElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.psilynx.psikit.wpi.Pose2d
import org.psilynx.psikit.wpi.Rotation2d

/**
 * Example PID class.
 */
class PID(var p: Double, var i: Double, var d: Double) {

    private var lastError = 0.0
    private var integral = 0.0
    private val timer =
        SimulatableElapsedTime() // this is important! using a normal elapsedtime will have the wrong timing.
    private val dt get() = timer.seconds()

    fun update(error: Double): Double {

        integral += error * dt

        val proportional = p * error
        val derivative = d * (error - lastError) / (if (dt > 0.01) dt else 0.01)
        val sum = integral * i

        lastError = error

        timer.reset()

        return proportional + derivative + sum

    }

}

/**
 * Differs from PsiKit in that this one is in inches and uses standard ftc coordinate system (i.e. math-usable but not loggable).
 */
fun Pose2D.asWpiPose2d(): Pose2d = Pose2d(
    this.getX(DistanceUnit.INCH), this.getY(DistanceUnit.INCH), Rotation2d(
        this.getHeading(
            AngleUnit.RADIANS
        )
    )
)