package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class TestOpMode : LinearOpMode() {

    override fun runOpMode() {
        waitForStart()

        while (opModeIsActive()) {
            var a = 3.0
        }

    }

}

fun main() {
    println("test")
}