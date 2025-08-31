package org.necessaryevil.waddle.ftc.time

import com.qualcomm.robotcore.util.ElapsedTime
import org.psilynx.psikit.Logger

class SimulatableElapsedTime : ElapsedTime() {

    override fun nsNow(): Long = (Logger.getTimestamp() / 1.0E9).toLong()

}