package org.necessaryevil.waddle.physics.mechanism

import org.psilynx.psikit.mechanism.LoggedMechanism2d
import org.psilynx.psikit.mechanism.LoggedMechanismLigament2d

interface Mechanism {

    fun append(ligament: LoggedMechanismLigament2d)

}