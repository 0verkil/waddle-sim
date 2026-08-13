package org.necessaryevil.waddle.physics.mechanism

import org.psilynx.psikit.core.mechanism.LoggedMechanismLigament2d

interface Mechanism {

    fun append(ligament: LoggedMechanismLigament2d)

}