package frc.robot.subsystems.io

import frc.robot.wrappers.PositionProvider
import frc.robot.wrappers.VelocityProvider
import frc.robot.wrappers.VoltageController

data class TurretIO(
    val voltageControllerSpin: VoltageController,
    val voltageControllerShoot: VoltageController,
    val positionProvider: PositionProvider,
    val velocityProvider: VelocityProvider
)