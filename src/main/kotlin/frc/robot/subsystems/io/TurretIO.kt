package frc.robot.subsystems.io

import frc.robot.wrappers.PositionProvider
import frc.robot.wrappers.VoltageController

data class TurretIO(
    val voltageController: VoltageController,
    val positionProvider: PositionProvider
)