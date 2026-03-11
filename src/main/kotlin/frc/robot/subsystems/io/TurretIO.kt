package frc.robot.subsystems.io

import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import frc.robot.wrappers.PositionProvider
import frc.robot.wrappers.VelocityProvider
import frc.robot.wrappers.VoltageController

data class TurretIO(
    val voltageControllerSpin: VoltageController,
    val voltageControllerShoot1: VoltageController,
    val voltageControllerShoot2: VoltageController,
    val positionProvider: PositionProvider,
    val velocityProvider: VelocityProvider,
    val treeMap: InterpolatingTreeMap<Double, Double>
)