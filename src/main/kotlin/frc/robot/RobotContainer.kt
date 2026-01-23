package frc.robot

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.TurretSubsystem

class RobotContainer {
    val loop = EventLoop()

    val turret = TurretSubsystem()
    val controller = CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT)

    private fun configureBindings() {
        val angles = listOf(0, 45, 90, 135, 180, 225, 270, 315)

        for (angle in angles) {
            controller.pov(angle).onTrue(turret.setAngle(Units.Degree.of(angle.toDouble())))
        }
//        controller.rightBumper().whileTrue(turret.runTurret()).whileFalse(turret.stopTurret())
    }

    init {
        configureBindings()
    }
}