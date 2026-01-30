//TO-DO: Green LEDs for ready to shoot, red LEDS for no more fuel, driver holds bumpers on controller to shoot/intake

package frc.robot

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.subsystems.Lights

class Robot : TimedRobot() {
    private val robotContainer: RobotContainer = RobotContainer()

    val turretFusionPublisher = NetworkTableInstance.getDefault().getTopic("turret/fusion").genericPublish("double")
    val turretEncoderOnePublisher = NetworkTableInstance.getDefault().getTopic("turret/encoder1").genericPublish("double")
    val turretEncoderTwoPublisher = NetworkTableInstance.getDefault().getTopic("turret/encoder2").genericPublish("double")
    var movingAverageGuess = 0.0

    override fun robotInit() {
        CommandScheduler.getInstance().registerSubsystem(robotContainer.swerveDriveSystem)
        CommandScheduler.getInstance().registerSubsystem(robotContainer.turret)

        addPeriodic({ -> robotContainer.swerveDriveSystem.controlPeriodic() }, 0.01)
        addPeriodic({ -> robotContainer.turret.controlPeriodic()}, 0.01)
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        robotContainer.loop.poll()
        robotContainer.turret.periodic()

        movingAverageGuess = robotContainer.turretFusionFilter.calculate(robotContainer.turretFusionEncoder.getPosition())
        turretFusionPublisher.setDouble(movingAverageGuess)
        turretEncoderOnePublisher.setDouble(robotContainer.turretEncoder1.getPosition())
        turretEncoderTwoPublisher.setDouble(robotContainer.turretEncoder2.getPosition())
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun autonomousInit() {
        robotContainer.turretMotorWrapped.setPosition(movingAverageGuess)
    }

    override fun autonomousPeriodic() {}

    override fun teleopInit() {
//        robotContainer.turretMotorWrapped.setPosition(robotContainer.turretFusionEncoder.getPosition() % 1.0)
    }

    override fun teleopPeriodic() {}

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
