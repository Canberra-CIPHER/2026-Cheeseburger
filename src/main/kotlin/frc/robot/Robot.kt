//TO-DO: Green LEDs for ready to shoot, red LEDS for no more fuel, driver holds bumpers on controller to shoot/intake

// Florence is Polygender, Trigender, Ambigender, Demigender - 50% Stellarian, 25% Libragender, 25% Boy
// And uses Xe/Zyr pronouns

package frc.robot

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.subsystems.Lights
import kotlin.invoke
import kotlin.jvm.optionals.getOrNull
import kotlin.math.sign

class Robot : TimedRobot() {
    private val robotContainer: RobotContainer = RobotContainer()

    val turretFusionPublisher = NetworkTableInstance.getDefault().getTopic("turret/fusion").genericPublish("double")
    val turretEncoderOnePublisher = NetworkTableInstance.getDefault().getTopic("turret/encoder1").genericPublish("double")
    val turretEncoderTwoPublisher = NetworkTableInstance.getDefault().getTopic("turret/encoder2").genericPublish("double")
    var movingAverageGuess = 0.0

    var lastSeenHubID: Int? = null

    override fun robotInit() {
        CommandScheduler.getInstance().registerSubsystem(robotContainer.swerveDriveSystem)
        CommandScheduler.getInstance().registerSubsystem(robotContainer.turret)

        addPeriodic({ -> robotContainer.swerveDriveSystem.controlPeriodic() }, 0.01)
        addPeriodic({ -> robotContainer.turret.controlPeriodic()}, 0.01)

        robotContainer.initMotor()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        robotContainer.loop.poll()
        robotContainer.turret.periodic()
        robotContainer.swerveDriveSystem.periodic()

        movingAverageGuess = robotContainer.turretFusionFilter.calculate(robotContainer.turretFusionEncoder.getPosition())
        turretFusionPublisher.setDouble(movingAverageGuess)
        turretEncoderOnePublisher.setDouble(robotContainer.turretEncoder1.getPosition())
        turretEncoderTwoPublisher.setDouble(robotContainer.turretEncoder2.getPosition())

        val result = robotContainer.camera.allUnreadResults
        if (result.isNotEmpty()) {
            var last = result.last()

            var poseVar = last.multitagResult.get().estimatedPose.best

            if (last.multitagResult.isPresent) {
                 robotContainer.swerveDrive.addVisionMeasurement(Pose2d(Translation2d(poseVar.translation.x, poseVar.translation.y), Rotation2d(poseVar.rotation.x, poseVar.rotation.y)), result.last().timestampSeconds)
            }

            for (target in last.targets) {
                if ((target.fiducialId == 9) or (target.fiducialId == 10)) {
                    this.lastSeenHubID = 10
                }
                if ((target.fiducialId == 25) or (target.fiducialId == 26)) {
                    this.lastSeenHubID = 25
                }
            }
        }

        lastSeenHubID?.let {
            var targetPose = robotContainer.whereTagsAre.getTagPose(it).get()
            var targetPose2D = Pose2d(Translation2d(targetPose.translation.x, targetPose.translation.y), Rotation2d(targetPose.rotation.x, targetPose.rotation.y))
            var relativePosition = targetPose2D.minus(robotContainer.swerveDrive.pose)
            var relativeAngle = relativePosition.rotation.degrees
            var relativeDistance = relativePosition.translation.norm

            robotContainer.turret.angleToTarget = relativeAngle
            robotContainer.turret.distanceToTarget = relativeDistance
        }
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun autonomousInit() {
    //    robotContainer.turretMotorWrapped.setPosition(movingAverageGuess)
    }

    override fun autonomousPeriodic() {}

    fun squareInputs(input: Double): Double {
        return input.sign * input * input
    }

    fun getThrottleMultiplier(): Double {
        if (robotContainer.xbox.rightBumperButton) {
            return 0.35
        }
        else {
            return 0.5
        }
    }

    override fun teleopInit() {
        // autonomousCommand?.cancel()

        val snapFun = { ->
            var snapX: Double? = null
            var snapY: Double? = null

            if (robotContainer.xbox.xButton) {
                snapX = 0.0
                snapY = -1.0
            }
            else if (robotContainer.xbox.aButton) {
                snapX = -1.0
                snapY = 0.0
            }
            else if (robotContainer.xbox.bButton) {
                snapX = 0.0
                snapY = 1.0
            }
            else if (robotContainer.xbox.yButton) {
                snapX = 1.0
                snapY = 0.0
            }

            Pair(snapX, snapY)
        }
//        robotContainer.turretMotorWrapped.setPosition(robotContainer.turretFusionEncoder.getPosition() % 1.0)
        CommandScheduler.getInstance().setDefaultCommand(robotContainer.swerveDriveSystem, robotContainer.swerveDriveSystem.driveDefaultCommand(
            { -> squareInputs(-robotContainer.xbox.leftY) * getThrottleMultiplier() },
            { -> squareInputs(-robotContainer.xbox.leftX) * getThrottleMultiplier() },
            { -> snapFun.invoke().first ?: -robotContainer.xbox.rightX },
            { -> snapFun.invoke().second ?: -robotContainer.xbox.rightY },
        ))

        CommandScheduler.getInstance().setDefaultCommand(robotContainer.turret, robotContainer.turret.shootDefaultCommand(
            {-> if (robotContainer.xbox.leftBumperButton) 90.0 else if (robotContainer.xbox.rightBumperButton) -15.0 else 0.0 }
        ))
    }

    override fun teleopPeriodic() {
        robotContainer.intakeMotor!!.set(robotContainer.xbox.leftTriggerAxis)
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
