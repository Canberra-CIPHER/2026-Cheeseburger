//TO-DO: Green LEDs for ready to shoot, red LEDS for no more fuel, driver holds bumpers on controller to shoot/intake

// Florence is Polygender, Trigender, Ambigender, Demigender - 50% Stellarian, 25% Libragender, 25% Boy
// And uses Xe/Zyr pronouns

package frc.robot

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Lights
import frc.robot.subsystems.Outtake
import org.dyn4j.geometry.Vector2
import org.dyn4j.geometry.Vector3
import java.sql.Driver
import kotlin.invoke
import kotlin.jvm.optionals.getOrNull
import kotlin.math.sign

class Robot : TimedRobot() {
    private val robotContainer: RobotContainer = RobotContainer()

    val turretFusionPublisher = NetworkTableInstance.getDefault().getTopic("turret/fusion").genericPublish("double")
    val turretEncoderOnePublisher = NetworkTableInstance.getDefault().getTopic("turret/encoder1").genericPublish("double")
    val turretEncoderTwoPublisher = NetworkTableInstance.getDefault().getTopic("turret/encoder2").genericPublish("double")

    val turretDistance = NetworkTableInstance.getDefault().getTopic("turret/distanceToTarget").genericPublish("double")
    val turretAngle = NetworkTableInstance.getDefault().getTopic("turret/angleToTarget").genericPublish("double")

    var movingAverageGuess = 0.0

    var lastSeenHubID: Int? = null

    override fun robotInit() {
        CommandScheduler.getInstance().registerSubsystem(robotContainer.swerveDriveSystem)
        CommandScheduler.getInstance().registerSubsystem(robotContainer.turret)
        CommandScheduler.getInstance().registerSubsystem(robotContainer.intake)
        CommandScheduler.getInstance().registerSubsystem(robotContainer.outtake)

        addPeriodic({ -> robotContainer.swerveDriveSystem.controlPeriodic() }, 0.01)
        addPeriodic({ -> robotContainer.turret.controlPeriodic()}, 0.01)
        addPeriodic({ -> robotContainer.intake.controlPeriodic()}, 0.01)
        addPeriodic({ -> robotContainer.outtake.controlPeriodic()}, 0.01)

        robotContainer.swerveDriveIO.swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 999999.999999))
    }

    fun calibrateAlliance() {
        val alliance = DriverStation.getAlliance()
        if (alliance.isPresent && alliance.get() == DriverStation.Alliance.Blue) {
            robotContainer.whereTagsAre.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide)
        } else {
            robotContainer.whereTagsAre.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide)
        }

//        TODO: Temporary - check coordinate system
//        if (alliance.isPresent && alliance.get() == DriverStation.Alliance.Blue) {
//            robotContainer.swerveDriveSystem.io.swerveDrive.setGyroOffset(Rotation3d(0.0, 0.0, -90.0))
//        }
//        else {
//            robotContainer.swerveDriveSystem.io.swerveDrive.setGyroOffset(Rotation3d(0.0, 0.0, 90.0))
//        }
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()

        calibrateAlliance()

        robotContainer.loop.poll()
        robotContainer.turret.periodic()
        robotContainer.swerveDriveSystem.periodic()
        robotContainer.intake.periodic()
        robotContainer.outtake.periodic()

        movingAverageGuess = robotContainer.turretFusionFilter.calculate(robotContainer.turretFusionEncoder.getPosition())
        turretFusionPublisher.setDouble(movingAverageGuess)
        turretEncoderOnePublisher.setDouble(robotContainer.turretEncoder1.getPosition())
        turretEncoderTwoPublisher.setDouble(robotContainer.turretEncoder2.getPosition())

        val result = robotContainer.camera.allUnreadResults
        if (result.isNotEmpty() && result.last().multitagResult.isPresent) {
            var last = result.last()

            var poseVar = last.multitagResult.get().estimatedPose.best

            if (last.multitagResult.isPresent) {
                 robotContainer.swerveDrive.addVisionMeasurement(Pose2d(Translation2d(poseVar.translation.x, poseVar.translation.y), Rotation2d(poseVar.rotation.x, poseVar.rotation.y)), result.last().timestampSeconds)
            }

            for (target in last.targets) {
                if ((target.fiducialId == 9) or (target.fiducialId == 10) or (target.fiducialId == 8) or (target.fiducialId == 5) or (target.fiducialId == 11) or (target.fiducialId == 2)) {
                    this.lastSeenHubID = 10
                }
                if ((target.fiducialId == 25) or (target.fiducialId == 26)  or (target.fiducialId == 18)  or (target.fiducialId == 27)  or (target.fiducialId == 21)  or (target.fiducialId == 24)) {
                    this.lastSeenHubID = 26
                }
            }
        }

        lastSeenHubID?.let {
            var targetPose = robotContainer.whereTagsAre.getTagPose(it).get()
            var targetPose2D = Pose2d(Translation2d(targetPose.translation.x, targetPose.translation.y), Rotation2d(targetPose.rotation.x, targetPose.rotation.y))
            val relativePosition = targetPose2D.relativeTo(robotContainer.swerveDrive.pose).relativeTo(robotContainer.cameraOffset).rotateBy(Rotation2d.fromDegrees(0.0))
            var relativeAngle = relativePosition.rotation.degrees
            var relativeDistance = relativePosition.translation.norm

            robotContainer.turret.angleToTarget = -relativeAngle
            robotContainer.turret.distanceToTarget = relativeDistance

            turretDistance.setDouble(relativeDistance)
            turretAngle.setDouble(relativeAngle)
        }
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun autonomousInit() {
        val currentPose = robotContainer.swerveDriveSystem.io.swerveDrive.pose
        val turnToTurret = robotContainer.swerveDriveSystem.driveToPosition(Pose2d(currentPose.translation, Rotation2d.fromDegrees(-135.0))).withTimeout(1.5)
        val autoRunAndSnap = robotContainer.turret.shootForTargetCommand()
        val autoRunToShoot = ParallelCommandGroup(robotContainer.turret.shootForTargetCommand(), robotContainer.outtake.outtakeDefaultCommand { 80.0 }, robotContainer.intake.intakeDefaultCommand { 80.0 }).withTimeout(5.0)

        val autoCommandGroup = SequentialCommandGroup( turnToTurret, autoRunAndSnap, autoRunToShoot)

        CommandScheduler.getInstance().schedule(autoCommandGroup)
//        CommandScheduler.getInstance().run()
        //    robotContainer.turretMotorWrapped.setPosition(movingAverageGuess)
    }

    override fun autonomousPeriodic() {}

    fun squareInputs(input: Double): Double {
        return input.sign * input * input
    }

    fun getThrottleMultiplier(): Double {
        if (robotContainer.xbox.rightBumperButton) {
            return 0.5
        } else {
            return 1.0
        }
    }

    fun getChassisDemand(): Transform2d{
        var translate = Translation2d(squareInputs(-robotContainer.xbox.leftY) * getThrottleMultiplier(), squareInputs(-robotContainer.xbox.leftX) * getThrottleMultiplier())
        var angle = Rotation2d(-robotContainer.xbox.rightX, -robotContainer.xbox.rightY)


//        val alliance = DriverStation.getAlliance()
//        if (alliance.isPresent && alliance.get() == DriverStation.Alliance.Blue) {
//            angle = angle.rotateBy(Rotation2d.fromDegrees(-90.0))
//            translate = translate.rotateBy(Rotation2d.fromDegrees(-90.0))
//        }
//        else {
//            angle = angle.rotateBy(Rotation2d.fromDegrees(90.0))
//            translate = translate.rotateBy(Rotation2d.fromDegrees(90.0))
//        }

        return Transform2d(translate, angle)
    }

    override fun teleopInit() {
        CommandScheduler.getInstance().cancelAll()

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

        Transform2d()

//        robotContainer.turretMotorWrapped.setPosition(robotContainer.turretFusionEncoder.getPosition() % 1.0)
        CommandScheduler.getInstance().setDefaultCommand(robotContainer.swerveDriveSystem, robotContainer.swerveDriveSystem.driveDefaultCommand(
            { -> getChassisDemand().x },
            { -> getChassisDemand().y },
            { -> snapFun.invoke().first ?: getChassisDemand().rotation.cos },
            { -> snapFun.invoke().second ?: getChassisDemand().rotation.sin },
        ))

        CommandScheduler.getInstance().setDefaultCommand(robotContainer.turret, robotContainer.turret.shootDefaultCommand(
            {-> if (robotContainer.xbox2.leftTriggerAxis > 0.05 && robotContainer.xbox2.leftBumperButton) -robotContainer.xbox2.leftTriggerAxis * 65.0 else if (robotContainer.xbox2.leftTriggerAxis > 0.05) robotContainer.xbox2.leftTriggerAxis * 65.0 else 0.0 }
        ))
        CommandScheduler.getInstance().setDefaultCommand(robotContainer.outtake, robotContainer.outtake.outtakeDefaultCommand(
            {-> if (robotContainer.xbox2.rightBumperButton && robotContainer.xbox2.leftBumperButton) -10.0 else if (robotContainer.xbox2.rightBumperButton) 10.0 else 0.0 }
        ))
        CommandScheduler.getInstance().setDefaultCommand(robotContainer.intake, robotContainer.intake.intakeDefaultCommand(
            {-> if (robotContainer.xbox2.rightTriggerAxis > 0.05 && robotContainer.xbox2.leftBumperButton) -robotContainer.xbox2.rightTriggerAxis * 12.0 else if (robotContainer.xbox2.rightTriggerAxis > 0.05) robotContainer.xbox2.rightTriggerAxis * 12.0 else 0.0 }
        ))
    }

    override fun teleopPeriodic() {
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
