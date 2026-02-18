package frc.robot

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.thethriftybot.devices.ThriftyNova
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.sensorfusion.CRTFusion
import frc.robot.subsystems.Lights
import frc.robot.subsystems.SwerveDrive
import frc.robot.subsystems.TurretSubsystem
import frc.robot.subsystems.io.SwerveDriveIO
import frc.robot.subsystems.io.TurretIO
import frc.robot.wrappers.AbsEncoderWrapper
import frc.robot.wrappers.WrappedTalonFX
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.io.File

class RobotContainer {
    val loop = EventLoop()

    val xbox = XboxController(0)

//    val intakeMotor = SparkMax(30, SparkLowLevel.MotorType.kBrushless)
    val intakeMotor = ThriftyNova(30)

    val swerveJsonDirectory = File(Filesystem.getDeployDirectory(), "swerve")
    val swerveDrive = SwerveParser(swerveJsonDirectory).createSwerveDrive(17.0)

    val driveXPID = ProfiledPIDController(5.0, 0.0, 0.0, TrapezoidProfile.Constraints(1.0, 30.0))
    val driveYPID = ProfiledPIDController(5.0, 0.0, 0.0, TrapezoidProfile.Constraints(1.0, 30.0))

    val swerveDriveIO = SwerveDriveIO(swerveDrive)
    val robotOriginToCameraTransform = Transform3d(
        Translation3d(0.0, 0.0, 0.0),
        Rotation3d(0.0, 0.0, 0.0)
    )
    val swerveDriveSystem = SwerveDrive(swerveDriveIO, driveXPID, driveYPID /*, vision, robotOriginToCameraTransform */)

    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH
    }

    val turretMotor = TalonFX(20)

    init {
        val outputConfig = MotorOutputConfigs()
        outputConfig.Inverted = InvertedValue.CounterClockwise_Positive
        turretMotor.configurator.apply(outputConfig)

        val currentLimitConfig = CurrentLimitsConfigs()
        currentLimitConfig.StatorCurrentLimit = 80.0
        currentLimitConfig.StatorCurrentLimitEnable = true
        currentLimitConfig.SupplyCurrentLimit = 30.0
        currentLimitConfig.SupplyCurrentLimitEnable = true
        turretMotor.configurator.apply(currentLimitConfig)

        val encoderConfig = FeedbackConfigs()
        encoderConfig.SensorToMechanismRatio = 1.0
        encoderConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor
        turretMotor.configurator.apply(encoderConfig)
    }

    val turretMotorWrapped = WrappedTalonFX(turretMotor, 3.0 * 5.0 * (76.0 / 12.0))
    val turretEncoder1 = AbsEncoderWrapper(DutyCycleEncoder(0, 1.0, 0.0))
    val turretEncoder2 = AbsEncoderWrapper(DutyCycleEncoder(1, 1.0, 0.0))
    val turretFusionEncoder = CRTFusion(turretEncoder1, 12, turretEncoder2, 13, 76, 0.0)

    val turretFusionFilter = LinearFilter.movingAverage(10)

    val turretIO = TurretIO(turretMotorWrapped, turretMotorWrapped)

    val turretPID = ProfiledPIDController(0.8, 0.0, 0.0, TrapezoidProfile.Constraints(360.0, 720.0))

    val turret = TurretSubsystem(turretIO, turretPID)
//    val controller = CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT)

    private fun configureBindings() {
        val angles = listOf(0, 45, 90, 135, 180, 225, 270, 315)

        for (angle in angles) {
            xbox.pov(angle, loop).ifHigh { turret.goToAngleCommand(angle.toDouble(), false) }
        }
//        controller.rightBumper().whileTrue(turret.runTurret()).whileFalse(turret.stopTurret())
    }


    val ledStrip = AddressableLED(8)
    val ledStripBuffer = AddressableLEDBuffer(240)
    val lights = Lights(ledStrip, 240, ledStripBuffer)

    init {
        configureBindings()
    }
}