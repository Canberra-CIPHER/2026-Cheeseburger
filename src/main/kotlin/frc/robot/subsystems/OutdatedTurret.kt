package frc.robot.subsystems

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import yams.gearing.GearBox
import yams.gearing.MechanismGearing
import yams.mechanisms.config.PivotConfig
import yams.mechanisms.positional.Pivot
import yams.motorcontrollers.SmartMotorController
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode
import yams.motorcontrollers.remote.TalonFXWrapper
import java.util.function.Supplier


class OutdatedTurretSubsystem : SubsystemBase() {
    private val turretMotor = TalonFX(20)
    private val motorConfig: SmartMotorControllerConfig? = SmartMotorControllerConfig(this)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        .withClosedLoopController(
            72.0,
            0.0,
            0.0,
            Units.DegreesPerSecond.of(720.0),
            Units.DegreesPerSecondPerSecond.of(1440.0)
        )
        .withGearing(MechanismGearing(GearBox.fromReductionStages(3.0, 5.0, 76.0 / 12.0)))
        .withIdleMode(MotorMode.BRAKE)
        .withMotorInverted(false)
        .withTelemetry("TurretMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH) // Power Optimization
        .withStatorCurrentLimit(Units.Amps.of(25.0))
        .withClosedLoopRampRate(Units.Seconds.of(0.0))
        .withOpenLoopRampRate(Units.Seconds.of(0.0))

    private val turretSMC: SmartMotorController = TalonFXWrapper(
        turretMotor,
        DCMotor.getKrakenX60(1),
        motorConfig
    )

    private val turretConfig: PivotConfig = PivotConfig(turretSMC)
        .withStartingPosition(Units.Degrees.of(0.0))
        .withWrapping(Units.Degrees.of(-180.0), Units.Degrees.of(180.0))
        .withSoftLimits(Units.Degrees.of(-200.0), Units.Degrees.of(200.0))
        .withHardLimit(Units.Degrees.of(-200.0), Units.Degrees.of(200.0))
        .withTelemetry("TurretMech", SmartMotorControllerConfig.TelemetryVerbosity.HIGH) // Telemetry
        .withMOI(Units.Meters.of(0.25), Units.Kilograms.of(2.5))

    private val turret = Pivot(turretConfig)

    fun transformAngle(angle: Angle): Angle {
        var a = angle
        if (a.gt(Units.Degrees.of(180.0))) {
            a -= Units.Degrees.of(360.0)
        }
        return a
    }

    fun setAngle(angle: Angle): Command {
        return turret.setAngle(transformAngle(angle))
    }

    fun setAngleDirect(angle: Angle) {
        turretSMC.setPosition(transformAngle(angle))
    }

    fun setAngle(angleSupplier: Supplier<Angle>): Command {
        return turret.setAngle { -> transformAngle(angleSupplier.get()) }
    }

    val angle: Angle
        get() = turret.angle

    fun sysId(): Command? {
        return turret.sysId(
            Units.Volts.of(4.0),  // maximumVoltage
            Units.Volts.per(Units.Second).of(0.5),  // step
            Units.Seconds.of(8.0) // duration
        )
    }

    fun setDutyCycle(dutyCycleSupplier: Supplier<Double?>?): Command? {
        return turret.set(dutyCycleSupplier)
    }

    fun setDutyCycle(dutyCycle: Double): Command? {
        return turret.set(dutyCycle)
    }

    override fun periodic() {
        turret.updateTelemetry()
    }

    override fun simulationPeriodic() {
        turret.simIterate()
    }
}