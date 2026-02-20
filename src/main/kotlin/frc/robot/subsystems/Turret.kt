package frc.robot.subsystems

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.smartdashboard.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.io.TurretIO
import java.util.function.DoubleSupplier


class TurretSubsystem(
    val io: TurretIO,
    var pid1: ProfiledPIDController,
    var pid2: ProfiledPIDController,
    var feedforward: SimpleMotorFeedforward
//    var feedforward: TurretFeedforward,
//    var motor: DCMotor,
//    var turretModel: MechanismLigament2d,
) :  SubsystemBase() {
    sealed class TurretState {
        class EStop() : TurretState()
        class HoldingAngle(val angle: Double) : TurretState()
        class Init() : TurretState()
    }

    sealed class ShootingState {
        class EStop() : ShootingState()
        class Shooting(val velocity: Double) : ShootingState()
        class Neutral() : ShootingState()
    }

    val turretErrorPublisher = NetworkTableInstance.getDefault().getTopic("turret/error").genericPublish("double")
    val turretPositionPublisher = NetworkTableInstance.getDefault().getTopic("turret/position").genericPublish("double")
    val turretVoltagePublisher = NetworkTableInstance.getDefault().getTopic("turret/voltage").genericPublish("double")
    val turretCurrentPublisher = NetworkTableInstance.getDefault().getTopic("turret/current").genericPublish("double")
    val turretSetpointPublisher = NetworkTableInstance.getDefault().getTopic("turret/setpoint").genericPublish("double")
    val shootVelocityPublisher = NetworkTableInstance.getDefault().getTopic("shoot/velocity").genericPublish("double")
    val shootSetpointPublisher = NetworkTableInstance.getDefault().getTopic("shoot/setpoint").genericPublish("double")

    var state: TurretState = TurretState.Init()
    var shootState: ShootingState = ShootingState.Neutral()

    fun estop() {
        this.state = TurretState.EStop()
    }

    fun goToAngle(angle: Double) {
        this.state = TurretState.HoldingAngle(angle)
    }

    fun isStable(): Boolean {
        return this.pid1.atSetpoint()
    }

    fun getCurrentAngle(): Double {
        return io.positionProvider.getPosition() * 360.0
    }

    fun getCurrentVelocity(): Double {
        return io.velocityProvider.getVelocity()
    }

    fun runTurret(velocity: Double) {
        this.shootState = ShootingState.Shooting(velocity)
    }

    fun controlPeriodic() {
        var currentAngle = getCurrentAngle()
        var currentVelocity = getCurrentVelocity()
        var voltage = 0.0

        when(val state = this.state) {
            is TurretState.EStop -> voltage = (0.0)
            is TurretState.HoldingAngle -> {
                var output = pid1.calculate(currentAngle, state.angle)
                voltage = output
            }
            is TurretState.Init -> {

            }
        }

        io.voltageControllerSpin.setVoltage(voltage)

        when(val shootState = this.shootState) {
            is ShootingState.EStop -> voltage = (0.0)
            is ShootingState.Shooting -> {
                var output = pid2.calculate(currentVelocity, shootState.velocity)
                voltage = output + feedforward.calculate(shootState.velocity)
            }
            is ShootingState.Neutral -> {

            }
        }

        io.voltageControllerShoot.setVoltage(voltage)
    }

    override fun periodic() {
        turretErrorPublisher.setDouble(pid1.positionError)
        turretPositionPublisher.setDouble(getCurrentAngle())
        turretVoltagePublisher.setDouble(io.voltageControllerSpin.getVoltage())
        turretCurrentPublisher.setDouble(io.voltageControllerSpin.getCurrent())
        turretSetpointPublisher.setDouble(pid1.setpoint.position)
        shootVelocityPublisher.setDouble(io.velocityProvider.getVelocity())
        shootSetpointPublisher.setDouble(pid2.setpoint.position)
    }

    fun goToAngleCommand(angle: Double, continuous: Boolean): Command {
        return FunctionalCommand(
            { -> this.goToAngle(angle) },
            { -> Unit },
            { _ -> Unit },
            { -> !continuous && this.isStable() },
            this
        )
    }

    fun shootDefaultCommand(
        speed: DoubleSupplier
    ): Command {
        return FunctionalCommand(
            { -> Unit },
            { -> runTurret(speed.asDouble) },
            { _: Boolean -> runTurret(0.0)},
            { -> false },
            this
        )
    }

 //   val sim = TurretSim(feedforward.kv, feedforward.ka, motor, minAngle, maxAngle, true, 0.0)

    /*override fun simulationPeriodic() {
        if (io.voltageController is WrappedSparkMax) {
            var simMotor = io.voltageController.sim
            simMotor?.iterate(sim.velocityMetersPerSecond, 12.0, 0.02)
        }

        println("Sim Periodic " + io.voltageController.getVoltage())

        sim.setInputVoltage(io.voltageController.getVoltage())
        sim.update(0.02)

        turretModel.length = sim.positionMeters
    }*/
}