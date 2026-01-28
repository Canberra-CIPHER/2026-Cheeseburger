package frc.robot.subsystems

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.smartdashboard.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.io.TurretIO


class TurretSubsystem(
    val io: TurretIO,
    var pid: ProfiledPIDController,
//    var feedforward: TurretFeedforward,
//    var motor: DCMotor,
//    var turretModel: MechanismLigament2d,
) :  SubsystemBase() {
    sealed class TurretState {
        class EStop() : TurretState()
        class HoldingAngle(val angle: Double) : TurretState()
        class Init() : TurretState()
    }

    val turretErrorPublisher = NetworkTableInstance.getDefault().getTopic("turret/error").genericPublish("double")
    val turretPositionPublisher = NetworkTableInstance.getDefault().getTopic("turret/position").genericPublish("double")
    val turretVoltagePublisher = NetworkTableInstance.getDefault().getTopic("turret/voltage").genericPublish("double")
    val turretCurrentPublisher = NetworkTableInstance.getDefault().getTopic("turret/current").genericPublish("double")

    var state: TurretState = TurretState.Init()

    fun estop() {
        this.state = TurretState.EStop()
    }

    fun goToAngle(angle: Double) {
        this.state = TurretState.HoldingAngle(angle)
    }

    fun isStable(): Boolean {
        return this.pid.atSetpoint()
    }

    fun getCurrentAngle(): Double {
        return io.positionProvider.getPosition() * 360.0
    }

    fun controlPeriodic() {
        var currentAngle = getCurrentAngle()
        var voltage = 0.0

        when(val state = this.state) {
            is TurretState.EStop -> voltage = (0.0)
            is TurretState.HoldingAngle -> {
                var output = pid.calculate(currentAngle, state.angle)
                voltage = output
            }
            is TurretState.Init -> {

            }
        }

        io.voltageController.setVoltage(voltage)
    }

    override fun periodic() {
        turretErrorPublisher.setDouble(pid.positionError)
        turretPositionPublisher.setDouble(getCurrentAngle())
        turretVoltagePublisher.setDouble(io.voltageController.getVoltage())
        turretCurrentPublisher.setDouble(io.voltageController.getCurrent())
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