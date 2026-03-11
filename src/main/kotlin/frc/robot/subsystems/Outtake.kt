package frc.robot.subsystems

import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.TurretSubsystem.ShootingState
import frc.robot.subsystems.TurretSubsystem.TurretState
import java.util.function.DoubleSupplier

class Outtake(val motor1: MotorController, val motor2: MotorController): SubsystemBase() {
    sealed class OuttakeState {
        class EStop() : OuttakeState()
        class Outtaking(val voltage: Double) : OuttakeState()
        class Init() : OuttakeState()
    }

    var state: OuttakeState = OuttakeState.Init()

    fun estop() {
        this.state = OuttakeState.EStop()
    }

    fun runOuttake(velocity: Double) {
        this.state = OuttakeState.Outtaking(velocity)
    }

    fun controlPeriodic() {
        var voltage = 0.0

        when(val state = this.state) {
            is OuttakeState.EStop -> voltage = (0.0)
            is OuttakeState.Outtaking -> {
                voltage = state.voltage
            }

            is OuttakeState.Init -> {
            }
        }

        motor1.setVoltage(voltage)
        motor2.setVoltage(voltage)
    }

    override fun periodic() {
    }

    fun outtakeDefaultCommand(
        speed: DoubleSupplier
    ): Command {
        return FunctionalCommand(
            { -> Unit },
            { -> runOuttake(speed.asDouble) },
            { _: Boolean -> runOuttake(0.0)},
            { -> false },
            this
        )
    }
}