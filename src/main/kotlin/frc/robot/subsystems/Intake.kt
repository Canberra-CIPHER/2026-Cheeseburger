package frc.robot.subsystems

import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Outtake.OuttakeState
import frc.robot.subsystems.TurretSubsystem.TurretState
import java.util.function.DoubleSupplier

class Intake(val motor: MotorController): SubsystemBase() {
    sealed class IntakeState {
        class EStop() : IntakeState()
        class Intaking(val voltage: Double) : IntakeState()
        class Outtaking(val voltage: Double) : IntakeState()
        class Init() : IntakeState()
    }

    var state: IntakeState = IntakeState.Init()

    fun estop() {
        this.state = IntakeState.EStop()
    }

    fun runIntake(velocity: Double) {
        this.state = IntakeState.Intaking(velocity)
    }

    fun runIntakeBackwards(velocity: Double) {
        this.state = IntakeState.Outtaking(velocity)
    }

    fun controlPeriodic() {
        var voltage = 0.0

        when(val state = this.state) {
            is IntakeState.EStop -> voltage = (0.0)
            is IntakeState.Intaking -> {
                voltage = state.voltage
            }
            is IntakeState.Outtaking -> {
                voltage = state.voltage
            }

            is IntakeState.Init -> {
            }
        }

        motor.setVoltage(voltage)
    }

    override fun periodic() {
    }

    fun intakeDefaultCommand(
        speed: DoubleSupplier
    ): Command {
        return FunctionalCommand(
            { -> Unit },
            { -> runIntake(speed.asDouble) },
            { _: Boolean -> runIntake(0.0)},
            { -> false },
            this
        )
    }

    fun outtakeDefaultCommand(
        speed: DoubleSupplier
    ): Command {
        return FunctionalCommand(
            { -> Unit },
            { -> runIntakeBackwards(speed.asDouble) },
            { _: Boolean -> runIntakeBackwards(0.0)},
            { -> false },
            this
        )
    }
}