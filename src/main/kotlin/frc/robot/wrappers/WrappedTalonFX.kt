package frc.robot.wrappers

import com.ctre.phoenix6.hardware.TalonFX

class WrappedTalonFX(val talon: TalonFX, val ratio: Double) : VoltageController, PositionProvider, VelocityProvider {
    override fun setVoltage(voltage: Double) {
        talon.setVoltage(voltage)
    }

    override fun getVoltage(): Double {
        return talon.motorVoltage.valueAsDouble
    }

    override fun setPosition(position: Double) {
        talon.setPosition(position * ratio)
    }

    override fun getPosition(): Double {
        return talon.position.valueAsDouble / ratio
    }

    override fun getCurrent(): Double {
        return talon.supplyCurrent.valueAsDouble
    }

    override fun getVelocity(): Double {
        return talon.velocity.valueAsDouble / ratio
    }
}