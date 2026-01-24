package frc.robot.wrappers

import edu.wpi.first.wpilibj.DutyCycleEncoder

class AbsEncoderWrapper(val encoder: DutyCycleEncoder) : PositionProvider {
    var offset: Double = 0.0

    override fun setPosition(position: Double) {
        offset = position
    }
    override fun getPosition(): Double {
        return encoder.get() + offset
    }
}