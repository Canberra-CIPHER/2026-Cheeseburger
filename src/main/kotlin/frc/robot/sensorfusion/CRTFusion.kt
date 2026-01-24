package frc.robot.sensorfusion

import frc.robot.wrappers.PositionProvider

class CRTFusion(
    val positionProvider1: PositionProvider,
    val pos1Teeth: Int,
    val positionProvider2: PositionProvider,
    val pos2Teeth: Int,
    val commonTeeth: Int,
    var offset: Double
): PositionProvider {
    val params = beizout(pos1Teeth, pos2Teeth)

    fun beizout(pos1Teeth: Int, pos2Teeth: Int): Triple<Int, Int, Int> {
        var oldR = pos1Teeth
        var r = pos2Teeth
        var oldS = 1
        var s = 0
        var oldT = 0
        var t = 1

        while (r != 0) {
            val quotient = oldR / r
            var tmp = oldR - (quotient * r)
            oldR = r
            r = tmp

            tmp = oldS - (quotient * s)
            oldS = s
            s = tmp

            tmp = oldT - (quotient * t)
            oldT = t
            t = tmp
        }

        return Triple(oldS, oldT, oldR)
    }

    override fun setPosition(position: Double) {
        offset = position
    }

    override fun getPosition(): Double {
        val nTeethPassedA = positionProvider1.getPosition() * pos1Teeth
        val nTeethPassedB = positionProvider2.getPosition() * pos2Teeth

        val m = (pos1Teeth * pos2Teeth) / params.third
        val x = (nTeethPassedA * params.second * pos2Teeth + nTeethPassedB * params.first * pos1Teeth) / params.third

        return ((x % m.toDouble()) / commonTeeth) + offset
    }
}