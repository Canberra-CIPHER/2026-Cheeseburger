package frc.robot.subsystems

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import java.util.Map


class Lights(val leds: AddressableLED, val numLeds: Int, var colour: AddressableLEDBuffer) {
    val bisexual = LEDPattern.steps(Map.of(0, Color.kDeepPink, 0.33, Color.kPurple, 0.66, Color.kDarkBlue))
    val italian = LEDPattern.steps(Map.of(0, Color.kGreen, 0.33, Color.kWhite, 0.66, Color.kRed))
    val trans = LEDPattern.steps(Map.of(0, Color.kLightBlue, 0.2, Color.kPink, 0.4, Color.kWhite, 0.6, Color.kPink, 0.8, Color.kLightBlue))
    val lesbian = LEDPattern.steps(Map.of(0, Color.kOrangeRed, 0.2, Color.kOrange, 0.4, Color.kWhite, 0.6, Color.kPink, 0.8, Color.kDeepPink))
    val rainbow = LEDPattern.rainbow(255, 128)

    fun setLights(pattern: LEDPattern) {
        pattern.applyTo(colour)
        leds.setData(colour)
    }

    /*fun seychellian() {
        for (i in 0..<leds.numLeds) {
            val position = i / leds.numLeds.toDouble()

            var colour = Color.BLACK

            if (i in 0..<6 || i in 24..<29 || i in 48..<52 || i in 72..<75 || i in 96..<98 || i in 120..<122 || i == 144 || i == 168 || i == 192) {
                colour = Color.DARKBLUE
            }
            else if (i in 6..<14 || i in 29..<37 || i in 52..<60 || i in 75..<83 || i in 98..<105 || i in 122..<127 || i in 145..<149 || i in 169..<172 || i in 193..<195) {
                colour = Color.YELLOW
            }
            else if (i in 14..<24 || i in 37..<48|| i in 60..<69 || i in 83..<92 || i in 105..<115 || i in 127..<137 || i in 149..< 158 || i in 172..< 179 || i in 195..< 200 || i in 216..< 220) {
                colour = Color.RED
            }
            else if (i in 69..<72 || i in 92..<96 || i in 115..<120 || i in 137..<144 || i in 158..<165 || i in 179..<186 || i in 200..<207 || i in 220..<225) {
                colour = Color.WHITE
            }
            else if (i in 165..<168 || i in 186..<198 || i in 207..<222 || i in 225..<246) {
                colour = Color.GREEN
            }

            leds.setColour(i, colour)
        }
    }

    fun progress() {
        for (i in 0..<leds.numLeds) {
            val position = i / leds.numLeds.toDouble()

            var colour = when {
                position < 0.2 -> Color.RED
                position < 0.4 -> Color.ORANGE
                position < 0.6 -> Color.YELLOW
                position < 0.8 -> Color.GREEN
                else -> Color.BLUE
            }

            if (i == 0 || i == 1 || i == 25 || i == 26 || i == 50 || i == 51 || i == 75 || i == 76  || i == 100 || i == 101 || i == 124 || i == 125 || i == 147 || i == 148 || i == 170 || i == 171 || i == 193 || i == 194 || i == 216 || i == 217) {
                colour = Color.SKYBLUE
            }
            else if (i == 2 || i == 3 || i == 27 || i == 28 || i == 52 || i == 53  || i == 77 || i == 78  || i == 102 || i == 103 || i == 126 || i == 127 || i == 149 || i == 150 || i == 172 || i == 173 || i == 195 || i == 196 || i == 218 || i == 219) {
                colour = Color.SADDLEBROWN
            }
            else if (i == 4 || i == 5 || i == 29 || i == 30 || i == 54 || i == 55  || i == 79 || i == 80  || i == 104 || i == 105 || i == 128 || i == 129 || i == 151 || i == 152 || i == 174 || i == 175 || i == 197 || i == 198 || i == 220 || i == 221) {
                colour = Color.BLACK
            }
            else if (i == 24 || i == 48 || i == 49 || i == 73 || i == 74  || i == 98 || i == 99 || i == 122 || i == 123 || i == 145 || i == 146 || i == 168 || i == 169 || i == 192) {
                colour = Color.HOTPINK
            }
            else if (i == 72 || i == 96 || i == 97 || i == 120 || i == 121 || i == 144) {
                colour = Color.WHITE
            }

            leds.setColour(i, colour)
        }
    }

    fun yugoslavian() {
        for (i in 0..<leds.numLeds) {
            val position = i / leds.numLeds.toDouble()

            var colour = when {
                position < 0.3 -> Color.BLUE
                position < 0.6 -> Color.WHITE
                else -> Color.RED
            }

            if (i == 59 || i == 82 || i == 84 || i == 105 || i == 109 || i == 130 || i == 131 || i == 132 || i == 154 || i == 156) {
                colour = Color.YELLOW
            }
            else if (i == 83 || i == 106 || i == 107 || i == 108) {
                colour = Color.RED
            }

            leds.setColour(i, colour)
        }
    }*/
}