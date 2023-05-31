package org.frc1778.commands.lights

import com.github.ajalt.colormath.model.RGB
import com.github.ajalt.colormath.transform.interpolator
import org.frc1778.Constants
import org.frc1778.animation.BlinkAnimation
import org.frc1778.subsystems.Gyro
import org.frc1778.subsystems.Lights
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.abs
import kotlin.math.ceil

class BalanceAnimation : FalconCommand(Lights) {

    private var currentPixels = Lights.pixels
    private val numLedsPerSection: Int = Constants.LedConstants.NUM_LEDS_PER_SECTION


    private val interpolator = RGB.interpolator {
        stop(RGB.from255(255, 0, 0))
        stop(RGB.from255(255, 255, 0))
        stop(RGB.from255(0, 255, 0))
    }

    override fun initialize() {
        Lights.animateOff()
    }

    override fun execute() {
        //Second abs is to deal with edge cases were read inclination is greater than expected max inclination
        val numLedsLit: Int =
            ceil(numLedsPerSection * abs(MAX_INCLINATION - abs(Gyro.inclination().degrees)) / MAX_INCLINATION).toInt()

        /**
         * This is a bit of a hack to get the lights to light up in the correct order.
         *  - The lights are split into sections based on the number of LEDs you have in a strip.
         *    (This only will work if you have the same number of LEDs per strip)
         *  - The LEDs are then set to the correct color based on an interpolation from red to green
         *  - The sections are then reversed every other section to get the correct order
         */
        Lights.pixels = currentPixels.chunked(Constants.LedConstants.NUM_LEDS_PER_SECTION).mapIndexed {index, section ->
            List(section.size) { sectionIndex: Int ->
                if (sectionIndex + 1 <= numLedsLit) {
                    interpolator.interpolate((sectionIndex.toDouble() / (numLedsPerSection - 1)))
                } else {
                    RGB(0, 0, 0)
                }
            }.toMutableList().apply {
                if(index % 2 == 1) {
                    this.reverse()
                }
            }
        }.flatten().toMutableList()


    }

    override fun end(interrupted: Boolean) {
        if (!interrupted) {
            Lights.setAnimation(
                BlinkAnimation(
                    RGB.from255(0,255,0), RGB, 2, 3
                )
            )
            Lights.animateOn()
        }
    }

    companion object {
    private const val MAX_INCLINATION = 12.0

    }


}
