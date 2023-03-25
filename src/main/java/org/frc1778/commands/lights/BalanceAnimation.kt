package org.frc1778.commands.lights

import com.github.ajalt.colormath.model.RGB
import com.github.ajalt.colormath.transform.interpolator
import org.frc1778.subsystems.DotStar
import org.frc1778.subsystems.Drive
import org.frc1778.Constants
import org.frc1778.animation.BlinkAnimation
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.abs
import kotlin.math.ceil

class BalanceAnimation() : FalconCommand(DotStar) {

    private val MAX_INCLINATION = 20.0
    var currentPixels = DotStar.pixels
    val numLedsPerSection: Int = Constants.LedConstants.NUM_LEDS_PER_SECTION


    private val interpolator = RGB.interpolator {
        stop(RGB.from255(255, 0, 0))
        stop(RGB.from255(255, 255, 0))
        stop(RGB.from255(0, 255, 0))
    }

    override fun initialize() {
        DotStar.animateOff()
    }

    override fun execute() {
        //Second abs is to deal with edge cases were read inclination is greater than expected max inclination
        val numLedsLit: Int =
            ceil(numLedsPerSection * abs(MAX_INCLINATION - abs(Math.toDegrees(Drive.boardInclination()))) / MAX_INCLINATION).toInt()
        //When we have more lights:
//        val currentPixelSections = currentPixels.chunked(Constants.LedConstants.NUM_LEDS_PER_SECTION).mapIndexed { index, section ->
//            if (index % 2 != 0) section.reversed() else section
//        }

        currentPixels = List(currentPixels.size) { index: Int ->
            if (index+1 <= numLedsLit) {
                interpolator.interpolate((index.toDouble() / (numLedsPerSection-1)))
            } else {
                RGB(0, 0, 0)
            }
        }.toMutableList()

        DotStar.pixels = currentPixels
    }

    override fun end(interrupted: Boolean) {
        if (!interrupted) {
            DotStar.setAnimation(
                BlinkAnimation(
                    RGB.from255(0,255,0), RGB, 6
                )
            )
            DotStar.animateOn()

        }
    }


}