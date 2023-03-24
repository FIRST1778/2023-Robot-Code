package org.frc1778.commands.lights

import com.github.ajalt.colormath.Color
import com.github.ajalt.colormath.model.HSV
import com.github.ajalt.colormath.model.Oklab
import com.github.ajalt.colormath.model.RGB
import com.github.ajalt.colormath.transform.Interpolator
import com.github.ajalt.colormath.transform.interpolator
import org.frc1778.subsystems.DotStar
import org.frc1778.subsystems.Drive
import org.frc1778.Constants
import org.ghrobotics.lib.commands.FalconCommand
import kotlin.math.abs
import kotlin.math.ceil

class BalanceAnimation() : FalconCommand(DotStar) {

	private val MAX_INCLINTATION = 15.0

	private val interpolator = Oklab.interpolator {
		stop(RGB(255, 0, 0).toOklab())
		stop(RGB(255, 255, 0).toOklab())
		stop(RGB(0, 255, 0).toOklab())
	}

	override fun initialize() {
		DotStar.animateOff()
	}

	override fun execute() {
		val currentPixels = DotStar.pixels
		val numLedsPerSection: Int = Constants.LedConstants.NUM_LEDS_PER_SECTION
		val numLedsLit: Int = ceil(numLedsPerSection * (abs(MAX_INCLINTATION - Drive.boardInclination())/MAX_INCLINTATION)).toInt()
		//When we have more lights:
		//val currentPixelSections = currentPixels.chunked(Constants.LedConstants.NUM_LEDS_PER_SECTION)

		currentPixels.mapIndexed {
			index: Int, _ ->
			if(index <= numLedsLit) {
				interpolator.interpolate(index/numLedsPerSection).toSRGB()
			} else {
				RGB(0,0,0)
			}
		}

		DotStar.pixels = currentPixels
		
	}

	

	
	
}