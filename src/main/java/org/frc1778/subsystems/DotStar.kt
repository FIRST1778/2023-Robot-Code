package org.frc1778.subsystems

import com.github.ajalt.colormath.Color
import com.github.ajalt.colormath.model.Oklab
import com.github.ajalt.colormath.model.RGB
import com.github.ajalt.colormath.transform.interpolator
import edu.wpi.first.wpilibj.SPI
import org.frc1778.Constants
import org.frc1778.animation.Animation
import org.frc1778.animation.GradientAnimation
import org.ghrobotics.lib.commands.FalconSubsystem

object DotStar : FalconSubsystem() {
    private const val LEDS = Constants.LedConstants.NUM_LEDS
    private const val ANIMATION_PER_TICK: Double = 0.1
    private const val TICK_SPEED: Int = 5 // 100 ms
    private var ledsNow: Int = 0
    private var ledsGoal: Int = 0
    private var executesTillAnimate: Int = TICK_SPEED
	var pixels = MutableList<RGB>(LEDS) {RGB(0,0,0)}

    /**
     * Red to blue animation using a color interpolator.
     */
    val redToBlueAnimation =
            GradientAnimation(
                    RGB.interpolator {
                        stop(RGB.from255(51, 51, 221))
                        stop(RGB.from255(255, 255, 255))
                        stop(RGB.from255(255, 0, 0))
                    },
                    4.0
            )

    val redPurpleBlueAnimation =
            GradientAnimation(
                    Oklab.interpolator {
                        stop(RGB.from255(0, 0, 255).toOklab())
                        stop(RGB.from255(255, 255, 255).toOklab())
                        stop(RGB.from255(255, 0, 0).toOklab())
                        stop(RGB.from255(0, 0, 255).toOklab())
                        stop(RGB.from255(255, 255, 255).toOklab())
                    },
                    4.0
            )

    private var currentAnimation: Animation = redPurpleBlueAnimation
    private var animationEnabled = true

    fun animateOn() {
        animationEnabled = true
    }

    fun animateOff() {
		animationEnabled = false
    }

    fun setAnimation(animation: Animation) {
        currentAnimation = animation
        currentAnimation.reset()
    }



    fun fill(color: Color) {
        pixels.fill(color.toSRGB())
    }


    // Adafruit DotStars are addressable LEDs controlled via an SPI
    // interface.  We use them to display purple or yellow lights so that the
    // human player knows which item they should give to the robot.
    //
    // The DotStars are powered by SK9822 LEDs according to this PDF:
    //     https://cdn-learn.adafruit.com/downloads/pdf/adafruit-dotstar-leds.pdf
    // The APA102 datasheet (which SK9822s are compatible with):
    //
    // https://cdn-learn.adafruit.com/assets/assets/000/084/592/original/APA102_White_LED.pdf?1574117639
    // A helpful blog post which seems more factually correct (and has better
    // English) than the datasheets:
    //     https://cpldcpu.wordpress.com/2014/11/30/understanding-the-apa102-superled/
    private val spi = SPI(SPI.Port.kOnboardCS0).apply { setClockRate(4_000_000) }

    private fun emit(frame: ByteArray) {
        spi.write(frame, frame.size)
    }

    private fun startFrame() =
            // The start frame is at least 4 0x00 bytes.
            ByteArray(4) { 0x00.toByte() }

    private fun ledFrame(rgb: RGB) =
            byteArrayOf(
                    // The first 3 bits (111) begin an LED frame.  The next 5 bits
                    // are the brightness, which we always set to full (31 out of 31).
                    0xFF.toByte(),
                    // Blue-green-red seems to be the norm for color order, but we might
                    // need to change this.
                    rgb.blueInt.toByte(),
                    rgb.greenInt.toByte(),
                    rgb.redInt.toByte()
            )

    private fun endFrame() =
            // The end frame is defined by the datasheet to be 4 0xFF bytes, but
            // the actual requirement seems to be a 1 bit for every two LEDs in
            // the chain, ergo at least one 0xFF byte for every 16 LEDs.
            ByteArray((LEDS + 15) / 16) { 0xFF.toByte() }

    override fun periodic() {
        emit(startFrame())

        // The LEDs which are farthest from the source need to have their frames
        // emitted first.
        if (animationEnabled) {
            val led = ledFrame(currentAnimation.get())
            repeat(LEDS) { emit(led) }
        } else {
			pixels.forEach {
				emit(ledFrame(it))
			}
		}

        emit(endFrame())
    }
}
