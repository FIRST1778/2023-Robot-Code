package org.frc1778.subsystems

import com.github.ajalt.colormath.model.HSV
import com.github.ajalt.colormath.model.RGB
import com.github.ajalt.colormath.transform.Interpolator
import com.github.ajalt.colormath.transform.interpolator
import edu.wpi.first.wpilibj.SPI
import org.ghrobotics.lib.commands.FalconSubsystem
import kotlin.math.ceil
import kotlin.math.floor

object DotStar : FalconSubsystem() {
    private const val LEDS = 10 // ???
    private const val ANIMATION_PER_TICK: Double = 0.1
    private const val TICK_SPEED: Int = 5  // 100 ms
    private var ledsNow: Int = 0
    private var ledsGoal: Int = 0
    private var executesTillAnimate: Int = TICK_SPEED

    fun animateOn() {
        ledsGoal = LEDS
    }

    fun animateOff() {
        ledsGoal = 0
    }

    // Adafruit DotStars are addressable LEDs controlled via an SPI
    // interface.  We use them to display purple or yellow lights so that the
    // human player knows which item they should give to the robot.
    //
    // The DotStars are powered by SK9822 LEDs according to this PDF:
    //     https://cdn-learn.adafruit.com/downloads/pdf/adafruit-dotstar-leds.pdf
    // The APA102 datasheet (which SK9822s are compatible with):
    //     https://cdn-learn.adafruit.com/assets/assets/000/084/592/original/APA102_White_LED.pdf?1574117639
    // A helpful blog post which seems more factually correct (and has better
    // English) than the datasheets:
    //     https://cpldcpu.wordpress.com/2014/11/30/understanding-the-apa102-superled/
    private val spi = SPI(SPI.Port.kOnboardCS0).apply {
        setClockRate(8_000_000)
    }

    /**
     * Red to blue animation using a color interpolator.
     * Color interpreters take in a value form 0 to 1 and interpolates between the colors
     * to create smooth animations.
     * Tool to create gradients [here](ajalt.github.io/colormath/gradient)
     */
    val redToBlueAnimation = GradientAnimation(RGB.interpolator {
        stop(RGB(51, 51, 221))
        stop(RGB(255, 255, 255))
        stop(RGB(255, 0, 0))
    }, 4.0)

    /**
     * Gradiant Animation by interpolating between colors over a given time interval. Get sequence by calling the [next] function
     *
     * @property interpolator
     * @param totalTime
     */
    class GradientAnimation(private val interpolator: Interpolator<RGB>, totalTime: Double) {
        //Step amount for interpolation & Convert time to the closest periodic step
        private val interpolatorStep = 1 / (totalTime - (totalTime % .2))
        private var currentInterpolationValue = 0.0

        fun next(): RGB {
            currentInterpolationValue += interpolatorStep
            if (currentInterpolationValue > 1.0) {
                currentInterpolationValue = 0.0
            }

            return interpolator.interpolate(currentInterpolationValue)
        }
    }

    private fun emit(frame: ByteArray) {
        spi.write(frame, frame.size)
    }

    private fun startFrame() =
        // The start frame is at least 4 0x00 bytes.
        ByteArray(4) { 0x00.toByte() }

    private fun ledFrame(r: Int, g: Int, b: Int, a: Int) =
        byteArrayOf(
            // The first 3 bits (111) begin an LED frame.  The next 5 bits
            // are the brightness, which we always set to full (31 out of 31).
            (0xE0 or (a and 0x1F)).toByte(),
            // Blue-green-red seems to be the norm for color order, but we might
            // need to change this.
            b.toByte(), g.toByte(), r.toByte()
        )

    private fun endFrame() =
        // The end frame is defined by the datasheet to be 4 0xFF bytes, but
        // the actual requirement seems to be a 1 bit for every two LEDs in
        // the chain, ergo at least one 0xFF byte for every 16 LEDs.
        ByteArray((LEDS + 15) / 16) { 0xFF.toByte() }

    override fun periodic() {
        executesTillAnimate--
        if (executesTillAnimate != 0) {
            return
        } else {
            executesTillAnimate = TICK_SPEED
        }

        emit(startFrame())

        // Calculate animation.
        val dx: Double = (ledsGoal - ledsNow).toDouble() * ANIMATION_PER_TICK
        ledsNow += (if (dx >= 0) ceil(dx) else floor(dx)).toInt()

        val led: ByteArray = ledFrame(0x80, 0x00, 0x80, 0xFF)
        val off: ByteArray = ledFrame(0, 0, 0, 0)
        // The LEDs which are farthest from the source need to have their frames
        // emitted first.
        repeat(LEDS - ledsNow) {
            emit(off)
        }
        repeat(ledsNow) {
            emit(led)
        }

        emit(endFrame())
    }
}