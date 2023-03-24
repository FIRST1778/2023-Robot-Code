package org.frc1778.subsystems

import com.github.ajalt.colormath.model.HSV
import com.github.ajalt.colormath.model.RGB
import com.github.ajalt.colormath.transform.Interpolator
import com.github.ajalt.colormath.transform.interpolator
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.commands.FalconSubsystem
import kotlin.math.ceil
import kotlin.math.floor
import kotlin.math.PI
import kotlin.math.sin

object DotStar : FalconSubsystem() {
    private const val LEDS = 7
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
        setClockRate(4_000_000)
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
     * Gradiant Animation by interpolating between colors over a given time interval.
     * Call start() to start the timer, then call get() to get the current RGB value.
     *
     * @property interpolator
     * @param totalTime
     */
    class GradientAnimation(private val interpolator: Interpolator<RGB>, private val totalTime: Double) {
        private val timer = Timer()
        fun start() {
            timer.start()
        }
        fun get(): RGB {
            val interp = sin(PI * timer.get() / totalTime)
            return interpolator.interpolate(interp)
        }
    }

    private fun emit(frame: ByteArray) {
        spi.write(frame, frame.size)
    }

    private fun startFrame() =
        // The start frame is at least 4 0x00 bytes.
        ByteArray(4) { 0x00.toByte() }

    private fun ledFrame(rgb: RGB) = byteArrayOf(
        // The first 3 bits (111) begin an LED frame.  The next 5 bits
        // are the brightness, which we always set to full (31 out of 31).
        0xFF.toByte(),
        // Blue-green-red seems to be the norm for color order, but we might
        // need to change this.
        rgb.blueInt.toByte(), rgb.greenInt.toByte(), rgb.redInt.toByte()
    )

    private fun endFrame() =
        // The end frame is defined by the datasheet to be 4 0xFF bytes, but
        // the actual requirement seems to be a 1 bit for every two LEDs in
        // the chain, ergo at least one 0xFF byte for every 16 LEDs.
        ByteArray((LEDS + 15) / 16) { 0xFF.toByte() }

    init {
        redToBlueAnimation.start()
    }

    override fun periodic() {
        emit(startFrame())

        // The LEDs which are farthest from the source need to have their frames
        // emitted first.
        val led = ledFrame(redToBlueAnimation.get())
        repeat(LEDS) {
            emit(led)
        }

        emit(endFrame())
    }
}