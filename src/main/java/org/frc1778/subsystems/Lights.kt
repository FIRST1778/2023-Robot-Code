package org.frc1778.subsystems

import com.github.ajalt.colormath.Color
import com.github.ajalt.colormath.model.Oklab
import com.github.ajalt.colormath.model.Oklch
import com.github.ajalt.colormath.model.RGB
import com.github.ajalt.colormath.model.RGBInt
import com.github.ajalt.colormath.transform.EasingFunctions
import com.github.ajalt.colormath.transform.interpolator
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.frc1778.Constants.LedConstants
import org.frc1778.animation.Animation
import org.frc1778.animation.BlinkAnimation
import org.frc1778.animation.GradientAnimation
import org.ghrobotics.lib.commands.FalconSubsystem
import kotlin.random.Random

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
object Lights : FalconSubsystem() {
	var pixels = MutableList<Color>(LedConstants.NUM_LEDS) { RGB(0,0,0) }

    private val redPurpleBlueAnimation =
            GradientAnimation(
                    Oklab.interpolator {
                        easing = EasingFunctions.easeInOut()
                        stop(RGB.from255(0, 0, 255))
                        stop(RGB.from255(255, 0, 255))
                        stop(RGB.from255(255, 0, 0))
                        stop(RGB.from255(255, 0, 255))
                        stop(RGB.from255(0, 0, 255))
                    },
                    6.0
            )


    private val rainbowOklchAnimation =
        GradientAnimation(
            Oklch.interpolator {
                easing = EasingFunctions.easeInOut()
                stop(Oklch(l = 0.8, c = 0.3, h = 0.0))
                stop(Oklch(l = 0.8, c = 0.3, h = 90.0))
                stop(Oklch(l = 0.8, c = 0.3, h = 180.0))
                stop(Oklch(l = 0.8, c = 0.3, h = 270.0))
                stop(Oklch(l = 0.8, c = 0.3, h = 0.0))
            },
            6.0
        )

    val animations = listOf(
        redPurpleBlueAnimation,
        rainbowOklchAnimation
    )

    val rebBlink = BlinkAnimation(RGB.from255(255, 0, 0), RGB, 4, 4)
    private var currentAnimation: Animation = rainbowOklchAnimation
    private var animationEnabled = true

    fun animateOn() {
        currentAnimation.reset()
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

    var ticksLeftPerDit = 0
    var morseCodeLed = false
    val morseCodeGenerator: Sequence<Boolean> = sequence {
        while (true) {
            var someOfTheManyImportantFellas = arrayOf(
                "Apollo",
                "Eli",
                "Gavin",
                "Grace",
                "Lilia",
                "Simon",
                "Zach")
            someOfTheManyImportantFellas.shuffle(Random.Default)
            for (importantFella: String in someOfTheManyImportantFellas) {
                for (letter: Char in importantFella.lowercase()) {
                    val idx = "abcdefghijklmnopqrstuvwxyz".indexOf(letter)
                    for (c: Char in LedConstants.MORSE_CODE[idx]) {
                        if (c == '-') {
                            yield(true)
                            yield(true)
                        }
                        yield(true)
                        yield(false)
                    }
                    yield(false)
                    yield(false)
                }
                yield(false)
                yield(false)
                yield(false)
                yield(false)
            }
        }
    }

    val morseCodeGeneratorIterator = morseCodeGenerator.iterator()

    private val spi = SPI(SPI.Port.kOnboardCS0).apply { setClockRate(4_000_000) }

    private fun emit(frame: ByteArray) {
        spi.write(frame, frame.size)
    }

    private fun startFrame() =
            // The start frame is at least 4 0x00 bytes.
            ByteArray(4) { 0x00.toByte() }

    private fun ledFrame(color: Color): ByteArray {
        val rgbInt = RGBInt.convert(color)
        return byteArrayOf(
            // The first 3 bits (111) begin an LED frame.  The next 5 bits
            // are the brightness, which we always set to full (31 out of 31).
            0xFF.toByte(),
            // Blue-green-red seems to be the norm for color order, but we might
            // need to change this.
            rgbInt.b.toByte(),
            rgbInt.g.toByte(),
            rgbInt.r.toByte()
        )
    }

    private fun endFrame() =
            // The end frame is defined by the datasheet to be 4 0xFF bytes, but
            // the actual requirement seems to be a 1 bit for every two LEDs in
            // the chain, ergo at least one 0xFF byte for every 16 LEDs.
            ByteArray((LedConstants.NUM_LEDS + 15) / 16) { 0xFF.toByte() }

    override fun periodic() {
        emit(startFrame())

        // The LEDs which are farthest from the source need to have their frames
        // emitted first.
        if (animationEnabled) {
            val curr = currentAnimation.get()

            pixels.fill(curr)
            if (currentAnimation.isDone())
                animateOff()
        }

        // Morse code stuff (comment out if you dislike) (looking at you
        // Gavin)
        if (ticksLeftPerDit == 0) {
            ticksLeftPerDit = LedConstants.TICKS_PER_DIT
            // This will lazily generate the next morse code bit. Only issue is I have no idea how it will
            // handle an infinite sequence. We might be able to convert it into a finite loop and just loop
            // over the people in the same order every time.
            morseCodeLed = morseCodeGeneratorIterator.next()
        } else {
            ticksLeftPerDit--
        }
//        pixels[0] = RGB.from255(0, 0, (if (morseCodeLed) 255 else 0))

        pixels.forEach {
            emit(ledFrame(it))
        }

        emit(endFrame())
    }

    init {
        Shuffleboard.getTab("Lights").add(this)
    }

    override fun initSendable(builder: SendableBuilder?) {
        super.initSendable(builder)
        builder!!.addBooleanProperty("Animation Enabled", { animationEnabled }, {})
    }
}
