package org.frc1778.subsystems

import edu.wpi.first.wpilibj.SPI
import org.ghrobotics.lib.commands.FalconSubsystem

object DotStar : FalconSubsystem() {
    // Adafruit DotStars are addressable LEDs controlled via an SPI
    // interface.  We use them to display blue or yellow lights so that the
    // human player knows which item they should give to the robot.
    //
    // The DotStars are powered by SK9822 LEDs according to this PDF:
    //     https://cdn-learn.adafruit.com/downloads/pdf/adafruit-dotstar-leds.pdf
    // The APA102 datasheet (which SK9822s are compatible with):
    //     https://cdn-learn.adafruit.com/assets/assets/000/084/592/original/APA102_White_LED.pdf?1574117639
    // A helpful blog post which seems more factually correct (and has better
    // English) than the datasheets:
    //     https://cpldcpu.wordpress.com/2014/11/30/understanding-the-apa102-superled/
    enum class Color { Yellow, Blue }

    private const val NUM_LEDS = 10 // ???

    private val spi = SPI(SPI.Port.kOnboardCS0).apply {
        setClockRate(8_000_000)
    }

    private fun makeLedFrame(r: Int, g: Int, b: Int): ByteArray {
        return byteArrayOf(
            // The first 3 bits (111) begin an LED frame.  The next 5 bits
            // are the brightness, which we always set to full (31 out of 31).
            0xFF.toByte(),
            // Blue-green-red seems to be the norm for color order, but we might
            // need to change this.
            b.toByte(), g.toByte(), r.toByte()
        )
    }

    fun display(color: Color) {
        // The start frame is at least 4 0x00 bytes.
        val startFrame = ByteArray(4) { 0x00.toByte() }
        spi.write(startFrame, startFrame.size)

        for (i in 1..NUM_LEDS) {
            val ledFrame: ByteArray = when (color) {
                Color.Yellow -> makeLedFrame(0xFF, 0xFF, 0x00)
                Color.Blue -> makeLedFrame(0x00, 0x00, 0xFF)
            }
            spi.write(ledFrame, ledFrame.size)
        }

        // The end frame is defined by the datasheet to be 4 0xFF bytes, but
        // the actual requirement seems to be a 1 bit for every two LEDs in
        // the chain, ergo at least one 0xFF byte for every 16 LEDs.
        val endFrame = ByteArray((NUM_LEDS+15)/16) { 0xFF.toByte() }
        spi.write(endFrame, endFrame.size)
    }
}