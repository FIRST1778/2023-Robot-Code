package org.frc1778

import com.github.ajalt.colormath.model.Oklch
import com.github.ajalt.colormath.model.RGB
import com.github.ajalt.colormath.transform.Interpolator
import com.github.ajalt.colormath.transform.interpolator
import org.junit.jupiter.api.Test
import java.io.FileOutputStream

class LightTests {

    private fun <T : com.github.ajalt.colormath.Color> writeInterpolation(interpolation: Interpolator<T>, name: String) {
        val fileOut = FileOutputStream("src/test/out/$name.txt")
        for(interp in 0.0..1.0 step 0.001) {
           fileOut.write((interpolation.interpolate(interp).toSRGB().toHex() + "\n").toByteArray())
        }
        fileOut.close()
    }

    @Test
    fun redPurpleBlue() {
        writeInterpolation(RGB.interpolator {
                        stop(RGB.from255(0, 0, 255))
                        stop(RGB.from255(255, 0, 255))
                        stop(RGB.from255(255, 0, 0))
                        stop(RGB.from255(255, 0, 255))
                        stop(RGB.from255(0, 0, 255))
                    }, "Red Purple Blue")
    }

    @Test
    fun RGBRainbow() {
        writeInterpolation(RGB.interpolator {
                stop(RGB.from255(255,0,0))
                stop(RGB.from255(255,128,0))
                stop(RGB.from255(255,255,0))
                stop(RGB.from255(0,255,0))
                stop(RGB.from255(0,128,255))
                stop(RGB.from255(0,0,255))
                stop(RGB.from255(128,0,255))
                stop(RGB.from255(0,255,255))
                stop(RGB.from255(128,0,255))
                stop(RGB.from255(0,0,255))
                stop(RGB.from255(0,128,255))
                stop(RGB.from255(0,255,0))
                stop(RGB.from255(255,255,0))
                stop(RGB.from255(255,128,0))
                stop(RGB.from255(255,0,0))
            }, "RGB Rainbow")
    }

    @Test
    fun oklhcRainbow() {
        writeInterpolation(Oklch.interpolator {
                stop(Oklch(l = 0.8, c = 0.3, h = 0.0))
                stop(Oklch(l = 0.8, c = 0.3, h = 90.0))
                stop(Oklch(l = 0.8, c = 0.3, h = 180.0))
                stop(Oklch(l = 0.8, c = 0.3, h = 270.0))
                stop(Oklch(l = 0.8, c = 0.3, h = 0.0))
            }, "Oklhc Rainbow")
    }



    private infix fun ClosedRange<Double>.step(step: Double): Iterable<Double> {
        require(start.isFinite())
        require(endInclusive.isFinite())
        require(step > 0.0) { "Step must be positive, was: $step." }
        val sequence = generateSequence(start) { previous ->
            if (previous == Double.POSITIVE_INFINITY) return@generateSequence null
            val next = previous + step
            if (next > endInclusive) null else next
        }
        return sequence.asIterable()
    }
}