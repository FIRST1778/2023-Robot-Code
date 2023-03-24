package org.frc1778.animation

import com.github.ajalt.colormath.Color
import com.github.ajalt.colormath.ColorSpace
import com.github.ajalt.colormath.model.HSV
import com.github.ajalt.colormath.model.Oklab
import com.github.ajalt.colormath.model.RGB
import com.github.ajalt.colormath.transform.Interpolator
import com.github.ajalt.colormath.transform.interpolator
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.mathematics.units.SIUnit
import kotlin.math.sin
import kotlin.math.PI
import kotlin.math.abs


interface Animation {
    fun get(): RGB
    fun reset()
}

/**
 * Gradiant Animation by interpolating between colors over a given time interval. Call start() to
 * start the timer, then call get() to get the current RGB value. Color interpreters take in a value form 0
 * to 1 and interpolates between the colors to create smooth animations. Tool to create
 * gradients [here](https://ajalt.github.io/colormath/gradient)
 *
 * @property interpolator
 * @param totalTime
 */
class GradientAnimation<T : Color>(
    private val interpolator: Interpolator<T>, private val totalTime: Double
) : Animation {
    private val timer = Timer()


    override fun get(): RGB {
        timer.start()
        val interp = (timer.get() / totalTime) % 1
        return interpolator.interpolate(interp).toSRGB()
    }

    override fun reset() {
        timer.reset()
    }
}

class BlinkAnimation<T : Color>(
    private val color: T, colorSpace: ColorSpace<T>, frequency: Int, private val count: Int? = null
) : Animation {

    val timer = Timer()


    private val interpolator = colorSpace.interpolator {
        stop(color)
        stop(RGB(0, 0, 0))
        stop(color)
    }

    /**
     * Interpolation shows color twice so our total time should be (1/frequency)/2
     */
    private val timePerAnimation = (1.0 / frequency) / 2.0

    @Suppress("NullableBooleanElvis")
    override fun get(): RGB {
        timer.start()
        return if (count?.let { timer.get() < it * timePerAnimation } ?: true) {
            val interp = (timer.get() / timePerAnimation) % 1
            interpolator.interpolate(interp).toSRGB()
        } else color.toSRGB()
    }



    override fun reset() {
        timer.reset()
    }
}
