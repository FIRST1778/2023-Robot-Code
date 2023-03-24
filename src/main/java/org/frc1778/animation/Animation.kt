package org.frc1778.animation

import com.github.ajalt.colormath.Color
import com.github.ajalt.colormath.model.HSV
import com.github.ajalt.colormath.model.Oklab
import com.github.ajalt.colormath.model.RGB
import com.github.ajalt.colormath.transform.Interpolator
import com.github.ajalt.colormath.transform.interpolator
import edu.wpi.first.wpilibj.Timer
import kotlin.math.sin
import kotlin.math.PI
import kotlin.math.abs


interface Animation {
    fun get(): RGB
}

/**
 * Gradiant Animation by interpolating between colors over a given time interval. Call start() to
 * start the timer, then call get() to get the current RGB value.
 *
 * @property interpolator
 * @param totalTime
 */
class GradientAnimation<T : Color>(
        private val interpolator: Interpolator<T>,
        private val totalTime: Double
) : Animation {
    private val timer = Timer()

    init {
        timer.start()
    }
    override fun get(): RGB {
        val interp = abs(sin(PI * timer.get() / totalTime))
        return interpolator.interpolate(interp).toSRGB()
    }
}