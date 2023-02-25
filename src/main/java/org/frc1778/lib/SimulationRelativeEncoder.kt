package org.frc1778.lib

import com.revrobotics.REVLibError
import com.revrobotics.RelativeEncoder

class SimulationRelativeEncoder : RelativeEncoder{
    var simulationInverted = false;
    var simulationPosition = 0.0;
    var simulationVelocity = 0.0;
    override fun setInverted(inverted: Boolean): REVLibError {
        this.simulationInverted = inverted;
        return REVLibError.kOk
    }

    override fun getInverted(): Boolean {
        return simulationInverted;
    }

    override fun getPosition(): Double {
        return simulationPosition;
    }

    override fun getVelocity(): Double {
        return simulationVelocity;
    }

    override fun setPosition(position: Double): REVLibError {
        this.simulationPosition = position
        return REVLibError.kOk
    }

    override fun setPositionConversionFactor(factor: Double): REVLibError {
        TODO("Not yet implemented")
    }

    override fun setVelocityConversionFactor(factor: Double): REVLibError {
        TODO("Not yet implemented")
    }

    override fun getPositionConversionFactor(): Double {
        TODO("Not yet implemented")
    }

    override fun getVelocityConversionFactor(): Double {
        TODO("Not yet implemented")
    }

    override fun setAverageDepth(depth: Int): REVLibError {
        TODO("Not yet implemented")
    }

    override fun getAverageDepth(): Int {
        TODO("Not yet implemented")
    }

    override fun setMeasurementPeriod(period_ms: Int): REVLibError {
        TODO("Not yet implemented")
    }

    override fun getMeasurementPeriod(): Int {
        TODO("Not yet implemented")
    }

    override fun getCountsPerRevolution(): Int {
        TODO("Not yet implemented")
    }
}