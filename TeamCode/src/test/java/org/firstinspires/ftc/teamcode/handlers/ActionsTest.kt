package org.firstinspires.ftc.teamcode.handlers

import org.junit.Assert.assertEquals
import org.junit.Test

class ActionsTest {
    @Test fun deadzoneSuppressesStickNoise() {
        assertEquals(0.0, Actions.shapeAxis(0.05, 0.1, true), 0.0)
        assertEquals(0.0, Actions.shapeAxis(-0.05, 0.1, true), 0.0)
    }

    @Test fun cubicResponseMatchesPreviousSeason() {
        assertEquals(0.125, Actions.shapeAxis(0.5, 0.1, true), 1e-12)
        assertEquals(-0.125, Actions.shapeAxis(-0.5, 0.1, true), 1e-12)
    }

    @Test fun linearResponseIsConfigurable() {
        assertEquals(0.5, Actions.shapeAxis(0.5, 0.1, false), 0.0)
    }

    @Test fun invalidInputAndDeadzoneProduceZero() {
        assertEquals(0.0, Actions.shapeAxis(Double.NaN, 0.1, true), 0.0)
        assertEquals(0.0, Actions.shapeAxis(1.0, Double.NaN, true), 0.0)
        assertEquals(0.0, Actions.shapeAxis(1.0, -1.0, true), 0.0)
    }
}
