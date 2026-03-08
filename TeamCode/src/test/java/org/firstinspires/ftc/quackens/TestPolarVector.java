package org.firstinspires.ftc.quackens;

import static junit.framework.TestCase.assertEquals;
import org.junit.Test;

public class TestPolarVector {
    @Test
    public void testToPoint() {
        PolarVector rv = new PolarVector(1, 0);
        assertEquals(1, rv.toPoint().x, 0.0005);
    }
}
