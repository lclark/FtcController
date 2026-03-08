package org.firstinspires.ftc.quackens;

import static junit.framework.TestCase.assertEquals;
import org.junit.Test;

public class TestPidController {
    @Test
    public void test1() {
        PidController c = new PidController(1.0, 1.0, 1.0);
        c.setTarget(0);
        double result = c.compute(1.0, 0);
        assertEquals(0.0, result, 0.00005);
    }
}
