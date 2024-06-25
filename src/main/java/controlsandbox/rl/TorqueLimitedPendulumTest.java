package controlsandbox.rl;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

import java.util.Random;

import static controlsandbox.rl.TorqueLimitedPendulum.*;

public class TorqueLimitedPendulumTest
{
   @Test
   public void testPendulum()
   {
      int tests = 5000;
      Random random = new Random(3290);
      double epsilon = 1e-10;

      for (int i = 0; i < tests; i++)
      {
         double q = EuclidCoreRandomTools.nextDouble(random, 5.0 * Math.PI);
         double qd = EuclidCoreRandomTools.nextDouble(random, MAX_VELOCITY);

         TorqueLimitedPendulum pendulum = new TorqueLimitedPendulum(q, qd);
         EuclidCoreTestTools.assertAngleEquals(q, pendulum.getQ(), 0.5 * POSITION_GRID_SIZE + epsilon);
         Assertions.assertTrue(Math.abs(qd - pendulum.getQd()) <= 0.5 * VELOCITY_GRID_SIZE + epsilon);
      }
   }
}
