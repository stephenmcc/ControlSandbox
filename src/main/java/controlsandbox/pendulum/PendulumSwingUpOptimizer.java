package controlsandbox.pendulum;

import org.hipparchus.optim.ConvergenceChecker;
import org.hipparchus.optim.InitialGuess;
import org.hipparchus.optim.PointValuePair;
import org.hipparchus.optim.nonlinear.scalar.GoalType;
import org.hipparchus.optim.nonlinear.scalar.ObjectiveFunction;
import org.hipparchus.optim.nonlinear.scalar.gradient.NonLinearConjugateGradientOptimizer;
import org.hipparchus.optim.nonlinear.scalar.noderiv.CMAESOptimizer;

public class PendulumSwingUpOptimizer
{
   private static final int N = 100;
   private static final double dt = 0.05;
   private static final int maxIterations = 10000;
   private static final double epsilonCost = 1e-5;

   private final double initialQ = 0.0;
   private final double initialQd = 0.0;
   private final double finalQ = Math.PI;
   private final double finalQd = 0.0;

   public PendulumSwingUpOptimizer(PendulumRobot robot)
   {
      NonLinearConjugateGradientOptimizer.Formula formula = NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE;
//      NonLinearConjugateGradientOptimizer.Formula formula = NonLinearConjugateGradientOptimizer.Formula.FLETCHER_REEVES;

      ConvergenceChecker<PointValuePair> convergenceChecker = (iteration, previous, current) -> (iteration > maxIterations || Math.abs(current.getSecond() - previous.getSecond()) < epsilonCost);

//      CMAESOptimizer optimizer = new CMAESOptimizer(10000, -1, );
//
//      GoalType goalType = GoalType.MINIMIZE;
//      ObjectiveFunction objectiveFunction = new ObjectiveFunction();
//
//      InitialGuess initialGuess = new InitialGuess();
//
//      optimizer.optimize(goalType, );
   }
}
