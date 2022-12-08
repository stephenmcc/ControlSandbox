package controlsandbox.lqr;

import controlsandbox.acrobot.AcrobotRobot;
import controlsandbox.cartPole.CartPoleRobot;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RiccatiEquationSolver;
import org.hipparchus.linear.RiccatiEquationSolverImpl;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.Random;
import java.util.function.Consumer;

public class LQRController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final DMatrixRMaj K;

   private final DynamicSystem dynamicSystem;
   private final DMatrixRMaj fixedPoint;
   private final OneDegreeOfFreedomJoint[] controllableJoints;
   private final OneDegreeOfFreedomJoint[] allJoints;

   private final YoBoolean perturb = new YoBoolean("perturb", registry);
   private final Random random = new Random();
   private Consumer<Random> perturbRunnable = r -> {};

   public LQRController(DynamicSystem dynamicSystem,
                        DMatrixRMaj fixedPoint,
                        double positionCost,
                        double velocityCost,
                        double controlCost,
                        OneDegreeOfFreedomJoint[] controllableJoints,
                        OneDegreeOfFreedomJoint[] allJoints)
   {
      this.dynamicSystem = dynamicSystem;
      this.controllableJoints = controllableJoints;
      this.allJoints = allJoints;
      this.fixedPoint = fixedPoint;

      DynamicSystemLinearization linearization = new DynamicSystemLinearization(dynamicSystem);
      DMatrixRMaj fixedPointQd = new DMatrixRMaj(dynamicSystem.getPlantDegreesOfFreedom(), 1);
      linearization.solve(fixedPoint, fixedPointQd);

      RealMatrix A = toHipparchus(linearization.getA());
      RealMatrix B = toHipparchus(linearization.getB());
      RealMatrix Q = new Array2DRowRealMatrix(2 * dynamicSystem.getPlantDegreesOfFreedom(), 2 * dynamicSystem.getPlantDegreesOfFreedom());
      RealMatrix R = new Array2DRowRealMatrix(dynamicSystem.getControlDegreesOfFreedom(), dynamicSystem.getControlDegreesOfFreedom());

      for (int i = 0; i < dynamicSystem.getPlantDegreesOfFreedom(); i++)
      {
         Q.setEntry(i, i, positionCost);
         Q.setEntry(i + dynamicSystem.getPlantDegreesOfFreedom(), i + dynamicSystem.getPlantDegreesOfFreedom(), velocityCost);
      }

      for (int i = 0; i < dynamicSystem.getControlDegreesOfFreedom(); i++)
      {
         R.setEntry(i, i, controlCost);
      }

      RiccatiEquationSolver solver = new RiccatiEquationSolverImpl(A, B, Q, R);
      K = toEJML(solver.getK());
   }

   @Override
   public void doControl()
   {
      if (perturb.getValue())
      {
         perturb.set(false);
         perturbRunnable.accept(random);
      }

      int plantDoFs = dynamicSystem.getPlantDegreesOfFreedom();
      DMatrixRMaj x = new DMatrixRMaj(2 * plantDoFs, 1);
      for (int i = 0; i < plantDoFs; i++)
      {
         x.set(i, 0, allJoints[i].getQ() - fixedPoint.get(i, 0));
         x.set(i + plantDoFs, 0, allJoints[i].getQD());
      }

      DMatrixRMaj u = new DMatrixRMaj(dynamicSystem.getControlDegreesOfFreedom(), 1);
      CommonOps_DDRM.mult(K, x, u);

      for (int i = 0; i < controllableJoints.length; i++)
      {
         controllableJoints[i].setTau(- u.get(i, 0));
      }
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public void setPerturbRunnable(Consumer<Random> perturbRunnable)
   {
      this.perturbRunnable = perturbRunnable;
   }

   private static RealMatrix toHipparchus(DMatrixRMaj ejmlMatrix)
   {
      Array2DRowRealMatrix matrix = new Array2DRowRealMatrix(ejmlMatrix.numRows, ejmlMatrix.numCols);

      for (int i = 0; i < ejmlMatrix.getNumRows(); i++)
      {
         for (int j = 0; j < ejmlMatrix.getNumCols(); j++)
         {
            matrix.setEntry(i, j, ejmlMatrix.get(i, j));
         }
      }

      return matrix;
   }

   private static DMatrixRMaj toEJML(RealMatrix matrix)
   {
      DMatrixRMaj ejmlMatrix = new DMatrixRMaj(matrix.getRowDimension(), matrix.getColumnDimension());

      for (int i = 0; i < ejmlMatrix.getNumRows(); i++)
      {
         for (int j = 0; j < ejmlMatrix.getNumCols(); j++)
         {
            ejmlMatrix.set(i, j, matrix.getEntry(i, j));
         }
      }

      return ejmlMatrix;
   }

   public static RobotController setupForCartPole(CartPoleRobot cartPoleRobot)
   {
      DMatrixRMaj fixedPoint = new DMatrixRMaj(2, 1);
      fixedPoint.set(1, 0, Math.PI);
      OneDegreeOfFreedomJoint[] controllableJoints = {cartPoleRobot.getSCSCartJoint()};
      OneDegreeOfFreedomJoint[] allJoints = {cartPoleRobot.getSCSCartJoint(), cartPoleRobot.getSCSPoleJoint()};

      LQRController controller = new LQRController(cartPoleRobot, fixedPoint, 1.0, 3.0, 0.75, controllableJoints, allJoints);
      controller.setPerturbRunnable(random -> cartPoleRobot.getSCSPoleJoint().setQ(Math.PI + EuclidCoreRandomTools.nextDouble(random, 0.4)));

      return controller;
   }

   public static RobotController setupForAcrobot(AcrobotRobot acrobotRobot)
   {
      DMatrixRMaj fixedPoint = new DMatrixRMaj(2, 1);
      fixedPoint.set(0, 0, Math.PI);
      OneDegreeOfFreedomJoint[] controllableJoints = {acrobotRobot.getElbowJoint()};
      OneDegreeOfFreedomJoint[] allJoints = {acrobotRobot.getShoulderJoint(), acrobotRobot.getElbowJoint()};

      LQRController controller = new LQRController(acrobotRobot, fixedPoint, 1.0, 3.0, 0.75, controllableJoints, allJoints);
      controller.setPerturbRunnable(random -> acrobotRobot.getElbowJoint().setQ(EuclidCoreRandomTools.nextDouble(random, 0.2)));

      return controller;
   }
}
