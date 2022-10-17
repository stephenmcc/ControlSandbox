package controlsandbox;

import controlsandbox.solver.DynamicSystemLinearization;
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

public class CartPoleLQRController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DMatrixRMaj K;
   private final OneDegreeOfFreedomJoint cartJoint;
   private final OneDegreeOfFreedomJoint poleJoint;

   private final YoBoolean randomPoleAngle = new YoBoolean("randomPoleAngle", registry);

   public CartPoleLQRController(CartPoleRobot robot)
   {
      cartJoint = robot.getSCSCartJoint();
      poleJoint = robot.getSCSPoleJoint();

      DynamicSystemLinearization linearization = new DynamicSystemLinearization(robot);
      DMatrixRMaj fixedPointQ = new DMatrixRMaj(2, 1);
      DMatrixRMaj fixedPointQd = new DMatrixRMaj(2, 1);
      fixedPointQ.set(1, 0, Math.PI);
      linearization.solve(fixedPointQ, fixedPointQd);

      RealMatrix A = toHipparchus(linearization.getA());
      RealMatrix B = toHipparchus(linearization.getB());
      RealMatrix Q = new Array2DRowRealMatrix(4, 4);
      RealMatrix R = new Array2DRowRealMatrix(1, 1);

      Q.setEntry(0, 0, 1.0);
      Q.setEntry(1, 1, 1.0);
      Q.setEntry(2, 2, 3.0);
      Q.setEntry(3, 3, 3.0);
      R.setEntry(0, 0, 0.75);

      RiccatiEquationSolver solver = new RiccatiEquationSolverImpl(A, B, Q, R);
      K = toEJML(solver.getK());

      System.out.println(K);
   }

   private final Random random = new Random();

   @Override
   public void doControl()
   {
      if (randomPoleAngle.getValue())
      {
         randomPoleAngle.set(false);
         poleJoint.setQ(Math.PI + EuclidCoreRandomTools.nextDouble(random, 0.4));
      }

      DMatrixRMaj x = new DMatrixRMaj(4, 1);
      x.set(0, 0, 0.0);
      x.set(1, 0, poleJoint.getQ() - Math.PI);
      x.set(2, 0, cartJoint.getQD());
      x.set(3, 0, poleJoint.getQD());

      DMatrixRMaj u = new DMatrixRMaj(1, 1);
      CommonOps_DDRM.mult(K, x, u);

      cartJoint.setTau(-u.get(0, 0));
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
}
