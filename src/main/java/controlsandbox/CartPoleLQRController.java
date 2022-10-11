package controlsandbox;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.DefectCorrectionCARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.EigenvectorCARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.SignFunctionCARESolver;
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

      CARESolver careSolver = new DefectCorrectionCARESolver(new SignFunctionCARESolver());
      DMatrixRMaj Q = new DMatrixRMaj(4, 4);
      DMatrixRMaj R = new DMatrixRMaj(1, 1);

      Q.set(0, 0, 0.0);
      Q.set(1, 1, 1.0);
      Q.set(2, 2, 10.0);
      Q.set(3, 3, 1.0);

      R.set(0, 0, 0.5);

      System.out.println("A:");
      System.out.println(robot.getA_lin());
      System.out.println("B:");
      System.out.println(robot.getB_lin());
      System.out.println("Q:");
      System.out.println(Q);
      System.out.println("R:");
      System.out.println(R);

      careSolver.setMatrices(robot.getA_lin(), robot.getB_lin(), CommonOps_DDRM.identity(4), Q, R);
      DMatrixRMaj P = careSolver.computeP();

      System.out.println("P:");
      System.out.println(P);

      K = new DMatrixRMaj(1, 4);
      DMatrixRMaj Rinv = new DMatrixRMaj(1, 1);
      CommonOps_DDRM.invert(R, Rinv);
      DMatrixRMaj BT = new DMatrixRMaj(1, 4);
      CommonOps_DDRM.transpose(robot.getB_lin(), BT);

      DMatrixRMaj BTS = new DMatrixRMaj(1, 4);
      CommonOps_DDRM.mult(BT, P, BTS);
      CommonOps_DDRM.mult(Rinv, BTS, K);

      System.out.println("K:");
      System.out.println(K);

      // Found using python's control package
      K.set(0, 0, 0.0);
      K.set(0, 1, 169.0);
      K.set(0, 2, -4.47);
      K.set(0, 3, 23.96);
   }

   private final Random random = new Random();

   @Override
   public void doControl()
   {
      if (randomPoleAngle.getValue())
      {
         randomPoleAngle.set(false);
         poleJoint.setQ(Math.PI + EuclidCoreRandomTools.nextDouble(random, 0.2));
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
}
