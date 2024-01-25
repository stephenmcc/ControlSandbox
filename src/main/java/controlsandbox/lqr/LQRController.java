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
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.Random;

import static controlsandbox.acrobot.AcrobotRobot.ELBOW_JOINT_NAME;
import static controlsandbox.acrobot.AcrobotRobot.SHOULDER_JOINT_NAME;
import static controlsandbox.cartPole.CartPoleRobot.CART_JOINT_NAME;
import static controlsandbox.cartPole.CartPoleRobot.POLE_JOINT_NAME;

public class LQRController implements Controller
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final DMatrixRMaj K;

   private final DynamicSystem dynamicSystem;
   private final DMatrixRMaj fixedPoint;

   private final ControllerInput controllerInput;
   private final ControllerOutput controllerOutput;
   private final OneDoFJointReadOnly[] allJoints;
   private final OneDoFJointReadOnly[] controllableJoints;

   private final OneDoFJointStateBasics[] allJointOutput;
   private final OneDoFJointStateBasics[] controllableJointOutput;

   private final YoBoolean perturb = new YoBoolean("perturb", registry);
   private final Random random = new Random();

   public LQRController(DynamicSystem dynamicSystem,
                        DMatrixRMaj fixedPoint,
                        double positionCost,
                        double velocityCost,
                        double controlCost,
                        String[] controllableJoints,
                        String[] allJoints,
                        ControllerInput controllerInput,
                        ControllerOutput controllerOutput)
   {
      this.dynamicSystem = dynamicSystem;
      this.fixedPoint = fixedPoint;

      this.controllableJoints = new OneDoFJointReadOnly[controllableJoints.length];
      this.controllableJointOutput = new OneDoFJointStateBasics[allJoints.length];
      for (int i = 0; i < controllableJoints.length; i++)
      {
         this.controllableJoints[i] = (OneDoFJointReadOnly) controllerOutput.getInput().findJoint(controllableJoints[i]);
         this.controllableJointOutput[i] = (OneDoFJointStateBasics) controllerOutput.getJointOutput(controllableJoints[i]);
      }

      this.allJoints = new OneDoFJointReadOnly[allJoints.length];
      this.allJointOutput = new OneDoFJointStateBasics[allJoints.length];
      for (int i = 0; i < allJoints.length; i++)
      {
         this.allJoints[i] = (OneDoFJointReadOnly) controllerOutput.getInput().findJoint(allJoints[i]);
         this.allJointOutput[i] = (OneDoFJointStateBasics) controllerOutput.getJointOutput(allJoints[i]);
      }

      this.controllerInput = controllerInput;
      this.controllerOutput = controllerOutput;

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
         Q.setEntry(i + dynamicSystem.getPlantDegreesOfFreedom(), i + dynamicSystem.getPlantDegreesOfFreedom(),  velocityCost);
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
         perturb.set(false, false);

         double deltaQMax = 0.02;
         double deltaQdMax = 0.05;

         for (JointReadOnly joint : controllerInput.getInput().getAllJoints())
         {
            if (joint instanceof SimOneDoFJointBasics oneDoFJoint)
            {
               double q = oneDoFJoint.getQ();
               double qd = oneDoFJoint.getQd();
               oneDoFJoint.setQ(q + EuclidCoreRandomTools.nextDouble(random, deltaQMax));
               oneDoFJoint.setQd(qd + EuclidCoreRandomTools.nextDouble(random, deltaQdMax));
            }
         }
      }

      for (int i = 0; i < allJointOutput.length; i++)
      {
         allJointOutput[i].setEffort(0.0);
      }

      int plantDoFs = dynamicSystem.getPlantDegreesOfFreedom();
      DMatrixRMaj x = new DMatrixRMaj(2 * plantDoFs, 1);
      for (int i = 0; i < plantDoFs; i++)
      {
         x.set(i, 0, allJoints[i].getQ() - fixedPoint.get(i, 0));
         x.set(i + plantDoFs, 0, allJoints[i].getQd());
      }

      DMatrixRMaj u = new DMatrixRMaj(dynamicSystem.getControlDegreesOfFreedom(), 1);
      CommonOps_DDRM.mult(K, x, u);

      for (int i = 0; i < controllableJoints.length; i++)
      {
         controllableJointOutput[i].setEffort(- u.get(i, 0));
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

   public static ControllerDefinition createLQRControllerDefinition(DynamicSystem dynamicSystem,
                                                                    DMatrixRMaj fixedPoint,
                                                                    double positionCost,
                                                                    double velocityCost,
                                                                    double controlCost,
                                                                    String[] controllableJoints,
                                                                    String[] allJoints)
   {
      return (input, output) -> new LQRController(dynamicSystem,
                                                  fixedPoint,
                                                  positionCost,
                                                  velocityCost,
                                                  controlCost,
                                                  controllableJoints,
                                                  allJoints,
                                                  input,
                                                  output);
   }

   public static ControllerDefinition setupForCartPole(CartPoleRobot cartPoleRobot)
   {
      DMatrixRMaj fixedPoint = new DMatrixRMaj(2, 1);
      fixedPoint.set(0, 0, 0.0);
      fixedPoint.set(1, 0, Math.PI);

      String[] controllableJoints = {CART_JOINT_NAME};
      String[] allJoints = {CART_JOINT_NAME, POLE_JOINT_NAME};

      return createLQRControllerDefinition(cartPoleRobot, fixedPoint, 1.0, 3.0, 0.75, controllableJoints, allJoints);
   }

   public static ControllerDefinition setupForAcrobot(AcrobotRobot acrobotRobot)
   {
      DMatrixRMaj fixedPoint = new DMatrixRMaj(2, 1);
      fixedPoint.set(0, 0, Math.PI);

      String[] controllableJoints = {ELBOW_JOINT_NAME};
      String[] allJoints = {SHOULDER_JOINT_NAME, ELBOW_JOINT_NAME};

      return createLQRControllerDefinition(acrobotRobot, fixedPoint, 1.0, 3.0, 0.75, controllableJoints, allJoints);
   }
}
