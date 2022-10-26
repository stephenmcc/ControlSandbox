package controlsandbox.acrobot;

import controlsandbox.solver.DynamicSystem;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.avatar.networkProcessor.quadTreeHeightMap.HeightQuadTreeMessageConverter;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.robotModels.FullRobotModelWrapper;
import us.ihmc.robotics.robotDescription.*;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.RobotFromDescription;

public class AcrobotRobot implements DynamicSystem
{
   public static double SHOULDER_MASS = 5.0;
   public static double SHOULDER_INERTIA = 0.16;
   public static double SHOULDER_LENGTH = 0.5;
   public static double SHOULDER_COM_OFFSET = 0.22;

   public static double ELBOW_MASS = 3.0;
   public static double ELBOW_INERTIA = 0.09;
   public static double ELBOW_LENGTH = 0.3;
   public static double ELBOW_COM_OFFSET = 0.16;

   public static double g = 9.81;

   public static final String SHOULDER_JOINT_NAME = "shoulder";
   public static final String ELBOW_JOINT_NAME = "elbow";

   private final RobotDescription robotDescription;
   private final PinJointDescription shoulderJoint;
   private final PinJointDescription elbowJoint;

   private final RobotFromDescription robot;

   public AcrobotRobot()
   {
      robotDescription = new RobotDescription("Acrobot");
      shoulderJoint = new PinJointDescription(SHOULDER_JOINT_NAME, new Vector3D(), Axis3D.X);
      elbowJoint = new PinJointDescription(ELBOW_JOINT_NAME, new Vector3D(0.0, 0.0, -SHOULDER_LENGTH), Axis3D.X);

      robotDescription.addRootJoint(shoulderJoint);

      LinkDescription shoulderLink = new LinkDescription("shoulderLink");
      shoulderLink.setMass(SHOULDER_MASS);
      shoulderLink.setMomentOfInertia(SHOULDER_INERTIA, SHOULDER_INERTIA, SHOULDER_INERTIA);
      shoulderLink.setCenterOfMassOffset(0.0, 0.0, -SHOULDER_COM_OFFSET);

      LinkDescription elbowLink = new LinkDescription("elbowLink");
      elbowLink.setMass(ELBOW_MASS);
      elbowLink.setMomentOfInertia(ELBOW_INERTIA, ELBOW_INERTIA, ELBOW_INERTIA);
      elbowLink.setCenterOfMassOffset(0.0, 0.0, -ELBOW_COM_OFFSET);

      LinkGraphicsDescription shoulderGraphics = new LinkGraphicsDescription();
      shoulderGraphics.translate(0.0, 0.0, -SHOULDER_LENGTH);
      shoulderGraphics.addCylinder(SHOULDER_LENGTH, 0.015, YoAppearance.Red());
      shoulderLink.setLinkGraphics(shoulderGraphics);

      LinkGraphicsDescription elbowGraphics = new LinkGraphicsDescription();
      elbowGraphics.translate(0.0, 0.0, -ELBOW_LENGTH);
      elbowGraphics.addCylinder(ELBOW_LENGTH, 0.015, YoAppearance.Blue());
      elbowLink.setLinkGraphics(elbowGraphics);

      shoulderJoint.setLink(shoulderLink);
      shoulderJoint.addJoint(elbowJoint);
      elbowJoint.setLink(elbowLink);

      robot = new RobotFromDescription(robotDescription);
   }

   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }

   public RobotFromDescription getRobot()
   {
      return robot;
   }

   public OneDegreeOfFreedomJoint getShoulderJoint()
   {
      return (OneDegreeOfFreedomJoint) robot.getJoint(SHOULDER_JOINT_NAME);
   }

   public OneDegreeOfFreedomJoint getElbowJoint()
   {
      return (OneDegreeOfFreedomJoint) robot.getJoint(ELBOW_JOINT_NAME);
   }

   @Override
   public DMatrixRMaj computeH(DMatrixRMaj q)
   {
      DMatrixRMaj H = new DMatrixRMaj(2, 2);
      double q1 = q.get(0, 0);
      double q2 = q.get(1, 0);

      H.set(0, 0, SHOULDER_INERTIA + ELBOW_INERTIA + ELBOW_MASS * MathTools.square(SHOULDER_LENGTH) + 2.0 * ELBOW_MASS * SHOULDER_LENGTH * ELBOW_COM_OFFSET * Math.cos(q2));
      H.set(0, 1, ELBOW_INERTIA + ELBOW_MASS * SHOULDER_LENGTH * ELBOW_COM_OFFSET * Math.cos(q2));
      H.set(1, 0, ELBOW_INERTIA + ELBOW_MASS * SHOULDER_LENGTH * ELBOW_COM_OFFSET * Math.cos(q2));
      H.set(1, 1, ELBOW_INERTIA);

      return H;
   }

   @Override
   public DMatrixRMaj computeC(DMatrixRMaj q, DMatrixRMaj qd)
   {
      DMatrixRMaj C = new DMatrixRMaj(2, 2);
      double q1 = q.get(0, 0);
      double q2 = q.get(1, 0);
      double qd1 = qd.get(0, 0);
      double qd2 = qd.get(1, 0);

      C.set(0, 0, -2.0 * ELBOW_MASS * SHOULDER_LENGTH * ELBOW_COM_OFFSET * Math.sin(q2) * qd2);
      C.set(0, 1, - ELBOW_MASS * SHOULDER_LENGTH * ELBOW_COM_OFFSET * Math.sin(q2) * qd2);
      C.set(1, 0, ELBOW_MASS * SHOULDER_LENGTH * ELBOW_COM_OFFSET * Math.sin(q2) * qd1);
      C.set(1, 1, 0.0);

      return C;
   }

   @Override
   public DMatrixRMaj computeG(DMatrixRMaj q)
   {
      DMatrixRMaj G = new DMatrixRMaj(2, 1);
      double q1 = q.get(0, 0);
      double q2 = q.get(1, 0);

      G.set(0, 0, - SHOULDER_MASS * g * SHOULDER_COM_OFFSET * Math.sin(q1) - ELBOW_MASS * g * (SHOULDER_LENGTH * Math.sin(q1) + ELBOW_COM_OFFSET * Math.sin(q1 + q2)));
      G.set(1, 0, - ELBOW_MASS * g * ELBOW_COM_OFFSET * Math.sin(q1 + q2));

      return G;
   }

   @Override
   public DMatrixRMaj computeB()
   {
      DMatrixRMaj B = new DMatrixRMaj(2, 1);
      B.set(1, 0, 1.0);
      return B;
   }

   @Override
   public int getPlantDegreesOfFreedom()
   {
      return 2;
   }

   @Override
   public int getControlDegreesOfFreedom()
   {
      return 1;
   }

   public static void main(String[] args)
   {
      AcrobotRobot robot = new AcrobotRobot();

      RigidBodyBasics elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      for (JointDescription rootJoint : robot.getRobotDescription().getRootJoints())
      {
         FullRobotModelWrapper.addJointRecursive(rootJoint, elevator);
      }
      MultiBodySystemBasics multiBodySystemBasics = MultiBodySystemBasics.toMultiBodySystemBasics(elevator);
      RevoluteJoint shoulder = (RevoluteJoint) multiBodySystemBasics.findJoint(SHOULDER_JOINT_NAME);
      RevoluteJoint elbow = (RevoluteJoint) multiBodySystemBasics.findJoint(ELBOW_JOINT_NAME);

      double q1 = 0.4;
      double q2 = -1.1;

      DMatrixRMaj q = new DMatrixRMaj(2, 1);
      q.set(0, 0, q1);
      q.set(1, 0, q2);

      CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(multiBodySystemBasics);
      shoulder.setQ(q1);
      elbow.setQ(q2);
      DMatrixRMaj massMatrixMecano = massMatrixCalculator.getMassMatrix();

      DMatrixRMaj Hmanual = robot.computeH(q);
      System.out.println("H manual:\n" + Hmanual);

      System.out.println("H mecano:\n" + massMatrixMecano);
   }
}
