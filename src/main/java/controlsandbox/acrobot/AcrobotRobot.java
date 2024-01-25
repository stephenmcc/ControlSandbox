package controlsandbox.acrobot;

import controlsandbox.lqr.DynamicSystem;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;

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

   private final RobotDefinition robotDefinition;
   private final RigidBodyDefinition rootBodyDefinition;
   private final RevoluteJointDefinition shoulderJoint;
   private final RevoluteJointDefinition elbowJoint;

   private final Robot robot;

   public AcrobotRobot()
   {
      robotDefinition = new RobotDefinition("Acrobot");
      rootBodyDefinition = new RigidBodyDefinition("root");
      shoulderJoint = new RevoluteJointDefinition(SHOULDER_JOINT_NAME, new Vector3D(), Axis3D.X);
      elbowJoint = new RevoluteJointDefinition(ELBOW_JOINT_NAME, new Vector3D(0.0, 0.0, -SHOULDER_LENGTH), Axis3D.X);

      robotDefinition.setRootBodyDefinition(rootBodyDefinition);

      RigidBodyDefinition shoulderLink = new RigidBodyDefinition("shoulderLink");
      shoulderLink.setMass(SHOULDER_MASS);
      shoulderLink.setMomentOfInertia(new MomentOfInertiaDefinition(SHOULDER_INERTIA, SHOULDER_INERTIA, SHOULDER_INERTIA));
      shoulderLink.setCenterOfMassOffset(0.0, 0.0, -SHOULDER_COM_OFFSET);

      RigidBodyDefinition elbowLink = new RigidBodyDefinition("elbowLink");
      elbowLink.setMass(ELBOW_MASS);
      elbowLink.setMomentOfInertia(new MomentOfInertiaDefinition(ELBOW_INERTIA, ELBOW_INERTIA, ELBOW_INERTIA));
      elbowLink.setCenterOfMassOffset(0.0, 0.0, -ELBOW_COM_OFFSET);

      VisualDefinitionFactory shoulderGraphics = new VisualDefinitionFactory();
      shoulderGraphics.appendTranslation(0.0, 0.0, -0.5 * SHOULDER_LENGTH);
      shoulderGraphics.addCylinder(SHOULDER_LENGTH, 0.015, ColorDefinitions.Red());
      shoulderLink.addVisualDefinitions(shoulderGraphics.getVisualDefinitions());

      VisualDefinitionFactory elbowGraphics = new VisualDefinitionFactory();
      elbowGraphics.appendTranslation(0.0, 0.0, -0.5 * ELBOW_LENGTH);
      elbowGraphics.addCylinder(ELBOW_LENGTH, 0.015, ColorDefinitions.Blue());
      elbowLink.addVisualDefinitions(elbowGraphics.getVisualDefinitions());

      rootBodyDefinition.addChildJoint(shoulderJoint);
      shoulderJoint.setSuccessor(shoulderLink);
      shoulderLink.addChildJoint(elbowJoint);
      elbowJoint.setSuccessor(elbowLink);

      robot = new Robot(robotDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);
//      addIntertialEllipsoidsToVisualizer(robot);
   }

   public RobotDefinition getRobotDefinition()
   {
      return robotDefinition;
   }

   public Robot getRobot()
   {
      return robot;
   }

   public SimOneDoFJointBasics getShoulderJoint()
   {
      return (SimOneDoFJointBasics) robot.getJoint(SHOULDER_JOINT_NAME);
   }

   public SimOneDoFJointBasics getElbowJoint()
   {
      return (SimOneDoFJointBasics) robot.getJoint(ELBOW_JOINT_NAME);
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

      G.set(0, 0, SHOULDER_MASS * g * SHOULDER_COM_OFFSET * Math.sin(q1) + ELBOW_MASS * g * (SHOULDER_LENGTH * Math.sin(q1) + ELBOW_COM_OFFSET * Math.sin(q1 + q2)));
      G.set(1, 0, ELBOW_MASS * g * ELBOW_COM_OFFSET * Math.sin(q1 + q2));

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
      RigidBodyBasics elevator = robot.getRobotDefinition().newInstance(ReferenceFrame.getWorldFrame());

      MultiBodySystemBasics multiBodySystemBasics = MultiBodySystemBasics.toMultiBodySystemBasics(elevator);
      RevoluteJoint shoulder = (RevoluteJoint) multiBodySystemBasics.findJoint(SHOULDER_JOINT_NAME);
      RevoluteJoint elbow = (RevoluteJoint) multiBodySystemBasics.findJoint(ELBOW_JOINT_NAME);

      double q1 = 0.4;
      double q2 = -1.1;
      double qd1 = 0.4;
      double qd2 = -0.2;

      DMatrixRMaj q = new DMatrixRMaj(2, 1);
      q.set(0, 0, q1);
      q.set(1, 0, q2);
      DMatrixRMaj qd = new DMatrixRMaj(2, 1);
      qd.set(0, 0, qd1);
      qd.set(1, 0, qd2);

      CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(multiBodySystemBasics);
      massMatrixCalculator.setEnableCoriolisMatrixCalculation(true);
      shoulder.setQ(q1);
      elbow.setQ(q2);
      shoulder.setQd(qd1);
      elbow.setQd(qd2);
      elevator.updateFramesRecursively();

      DMatrixRMaj massMatrixMecano = massMatrixCalculator.getMassMatrix();
      DMatrixRMaj Cmecano = massMatrixCalculator.getCoriolisMatrix();

      DMatrixRMaj Hmanual = robot.computeH(q);
      DMatrixRMaj Cmanual = robot.computeC(q, qd);

      System.out.println("H manual:\n" + Hmanual);
      System.out.println("H mecano:\n" + massMatrixMecano);

      System.out.println("C manual:\n" + Cmanual);
      System.out.println("C mecano:\n" + Cmecano);

      DMatrixRMaj CqdMan = new DMatrixRMaj(2, 1);
      DMatrixRMaj CqdMec = new DMatrixRMaj(2, 1);

      CommonOps_DDRM.mult(Cmanual, qd, CqdMan);
      CommonOps_DDRM.mult(Cmecano, qd, CqdMec);

      System.out.println("Cqd man:\n" + CqdMan);
      System.out.println("Cqd mec:\n" + CqdMec);

      GravityCoriolisExternalWrenchMatrixCalculator gravityCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(elevator);
      gravityCalculator.setGravitionalAcceleration(-g);
      gravityCalculator.compute();

      DMatrixRMaj Gmec = gravityCalculator.getJointTauMatrix();
      DMatrixRMaj Gman = robot.computeG(q);

      System.out.println("G man:\n" + Gman);
      System.out.println("G mec:\n" + Gmec);
   }
}
