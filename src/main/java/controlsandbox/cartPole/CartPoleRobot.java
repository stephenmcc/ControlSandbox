package controlsandbox.cartPole;

import controlsandbox.lqr.DynamicSystem;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;

public class CartPoleRobot implements DynamicSystem
{
   public static double CART_MASS = 5.0;
   public static double POLE_MASS = 3.0;
   public static double POLE_COM_OFFSET = 0.3;
   public static double g = 9.81;

   public static final String CART_JOINT_NAME = "cartJoint";
   public static final String POLE_JOINT_NAME = "poleJoint";

   private final RobotDefinition robotDefinition;
   private final RigidBodyDefinition rootBodyDefinition;
   private final PrismaticJointDefinition cartJoint;
   private final RevoluteJointDefinition poleJoint;

   private final Robot robot;

   private final DMatrixRMaj A_lin;
   private final DMatrixRMaj B_lin;

   public CartPoleRobot()
   {
      robotDefinition = new RobotDefinition("CartPole");
      rootBodyDefinition = new RigidBodyDefinition("root");
      cartJoint = new PrismaticJointDefinition(CART_JOINT_NAME, new Vector3D(), Axis3D.X);
      poleJoint = new RevoluteJointDefinition(POLE_JOINT_NAME, new Vector3D(), Axis3D.Y.negated());

      RigidBodyDefinition cartLink = new RigidBodyDefinition("cartLink");
      cartLink.setMass(CART_MASS);
      cartLink.setMomentOfInertia(new MomentOfInertiaDefinition(0.5, 0.5, 0.5));
      cartLink.setCenterOfMassOffset(0.0, 0.0, 0.0);

      RigidBodyDefinition poleLink = new RigidBodyDefinition("poleLink");
      poleLink.setMass(POLE_MASS);
      poleLink.setMomentOfInertia(new MomentOfInertiaDefinition(0.2, 1.0, 0.2));
      poleLink.setCenterOfMassOffset(0.0, 0.0, -POLE_COM_OFFSET);

      VisualDefinitionFactory cartGraphics = new VisualDefinitionFactory();
      cartGraphics.addBox(0.1, 0.1, 0.1, ColorDefinitions.Black());
      cartLink.addVisualDefinitions(cartGraphics.getVisualDefinitions());

      VisualDefinitionFactory poleGraphics = new VisualDefinitionFactory();
      poleGraphics.appendTranslation(0.0, 0.0, - POLE_COM_OFFSET);
      poleGraphics.addCylinder(POLE_COM_OFFSET * 2, 0.01, ColorDefinitions.Red());
      poleLink.addVisualDefinitions(poleGraphics.getVisualDefinitions());

      robotDefinition.setRootBodyDefinition(rootBodyDefinition);
      rootBodyDefinition.addChildJoint(cartJoint);
      cartJoint.setSuccessor(cartLink);
      cartLink.addChildJoint(poleJoint);
      poleJoint.setSuccessor(poleLink);

      robot = new Robot(robotDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);

      // from https://ocw.mit.edu/courses/6-832-underactuated-robotics-spring-2009/72bc06c4dc73315bf49c28a81dc2b996_MIT6_832s09_read_ch03.pdf
      A_lin = new DMatrixRMaj(4, 4);
      B_lin = new DMatrixRMaj(4, 1);

      A_lin.set(0, 2, 1.0);
      A_lin.set(1, 3, 1.0);
      A_lin.set(2, 1, POLE_MASS * g / CART_MASS);
      A_lin.set(3, 1, (POLE_MASS + CART_MASS) * g / (CART_MASS * POLE_COM_OFFSET));

      B_lin.set(2, 0, 1.0 / CART_MASS);
      B_lin.set(3, 0, 1.0 / (CART_MASS * POLE_COM_OFFSET));
   }

   public RobotDefinition getRobotDefinition()
   {
      return robotDefinition;
   }

   public Robot getRobot()
   {
      return robot;
   }


   public DMatrixRMaj getA_lin()
   {
      return A_lin;
   }

   public DMatrixRMaj getB_lin()
   {
      return B_lin;
   }

   public SimOneDoFJointBasics getPoleJoint()
   {
      return (SimOneDoFJointBasics) robot.getJoint(POLE_JOINT_NAME);
   }

   @Override
   public DMatrixRMaj computeH(DMatrixRMaj q)
   {
      double theta = q.get(1, 0);

      DMatrixRMaj H = new DMatrixRMaj(2, 2);

      H.set(0, 0, POLE_MASS + CART_MASS);
      H.set(0, 1, POLE_MASS * POLE_COM_OFFSET * Math.cos(theta));
      H.set(1, 0, POLE_MASS * POLE_COM_OFFSET * Math.cos(theta));
      H.set(1, 1, POLE_MASS * EuclidCoreTools.square(POLE_COM_OFFSET));

      return H;
   }

   @Override
   public DMatrixRMaj computeC(DMatrixRMaj q, DMatrixRMaj qd)
   {
      double theta = q.get(1, 0);
      double thetaDot = qd.get(1, 0);

      DMatrixRMaj C = new DMatrixRMaj(2, 2);
      C.set(0, 1, - POLE_MASS * POLE_COM_OFFSET * thetaDot * Math.sin(theta));

      return C;
   }

   @Override
   public DMatrixRMaj computeG(DMatrixRMaj q)
   {
      double theta = q.get(1, 0);
      DMatrixRMaj G = new DMatrixRMaj(2, 1);
      G.set(1, 0, POLE_MASS * g * POLE_COM_OFFSET * Math.sin(theta));

      return G;
   }

   @Override
   public DMatrixRMaj computeB()
   {
      DMatrixRMaj B = new DMatrixRMaj(2, 1);
      B.set(0, 0, 1.0);
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
      CartPoleRobot robot = new CartPoleRobot();

      RigidBodyBasics elevator = robot.getRobotDefinition().newInstance(ReferenceFrame.getWorldFrame());

      MultiBodySystemBasics multiBodySystemBasics = MultiBodySystemBasics.toMultiBodySystemBasics(elevator);
      PrismaticJoint cart = (PrismaticJoint) multiBodySystemBasics.findJoint(CART_JOINT_NAME);
      RevoluteJoint pole = (RevoluteJoint) multiBodySystemBasics.findJoint(POLE_JOINT_NAME);

      double q1 = 0.1;
      double q2 = -1.1;

      DMatrixRMaj q = new DMatrixRMaj(2, 1);
      q.set(0, 0, q1);
      q.set(1, 0, q2);

      CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(multiBodySystemBasics);
      cart.setQ(q1);
      pole.setQ(q2);
      DMatrixRMaj massMatrixMecano = massMatrixCalculator.getMassMatrix();

      DMatrixRMaj Hmanual = robot.computeH(q);
      System.out.println("H manual:\n" + Hmanual);

      System.out.println("H mecano:\n" + massMatrixMecano);
   }
}
