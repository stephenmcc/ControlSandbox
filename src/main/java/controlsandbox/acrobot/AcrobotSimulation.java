package controlsandbox.acrobot;

import controlsandbox.lqr.LQRController;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.Random;

import static controlsandbox.acrobot.AcrobotRobot.ELBOW_JOINT_NAME;
import static controlsandbox.acrobot.AcrobotRobot.SHOULDER_JOINT_NAME;

public class AcrobotSimulation
{
   public AcrobotSimulation()
   {
      AcrobotRobot acrobot = new AcrobotRobot();
      Random random = new Random(3290);

      PhysicsEngineFactory contactPhysicsEngine = SimulationConstructionSet2.contactPointBasedPhysicsEngineFactory();
      PhysicsEngineFactory impulsePhysicsEngine = SimulationConstructionSet2.impulseBasedPhysicsEngineFactory();

      SimulationConstructionSet2 scs2 = new SimulationConstructionSet2("AcrobotSimulation", contactPhysicsEngine);
      scs2.addRobot(acrobot.getRobot());

      acrobot.getShoulderJoint().setQ(Math.PI - 0.02);

      acrobot.getRobot().addController(LQRController.setupForAcrobot(acrobot));
      scs2.setDT(0.0001);
      scs2.setRealTimeRateSimulation(true);

      scs2.start(true, false, false);
   }

   public static void main(String[] args)
   {
      new AcrobotSimulation();
   }
}
