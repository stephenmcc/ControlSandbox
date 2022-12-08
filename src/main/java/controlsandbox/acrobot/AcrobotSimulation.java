package controlsandbox.acrobot;

import controlsandbox.lqr.LQRController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class AcrobotSimulation
{
   public AcrobotSimulation()
   {
      AcrobotRobot robot = new AcrobotRobot();
      robot.getShoulderJoint().setQ(Math.PI - 0.02);
      robot.getRobot().setController(LQRController.setupForAcrobot(robot));

      SimulationConstructionSet scs = new SimulationConstructionSet(robot.getRobot());

      scs.setGroundVisible(false);
      scs.setSimulateNoFasterThanRealTime(true);
      scs.setCameraPosition(6.0, 0.0, 0.0);
      scs.setCameraFix(-3.0, 0.0, 0.0);
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new AcrobotSimulation();
   }
}
