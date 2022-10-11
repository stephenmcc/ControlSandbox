package controlsandbox;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class CartPoleSimulation
{
   public CartPoleSimulation()
   {
      CartPoleRobot robot = new CartPoleRobot();
      robot.getSCSPoleJoint().setQ(Math.PI);
      robot.getRobot().setController(new CartPoleLQRController(robot));

      SimulationConstructionSet scs = new SimulationConstructionSet(robot.getRobot());

      scs.setSimulateNoFasterThanRealTime(true);
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new CartPoleSimulation();
   }
}
