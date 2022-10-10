package controlsandbox;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class CartPoleSimulation
{
   public CartPoleSimulation()
   {
      CartPoleRobot robot = new CartPoleRobot();
      robot.getPinJoint().setQd(0.1);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      scs.setSimulateNoFasterThanRealTime(true);
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new CartPoleSimulation();
   }
}
