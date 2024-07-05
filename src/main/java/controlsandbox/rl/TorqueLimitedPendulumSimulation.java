package controlsandbox.rl;

import us.ihmc.scs2.SimulationConstructionSet2;

public class TorqueLimitedPendulumSimulation
{
   public TorqueLimitedPendulumSimulation()
   {
      SimulationConstructionSet2 scs = new SimulationConstructionSet2(getClass().getSimpleName());
   }

   public static void main(String[] args)
   {
      new TorqueLimitedPendulumSimulation();
   }
}
