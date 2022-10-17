package controlsandbox.solver;

import controlsandbox.CartPoleRobot;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.matrixlib.MatrixTools;

public class DynamicSystemLinearization
{
   private final DynamicSystem system;
   private final DMatrixRMaj A;
   private final DMatrixRMaj B;

   public DynamicSystemLinearization(DynamicSystem system)
   {
      this.system = system;

      A = new DMatrixRMaj(2 * system.getPlantDegreesOfFreedom(), 2 * system.getPlantDegreesOfFreedom());
      B = new DMatrixRMaj(2 * system.getPlantDegreesOfFreedom(), system.getControlDegreesOfFreedom());
   }

   public void solve(DMatrixRMaj fixedPointQ, DMatrixRMaj fixedPointQd)
   {
      DMatrixRMaj H_inverse = new DMatrixRMaj(system.getPlantDegreesOfFreedom(), system.getPlantDegreesOfFreedom());
      DMatrixRMaj I = CommonOps_DDRM.identity(system.getPlantDegreesOfFreedom());
      DMatrixRMaj C = system.computeC(fixedPointQ, fixedPointQd);
      DMatrixRMaj G = system.computeG(fixedPointQ);
      DMatrixRMaj partialGpartialQ = new DMatrixRMaj(system.getPlantDegreesOfFreedom(), system.getPlantDegreesOfFreedom());

      CommonOps_DDRM.invert(system.computeH(fixedPointQ), H_inverse);

      for (int i = 0; i < system.getPlantDegreesOfFreedom(); i++)
      {
         DMatrixRMaj fixedPointQDelta = new DMatrixRMaj(fixedPointQ);
         double epsilon = 1e-8;
         fixedPointQDelta.add(i, 0, epsilon);
         DMatrixRMaj fixedPointDeltaG = system.computeG(fixedPointQDelta);

         DMatrixRMaj deltaG = new DMatrixRMaj(system.getPlantDegreesOfFreedom(), 1);
         CommonOps_DDRM.subtract(fixedPointDeltaG, G, deltaG);

         for (int j = 0; j < system.getPlantDegreesOfFreedom(); j++)
         {
            partialGpartialQ.set(j, i, deltaG.get(j, 0) / epsilon);
         }
      }

      MatrixTools.setMatrixBlock(A, 0, system.getPlantDegreesOfFreedom(), I, 0, 0, system.getPlantDegreesOfFreedom(), system.getPlantDegreesOfFreedom(), 1.0);

      DMatrixRMaj temp = new DMatrixRMaj(system.getPlantDegreesOfFreedom(), system.getPlantDegreesOfFreedom());
      CommonOps_DDRM.mult(-1.0, H_inverse, partialGpartialQ, temp);
      MatrixTools.setMatrixBlock(A, system.getPlantDegreesOfFreedom(), 0, temp, 0, 0, system.getPlantDegreesOfFreedom(), system.getPlantDegreesOfFreedom(), 1.0);

      CommonOps_DDRM.mult(-1.0, H_inverse, C, temp);
      MatrixTools.setMatrixBlock(A, system.getPlantDegreesOfFreedom(), system.getPlantDegreesOfFreedom(), temp, 0, 0, system.getPlantDegreesOfFreedom(), system.getPlantDegreesOfFreedom(), 1.0);

      temp = new DMatrixRMaj(system.getPlantDegreesOfFreedom(), system.getControlDegreesOfFreedom());
      DMatrixRMaj b = system.computeB();
      CommonOps_DDRM.mult(H_inverse, b, temp);
      MatrixTools.setMatrixBlock(B, system.getPlantDegreesOfFreedom(), 0, temp, 0, 0, temp.getNumRows(), temp.getNumCols(), 1.0);
   }

   public DMatrixRMaj getA()
   {
      return A;
   }

   public DMatrixRMaj getB()
   {
      return B;
   }

   public void print()
   {
      System.out.println("A:");
      System.out.println(A);

      System.out.println("\nB:");
      System.out.println(B);
   }

   public static void main(String[] args)
   {
      CartPoleRobot system = new CartPoleRobot();
      System.out.println("manual lin A");
      System.out.println(system.getA_lin());
      System.out.println("\nmanual lin B");
      System.out.println(system.getB_lin());

      DynamicSystemLinearization linearization = new DynamicSystemLinearization(system);

      DMatrixRMaj fixedPointQ = new DMatrixRMaj(2, 1);
      DMatrixRMaj fixedPointQd = new DMatrixRMaj(2, 1);
      fixedPointQ.set(1, 0, Math.PI);
      linearization.solve(fixedPointQ, fixedPointQd);

      linearization.print();
   }
}
