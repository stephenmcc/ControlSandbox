package controlsandbox.lqr;

import org.ejml.data.DMatrixRMaj;

public interface DynamicSystem
{
   DMatrixRMaj computeH(DMatrixRMaj q);
   DMatrixRMaj computeC(DMatrixRMaj q, DMatrixRMaj qd);
   DMatrixRMaj computeG(DMatrixRMaj q);
   DMatrixRMaj computeB();

   int getPlantDegreesOfFreedom();
   int getControlDegreesOfFreedom();
}
