package controlsandbox;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.robotics.screwTheory.MassMatrixCalculator;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;

public class DynamicMatrix
{
   public DynamicMatrix()
   {
      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
      RigidBodyDefinition pelvis = new RigidBodyDefinition("pelvis");

      RigidBodyDefinition r1 = new RigidBodyDefinition("r1");
      RigidBodyDefinition r2 = new RigidBodyDefinition("r2");
      RigidBodyDefinition r3 = new RigidBodyDefinition("r3");

      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition("rootJoint");
      RevoluteJointDefinition j1 = new RevoluteJointDefinition("j1", new Vector3D(0.0, 0.0, -0.2), Axis3D.Z);
      RevoluteJointDefinition j2 = new RevoluteJointDefinition("j2", new Vector3D(0.0, 0.0, -0.2), Axis3D.X);
      RevoluteJointDefinition j3 = new RevoluteJointDefinition("j3", new Vector3D(0.0, 0.0, -0.2), Axis3D.Y);

      pelvis.setMass(100.0);
      pelvis.setMomentOfInertia(new MomentOfInertiaDefinition(3.0, 3.0, 3.0));

      r1.setMass(0.2);
      r1.setMomentOfInertia(new MomentOfInertiaDefinition(0.01, 0.01, 0.01));
      r2.setMass(0.2);
      r2.setMomentOfInertia(new MomentOfInertiaDefinition(0.01, 0.01, 0.01));
      r3.setMass(0.2);
      r3.setMomentOfInertia(new MomentOfInertiaDefinition(0.01, 0.01, 0.01));

      RobotDefinition robotDefinition = new RobotDefinition("test");
      robotDefinition.setRootBodyDefinition(rootBody);
      rootBody.addChildJoint(rootJoint);
      rootJoint.setSuccessor(pelvis);
      pelvis.addChildJoint(j1);
      j1.setSuccessor(r1);
      r1.addChildJoint(j2);
      j2.setSuccessor(r2);
      r2.addChildJoint(j3);
      j3.setSuccessor(r3);

      RigidBodyBasics rootBodyID = robotDefinition.newInstance(ReferenceFrame.getWorldFrame());
      MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(rootBodyID);

      FloatingJointBasics rootJointID = (FloatingJointBasics) system.findJoint("rootJoint");
      RevoluteJoint j1ID = (RevoluteJoint) system.findJoint("j1");
      RevoluteJoint j2ID = (RevoluteJoint) system.findJoint("j2");
      RevoluteJoint j3ID = (RevoluteJoint) system.findJoint("j3");

      rootJointID.getJointPose().set(0.1, 0.2, -0.2, 0.4, -0.9, 1.0);
      rootJointID.setJointLinearVelocity(new Vector3D(1.0, 0.5, -0.3));
      rootJointID.setJointAngularVelocity(new Vector3D(0.3, 1.0, -0.3));

      j1ID.setQ(0.3);
      j1ID.setQd(-0.4);

      j2ID.setQ(-0.2);
      j2ID.setQd(-0.4);

      j3ID.setQ(0.6);
      j3ID.setQd(0.8);

      rootBodyID.updateFramesRecursively();

      CompositeRigidBodyMassMatrixCalculator matrixCalculator = new CompositeRigidBodyMassMatrixCalculator(rootBodyID);
      matrixCalculator.setEnableCoriolisMatrixCalculation(true);
      DMatrixRMaj massMatrix = matrixCalculator.getMassMatrix();
      System.out.println("mass matrix");
      System.out.println(massMatrix);

      GravityCoriolisExternalWrenchMatrixCalculator gravityCoriolisExternalWrenchMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBodyID), true, true);
      gravityCoriolisExternalWrenchMatrixCalculator.setGravitionalAcceleration(-9.81);
      gravityCoriolisExternalWrenchMatrixCalculator.compute();
      DMatrixRMaj cqd_g = gravityCoriolisExternalWrenchMatrixCalculator.getJointTauMatrix();
      System.out.println("coriolis and gravity matrices");
      System.out.println(cqd_g);
   }

   public static void main(String[] args)
   {
      new DynamicMatrix();
   }
}
