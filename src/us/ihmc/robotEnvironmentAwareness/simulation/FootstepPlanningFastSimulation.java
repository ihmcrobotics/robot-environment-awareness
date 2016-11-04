package us.ihmc.robotEnvironmentAwareness.simulation;

import java.io.IOException;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.environment.FlatGroundEnvironment;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.util.RealtimeTools;

public class FootstepPlanningFastSimulation
{
   private enum GroundType
   {
      OBSTACLE_COURSE, FLAT
   }

   public static final int POINT_CLOUD_PUBLISHING_PERIOD_MILLSECONDS = 100;
   public static final double DEFAULT_SPIN_VELOCITY = 0.3;

   public FootstepPlanningFastSimulation() throws IOException
   {
      SimpleLidarRobot robot = new SimpleLidarRobot();
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, RealtimeTools.nextPowerOfTwo(200000));
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      double simDT = 0.0001;
      double controlDT = 0.01;
      scs.setDT(simDT, 10);

      Graphics3DAdapter graphics3dAdapter = scs.getGraphics3dAdapter();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.REA_MODULE_PORT, new IHMCCommunicationKryoNetClassList());
      packetCommunicator.connect();

      SimpleLidarRobotController controller = new SimpleLidarRobotController(robot, controlDT, packetCommunicator, graphics3dAdapter, yoGraphicsListRegistry);
      robot.setController(controller, (int) (controlDT / simDT));
//      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      createGroundTypeListener(scs);

      scs.setGroundVisible(false);
      scs.startOnAThread();
   }

   private void createGroundTypeListener(final SimulationConstructionSet scs)
   {
      final EnumYoVariable<GroundType> groundType = new EnumYoVariable<>("GroundType", scs.getRootRegistry(), GroundType.class, false);

      VariableChangedListener listener = new VariableChangedListener()
      {
         private Graphics3DNode groundGraphicsNode = null;

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (groundGraphicsNode != null)
               scs.removeGraphics3dNode(groundGraphicsNode);

            CommonAvatarEnvironmentInterface environment = null;

            switch (groundType.getEnumValue())
            {
            case OBSTACLE_COURSE:
               environment = new DefaultCommonAvatarEnvironment();
               break;
            case FLAT:
               environment = new FlatGroundEnvironment();
               break;
            }
            groundGraphicsNode = scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());
         }
      };
      groundType.addVariableChangedListener(listener);
      listener.variableChanged(null);
   }

   public static void main(String[] args) throws IOException
   {
      new FootstepPlanningFastSimulation();
   }
}
