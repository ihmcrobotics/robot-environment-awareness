package us.ihmc.robotEnvironmentAwareness.simulation;

import java.io.IOException;
import java.util.EnumMap;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationKryoNetClassLists;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.util.RealtimeTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class LidarFastSimulation
{
   private enum GroundType
   {
      OBSTACLE_COURSE, FLAT, NOTHING
   }

   public static final int POINT_CLOUD_PUBLISHING_PERIOD_MILLSECONDS = 100;
   public static final double DEFAULT_SPIN_VELOCITY = 0.3;
   public static final boolean VISUALIZE_GPU_LIDAR = false;

   public LidarFastSimulation() throws IOException
   {
      SimpleLidarRobot robot = new SimpleLidarRobot();
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, RealtimeTools.nextPowerOfTwo(200000));
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      double simDT = 0.0001;
      double controlDT = 0.01;
      scs.setDT(simDT, 10);
      scs.setSimulateDoneCriterion(() -> false);

      Graphics3DAdapter graphics3dAdapter = scs.getGraphics3dAdapter();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.REA_MODULE_PORT, REACommunicationKryoNetClassLists.getPublicNetClassList());
      packetCommunicator.connect();

      SimpleLidarRobotController controller = new SimpleLidarRobotController(robot, controlDT, packetCommunicator, graphics3dAdapter, yoGraphicsListRegistry);
      robot.setController(controller, (int) (controlDT / simDT));
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      createGroundTypeListener(scs);

      scs.setGroundVisible(false);
      scs.startOnAThread();
      scs.simulate();
   }

   private void createGroundTypeListener(final SimulationConstructionSet scs)
   {
      final YoEnum<GroundType> groundType = new YoEnum<>("GroundType", scs.getRootRegistry(), GroundType.class, false);

      final EnumMap<GroundType, Graphics3DObject> environmentsGraphics = new EnumMap<>(GroundType.class);
      environmentsGraphics.put(GroundType.OBSTACLE_COURSE, new DefaultCommonAvatarEnvironment().getTerrainObject3D().getLinkGraphics());
      environmentsGraphics.put(GroundType.FLAT, new FlatGroundEnvironment().getTerrainObject3D().getLinkGraphics());
      environmentsGraphics.put(GroundType.NOTHING, new Graphics3DObject());

      VariableChangedListener listener = new VariableChangedListener()
      {
         private Graphics3DNode groundGraphicsNode = null;

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (groundGraphicsNode != null)
               scs.removeGraphics3dNode(groundGraphicsNode);
            groundGraphicsNode = scs.addStaticLinkGraphics(environmentsGraphics.get(groundType.getEnumValue()));
         }
      };
      groundType.addVariableChangedListener(listener);
      listener.variableChanged(null);
   }

   public static void main(String[] args) throws IOException
   {
      new LidarFastSimulation();
   }
}
