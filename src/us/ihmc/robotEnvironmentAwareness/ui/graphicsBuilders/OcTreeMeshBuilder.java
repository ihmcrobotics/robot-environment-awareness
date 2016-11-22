package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.javaFXToolkit.shapes.MeshBuilder;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OctreeNodeData;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OctreeNodeMessage;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Created by adrien on 11/21/16.
 */
public class OcTreeMeshBuilder implements Runnable
{

   public interface OctreeMeshBuilderListener
   {
      void meshAndMaterialChanged(Pair<Mesh, Material> meshMaterial);
   }


   private final AtomicReference<ArrayList<OctreeNodeData>> octreeNodeDataList = new AtomicReference<>(null);

   private OctreeMeshBuilderListener listener;
   private boolean hasClearedBufferGraphics = false;


   public OcTreeMeshBuilder(REAMessager reaMessager, OctreeMeshBuilderListener octreeMeshBuilderListener)
   {
      // local
//      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, false);
//      showBuffer = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsShowBuffer, true);

      setListener(octreeMeshBuilderListener);

      reaMessager.getPacketCommunicator().attachListener(OctreeNodeMessage.class, octreeNodeMessagePacketConsumer);
   }

   public PacketConsumer<OctreeNodeMessage> octreeNodeMessagePacketConsumer = (PacketConsumer<OctreeNodeMessage>) packet ->
   {
      if (packet == null || packet.getMessageID() != REAModuleAPI.OctreeMessageID)
         return;

      octreeNodeDataList.set(packet.getOctreeNodeDataList());
   };




   private void setListener(OctreeMeshBuilderListener listener)
   {
      if (listener == null)
         throw new IllegalArgumentException("OctreeMeshBuilderListener cannot be null");
      this.listener = listener;
   }

   @Override public void run()
   {


   }


}
