package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.javaFXToolkit.shapes.MeshBuilder;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OctreeNodeData;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OctreeNodeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.REAMessagePacket;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Created by adrien on 11/20/16.
 */
public class BufferOctreeMeshBuilder implements Runnable
{

   public interface BufferOctreeMeshBuilderListener
   {
      void meshAndMaterialChanged(Pair<Mesh, Material> meshMaterial);
   }

   private static final Color DEFAULT_BUFFER_COLOR = Color.RED;
   private static final double NODE_SCALE = 0.5;

   private final MeshBuilder bufferMeshBuilder = new MeshBuilder();
   private final Material bufferMaterial = new PhongMaterial(DEFAULT_BUFFER_COLOR);

   private BufferOctreeMeshBuilderListener listener;

   private final AtomicReference<ArrayList<OctreeNodeData>> octreeNodeDataList = new AtomicReference<>(null);
   private final AtomicReference<Float> defaultNodeSize = new AtomicReference<>(null);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> showBuffer;

   private boolean hasClearedBufferGraphics = false;

   public BufferOctreeMeshBuilder(REAMessager reaMessager, BufferOctreeMeshBuilderListener bufferOctreeMeshBuilderListener)
   {
      // local
      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      showBuffer = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsShowBuffer, true);

      setListener(bufferOctreeMeshBuilderListener);

      reaMessager.getPacketCommunicator().attachListener(OctreeNodeMessage.class, octreeNodeMessagePacketConsumer);
   }

   public PacketConsumer<OctreeNodeMessage> octreeNodeMessagePacketConsumer = (PacketConsumer<OctreeNodeMessage>) packet ->
   {
      if (packet == null || packet.getMessageID() != REAModuleAPI.BufferOctreeMessageID)
         return;

      octreeNodeDataList.set(packet.getOctreeNodeDataList());
      defaultNodeSize.set(packet.getDefaultNodeSize());

   };

   private void setListener(BufferOctreeMeshBuilderListener listener)
   {
      if (listener == null)
         throw new IllegalArgumentException("BufferOctreeMeshBuilderListener cannot be null");
      this.listener = listener;
   }

   @Override public void run()
   {
      if (!enable.get() || !showBuffer.get())
      {
         clearBufferGraphicsIfNeeded();
      }
      else
      {

         bufferMeshBuilder.clear();

         if (octreeNodeDataList.get() != null)
         {
            ArrayList<OctreeNodeData> data = octreeNodeDataList.getAndSet(null);
            float size = defaultNodeSize.getAndSet(null);
            for (OctreeNodeData octreeNodeData : data)
            {
               NormalOcTreeNode normalOcTreeNode = octreeNodeData.getNormalOcTreeNode();
               bufferMeshBuilder.addCube(NODE_SCALE * size, normalOcTreeNode.getHitLocationX(), normalOcTreeNode.getHitLocationY(), normalOcTreeNode.getHitLocationZ());
            }

            Pair<Mesh, Material> meshAndMaterial = new Pair<>(bufferMeshBuilder.generateMesh(), bufferMaterial);
            listener.meshAndMaterialChanged(meshAndMaterial);

            hasClearedBufferGraphics = false;
         }
      }
   }

   private void clearBufferGraphicsIfNeeded()
   {
      if (hasClearedBufferGraphics)
         return;
      bufferMeshBuilder.clear();
      Pair<Mesh, Material> meshAndMaterial = new Pair<>(bufferMeshBuilder.generateMesh(), bufferMaterial);
      listener.meshAndMaterialChanged(meshAndMaterial);

      hasClearedBufferGraphics = true;
   }

}
