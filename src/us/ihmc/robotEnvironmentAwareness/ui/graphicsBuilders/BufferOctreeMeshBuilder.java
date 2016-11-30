package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;

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

   private final JavaFXMeshBuilder bufferMeshBuilder = new JavaFXMeshBuilder();
   private final Material bufferMaterial = new PhongMaterial(DEFAULT_BUFFER_COLOR);

   private BufferOctreeMeshBuilderListener listener;

   private final AtomicReference<UIOcTree> uiOctree = new AtomicReference<>(null);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> showBuffer;

   private boolean hasClearedBufferGraphics = false;

   public BufferOctreeMeshBuilder(REAMessager reaMessager, BufferOctreeMeshBuilderListener bufferOctreeMeshBuilderListener)
   {
      // local
      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      showBuffer = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsShowBuffer, true);

      setListener(bufferOctreeMeshBuilderListener);

      reaMessager.getPacketCommunicator().attachListener(NormalOcTreeMessage.class, normalOctreeMessagePacketConsumer);
   }

   public PacketConsumer<NormalOcTreeMessage> normalOctreeMessagePacketConsumer = (PacketConsumer<NormalOcTreeMessage>) packet ->
   {
      if (packet == null || packet.messageID != REAModuleAPI.BufferOctreeMessageID)
         return;

      uiOctree.set(new UIOcTree(packet));
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

         if (uiOctree.get() != null)
         {
            UIOcTree uiOcTree = uiOctree.getAndSet(null);

            for (UIOcTreeNode uiOcTreeNode : uiOcTree)
            {
               bufferMeshBuilder.addTetrahedron(NODE_SCALE * uiOcTreeNode.getSize(), new Point3d(uiOcTreeNode.getX(), uiOcTreeNode.getY(), uiOcTreeNode.getZ()));
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
