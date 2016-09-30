package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.property.SimpleBooleanProperty;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessager;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleAPI;

public class OcTreeBasicsAnchorPaneController
{
   @FXML
   private ToggleButton enableButton;
   @FXML
   private Button clearButton;
   private REAMessager outputMessager;

   public OcTreeBasicsAnchorPaneController()
   {
   }

   public void attachOutputMessager(REAMessager outputMessager)
   {
      this.outputMessager = outputMessager;
   }

   public void bindControls()
   {
      new SimpleBooleanProperty(false)
      {
         @Override
         protected void invalidated()
         {
            outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeEnable, get()));
         }
      }.bind(enableButton.selectedProperty());
   }

   @FXML
   public void clear()
   {
      outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeClear, true));
   }
}
