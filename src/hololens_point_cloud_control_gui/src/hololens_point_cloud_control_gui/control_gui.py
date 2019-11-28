import os
import rospy
import rospkg

from os.path import expanduser

from std_msgs.msg import Bool
from std_msgs.msg import String

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

class ControlGuiPlugin(Plugin):
    currentUi = 0

    clearPublisher = rospy.Publisher('/clearPointCloud', Bool, queue_size=10)
    savePublisher = rospy.Publisher('/savePointCloud', Bool, queue_size=10)
    recordPublisher = rospy.Publisher('/recordDepthFrames', Bool, queue_size=10)
    loadPublisher = rospy.Publisher('/loadRecording', String, queue_size=10)

    def __init__(self, context):
        super(ControlGuiPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ControlGuiPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('hololens_point_cloud_control_gui'), 'resource', 'ControlGui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ControlGuiPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Register the callbacks.
        self._widget.clear_button.clicked.connect(self.clear_button_clicked)
        self._widget.save_button.clicked.connect(self.save_button_clicked)
        self._widget.record_button.clicked.connect(self.record_button_clicked)
        self._widget.load_button.clicked.connect(self.load_button_clicked)

        # Update and show the UI elements.
        self._widget.record_button.setCheckable(True)
        self._widget.record_button.setStyleSheet('QPushButton:checked{background-color:red}\nQPushButton:focus{border-style:groove;border-width:1px;border-radius:1px;border-color:silver;}')
        self._widget.recording_directory_input.setPlainText(expanduser("~/recordings/"))
        self.updateUi()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.clearPublisher.unregister()
        self.savePublisher.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        instance_settings.set_value('currentUi', self.currentUi)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        self.currentUi = instance_settings.value('currentUi')
        if self.currentUi == None:
            self.currentUi = 0
        self.updateUi()
        pass

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # # Usually used to open a modal configuration dialog
        if self.currentUi == None:
            self.currentUi = 0
        self.currentUi = (self.currentUi + 1) % 3
        rospy.loginfo('Current UI: ' + str(self.currentUi))
        self.updateUi()
        pass       

    def updateUi(self):
        if self.currentUi == 0:
            self.updateToRealtimeSpatialMapperUi()
        elif self.currentUi == 1:
            self.updateToDepthFrameRecorderUi()
        elif self.currentUi == 2:
            self.updateToRecordedSpatialMapperUi()

    def updateToRealtimeSpatialMapperUi(self):
        self._widget.clear_button.show()
        self._widget.save_button.show()
        self._widget.record_button.hide()
        self._widget.recording_directory_input.hide()
        self._widget.load_button.hide()
    
    def updateToDepthFrameRecorderUi(self):
        self._widget.clear_button.hide()
        self._widget.save_button.hide()
        self._widget.record_button.show()
        self._widget.recording_directory_input.hide()
        self._widget.load_button.hide()

    def updateToRecordedSpatialMapperUi(self):
        self._widget.clear_button.show()
        self._widget.save_button.show()
        self._widget.record_button.hide()
        self._widget.recording_directory_input.show()
        self._widget.load_button.show()

    def clear_button_clicked(self, checked):
        rospy.loginfo('Clear button clicked.')
        self.clearPublisher.publish(True)

    def save_button_clicked(self, checked):
        rospy.loginfo('Save button clicked.')
        self.savePublisher.publish(True)
    
    def record_button_clicked(self, checked):
        rospy.loginfo('Record button clicked: ' + str(checked))
        self.recordPublisher.publish(checked)
    
    def load_button_clicked(self, checked):
        recordingDirectory = self._widget.recording_directory_input.toPlainText()
        rospy.loginfo('Load button clicked: ' + recordingDirectory)
        self.loadPublisher.publish(recordingDirectory)
