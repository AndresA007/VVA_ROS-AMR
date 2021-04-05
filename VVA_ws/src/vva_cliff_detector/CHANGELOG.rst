^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vva_cliff_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* First commit
* Contributors: Michal Drwiega


* vva_cliff_detector: based on cliff_detector.
* Original cliff_detector requires a node to be subscribed to the topic "cliff_detector/depth/image"
  to publish the topic "cliff_detector/points", this modification makes if publish the points even
  when there is no subscriber to the depth image.

