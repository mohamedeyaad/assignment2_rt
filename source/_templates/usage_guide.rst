Usage Guide
===========

Running the Action Client
-------------------------
To run the action client, use the following command:

.. code-block:: bash

    rosrun assignment2_rt action_client.py

Setting a Goal
--------------
Use the GUI or terminal to set a goal for the robot. For example:

.. code-block:: python

    send_goal(client, x=2.0, y=3.0)

Monitoring Robot State
-----------------------
The robot's state is published to the `/robot_state` topic. Use the following command to monitor it:

.. code-block:: bash

    rostopic echo /robot_state