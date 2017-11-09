UWB Ultra Wide Band
===

There are 2 models of tags v1 and v2.


for tag v1 you can connect 1-3 anchors and it will work, NOTE: you need to send an "a" char to begin readings and "b" char to stop them. If you don't connect all anchors

you get 0.0 distance for the not detected anchors


for tag v2 you MUST connect the 3 anchors to get readings, otherwise it will not work!, NOTE: you don't need to send any chars trough serial port

readings come automatically and you cannot stop them by sending "b" char.

A nice visualization is available (check uwb_visualization.png file on this folder) which publishes the readings as markers.

This visualizationis included in the sample launch file, however you have to add a marker topic in rviz to see them and the frame_id of the uwb needs to exist

or be set as fixed frame.


Bill of materials
===

- 2 Mini USB cables

- 2 Kio tags (version 2)

- 3 Kio anchors (version 2)

- 3 anchor 5V power supply

- 3 Micro USB cables

I _______________________ confirm the reception of all the materials above listed under "bill of materials" section for

the Autonomous Systems as part of my project inside Instituto Superior Tecnico Lisboa. I make myself responsible for

this material and promise to return in good conditions.

|

|

|

|

-------------------

Signature and date
