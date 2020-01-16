# Modified turtlebot3_fake simulation

The modifications add a publisher to the turtlebot3_fake.cpp simulation, which simply broadcasts a monotonically increasing count.

A separate listener (in listener.cpp) can be launched to subscribe and print the published messages.

This serves two purposes:
1. It provides a simple exercise for modifying existing turtlebot3 source files
2. It makes it easy to verify that the turtlebot3_fake simulation is running without a graphical interface
