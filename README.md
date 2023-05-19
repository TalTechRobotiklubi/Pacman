# Pacman

Repository for the project Pacman game.

## Project members
* Erki Meinberg - project lead
* Märtin Laanemets - electronics engineer for the new PisiBot controller module
  Pisi-Xbee 6
* Oliver Paljak - developer of the Pacman game (this repository)

## Building the project

1. Make sure you have necessary dependencies installed (OpenCV, SDL, Boost, and
   XIMEA)
2. Make dictionary `build` and run the command `cmake ..`
3. After that run the command `make`
4. If everything compiles, you are good to go

## Demos

Robot with the (ArUco) ID 1 is the robot that is controlled by a human player.
The robots (with IDs 10 and 12) are the "ghosts" that are controlled by the 
computer to try to catch the human controlled robot.

If the ghosts get too close to the human controlled robot, then the game is
over and can be reset so that all the robots will return to their starting
positions.

See more demos in the demo_videos folder.

https://github.com/TalTechRobotiklubi/Pacman/assets/15703089/fb9042ce-8f58-4ab8-87f1-cb3239f578cf

https://github.com/TalTechRobotiklubi/Pacman/assets/15703089/59f8d5a0-dad4-4008-93d9-1f681f51b356
