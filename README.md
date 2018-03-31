#Project overview

##Goal
<p>This project was created in order to make motion profiling code that was simple to use for even rookie FRC teams. This motion profilling includes error correction and the ability to chain motions.</p>

##Included
<p>This project include two commands SimpleSplines and SimpleTurn wich combined gives a wide range of avaliable motions for non mechanum wheels.</p>
<table>
  <tr>
  <th>
    Command
  </th>
  <th>
    Description
  </th>
  </tr>
  <tr>
    <th>
      SimpleSimple
    </th>
    <th>
      Creates a Spline between n number of points. This path will go through all the points that it was given as parameter.
    </th>
  </tr>
  <tr>
    <th>
      PointTurn
    </th>
    <th>
      Given its start position and the requested end angle it will turn to that specific end angle.
    </th>
  </tr>
</table>

##How to use
###Using Jar
Copy 2974Splines.jar to "C:\Users\\{USER}\wpilib\user\java\lib\\"
If you don't do this, it will not build to the robot.
###SimpleSpline example
Going through 2 points facing forwards ending facing forwards.
`SimpleSpline.pathFromPosesWithAngle(false, new Pose(0,0, 90), new Pose(1,1, 90));`

Going through 3 points backwards with each point at different angles.
`SimpleSpline.pathFromPosesWithAngle(true, new Pose(2,0, 0), new Pose(-4,-2, 90), new Pose(-2, 3, -180));`
###SimpleTurn example
Turning from starting angle of 90 degrees to finishing angle of 180 degrees.
`SimpleTurn.pointTurn(new Pose(0,0, 90), new Pose(0,0, 180));`

this is the same as
`SimpleTurn.pointTurn(new Pose(0,0, 90), 180);`