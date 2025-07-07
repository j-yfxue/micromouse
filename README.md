Introduction:

This project is a senior design under the Department of Electrical and Computer Engineering at the University of New Hampshire. The goal is to build a prototype to help start a student organization that can compete in Micromouse competitions. 

In a typical Micromouse competition, the maze has a designated center cell as the destination. The robot must first explore the maze to find a valid path, then return to the start and solve the maze in the shortest time possible.

Detailed Rules of the competition can be found at: https://attend.ieee.org/r2sac-2020/wp-content/uploads/sites/175/2020/01/MicroMouse_Rules_2020.pdf

----------------------------------------------------------------------------------------------------------------------------------------------------



Demo: 

  This code was tested in a 3x3 maze with each of the blocks being 26cm x 26cm

  Link to Demo:
  https://www.youtube.com/watch?v=hlFdUKvN9es

  The robot is limited by the IR sensors. The original plan was to use three IR sensors to detect the presence of walls. However, due to the limited detection range of the sensors we used, the side   sensors failed to function most of the time. They are disabled when the demo was done. As a result, only the front IR sensor was used in the demo, though it still occasionally failed.


----------------------------------------------------------------------------------------------------------------------------------------------------


Componenets:

  2WD miniQ Robot Chassis https://www.dfrobot.com/product-367.html
	
  NUCLEO-F446RE 

  50:1 gear ratio motor with encoder https://www.dfrobot.com/product-1432.html

  L298N

  LSM6DSOX Gyro and accelerometer  https://www.adafruit.com/product/4438?srsltid=AfmBOopEDnr-NoMDJTALgeg2A_9HXfCZseXL8RcEJNjc3BZC9T981ppC

  IR sensor


----------------------------------------------------------------------------------------------------------------------------------------------------




Reference:

  The search alogorithm was tested using this simulator: https://github.com/mackorone/mms?tab=readme-ov-file


----------------------------------------------------------------------------------------------------------------------------------------------------


Acknowledgements:

  Developed by Christina Karagianis, Abby Mathieu, and Yifeng Xue (https://github.com/j-yfxue)

  Advisors: Professor Se Young Yoon and Professor MD Shaad Mahmud

  Senior Design Report: https://docs.google.com/document/d/1eB9g1LCVUFNQIfvklPDIRrS_rpDlfjf1XYIzxSF4wa8/edit?usp=sharing

  
