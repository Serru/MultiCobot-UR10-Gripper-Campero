<p align="center">

<img alt="MultiCobot-UR10-Gripper" style="border-width:0" src="https://raw.githubusercontent.com/Serru/MultiCobot-UR10-Gripper/main/doc/imgs_md/proyect-logo.png" />
</p>

# MultiCobot-UR10-Gripper-Campero
<p align="center">
Multi-robot system of collaborative robots (cobots) <a rel="UR10s" href="https://www.universal-robots.com/products/ur10-robot/">UR10s</a> with grippers from Robotiq (<a rel="robotiq_85_gripper" href="https://robotiq.com/products/2f85-140-adaptive-robot-gripper">robotiq_85_gripper</a>) that allows simultaneous execution of tasks with different types of controllers and brands of cobots, as well as direct control of the cobot by a person via the Leap Motion device.
</p>

[Español](https://github.com/Serru/MultiCobot-UR10-Gripper-Campero/blob/main/README_ESP.md) | **English**

## About this project
This thesis project focuses on developing a multi-robot system that can cooperatively perform tasks such as transporting objects. There is not much documentation on how to develop a system where multiple robots can be controlled simultaneously in the environment of ROS, which is widely used in research and prototyped for testing before going into production.

Two solutions have been designed, developed, implemented and experimentally evaluated: the first is the *ROS* package called `MoveIt!`, where the work is mainly focused on the configuration to allow the simultaneous control of different cobots; the second is the creation or use of a third-party planner that sends the commands directly to the controllers in charge of executing the movements of the cobots, and each of them has a built-in gripper that allows them to perform different tasks.

The Leap Motion device is also integrated into the system. It is capable of detecting, tracking and recognizing the user's hand gestures and serves as an interface for the simultaneous control of up to two cobots, enabling the manipulation of objects.

[Read more...](https://deposita.unizar.es/record/66296?ln=es)

## System requirements
- Ubuntu 16.04
- Python 2.7
- ROS Kinetic Kame

## Documentation
- [System Configuration in Gazebo Simulator](https://github.com/Serru/MultiCobot-UR10-Gripper#readme)

### Configuration on the physical robot

![image](https://raw.githubusercontent.com/Serru/MultiCobot-UR10-Gripper/main/doc/imgs_md/dise%C3%B1o-general-campero.png)

Since the internal configuration of the Campero robot system may vary depending on the company that configured it. We will not go into detail on how the system configuration was done. The code serves as a reference that you can modify or take from the implemented code.

## Video with the results
Here is a video with the results of the simulations performed in `Gazebo`. The video shows two and four robots performing a *pick & place* without human intervention. Then it shows how a person controls two cobots with Leap Motion, and finally the developed result was tested with the physical Campero robot.

<p>
<a href="https://drive.google.com/file/d/1oqVyre4vlfHqH9SrQuyXH00GcmwIuP97/view?usp=sharing" title="Link Title">
  <img src="https://raw.githubusercontent.com/Serru/MultiCobot-UR10-Gripper/main/doc/imgs_md/img-fondo-video.png" alt="Results of the project." />
</a>
</p>


## Help & Support
Unfortunately, this repository is not actively maintained. The main goal is to publish what has been learned for the community who might need the knowledge and content of this repository to develop their project or research.

A response is not guaranteed, but you can contact the authors using the information in the [Authors](#autores) section.

## License

<p align="left">
  <a href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/LICENSE">
    <img src="https://licensebuttons.net/l/by/4.0/88x31.png" alt="This repository is published under the Creative Commons Attribution 4.0 International license." />
  </a>
  </br>
  </br>
This repository is published under the <a href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/LICENSE">Creative Commons Attribution 4.0 International</a> license.
</p>

## Authors
- [Burgh Oliván, Miguel](https://github.com/Serru) - *Author of the Final Degree Project entitled **Multirobot system for the collaborative transport of objects**.*
- [López Nicolás, Gonzalo](https://i3a.unizar.es/es/investigadores/gonzalo-lopez-nicolas) - *Director of the Final Degree Project entitled **Multirobot system for the collaborative transport of objects**.*

The memory of the Final Degree Project can be found in the [Repository](https://deposita.unizar.es/record/66296?ln=es) of TFGs of the [University of Zaragoza](http://www.unizar.es/).

## Acknowledgement

This work is part of the [RoPeRT](https://i3a.unizar.es/es/grupos-de-investigacion/ropert) research group of the [i3A](https://i3a.unizar.es), the [University of Zaragoza](http://www.unizar.es/).

![image](https://www.unizar.es/sites/default/files/i3a.png)

---

The developed work has been experimentally evaluated and validated, showing a correct operation of the physical robot [Campero](http://commandia.unizar.es/wp-content/uploads/camperoRobot.jpg). For this reason, this work is part of the activities of the project [COMMANDIA (2019)](http://commandia.unizar.es/), co-funded by the [Interreg Sudoe Program](https://www.interreg-sudoe.eu/inicio) and the [European Regional Development Fund (ERDF)](https://ec.europa.eu/regional_policy/es/funding/erdf/).

![image](http://commandia.unizar.es/wp-content/uploads/cropped-logoCommandia-1.png)

## Recognition

Please cite this work if the content of this repository has been useful to you:

BibTeX: 
```
@article{
    BurghOliván:66296,
    author = "Burgh Oliván, Miguel Yankan and López Nicolás, Gonzalo",
    title = "{Sistema multirobot para el transporte colaborativo de objetos}",
    year  = "2022",
}
```
