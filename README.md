<a name="readme-top"></a>

<!-- PROJECT LOGO -->
<br />
<div align="center">


  <h1 align="center">ROS 2 teleop and obstacle avoidance using Robot Car </h1>


</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary><h3>Table of Contents</h3></summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#demo">Demo</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#license">License</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

A four-wheeled robot was designed with **SolidWorks** and a **ROS 2** package was created in ```Python``` for Teleoperation and Navigation with obstacle avoidance.

Summary of tasks achieved: 
* Created robot model in ```SolidWorks``` and converted into ```urdf```.
* Attached ```lidar``` on the robot and deployed ```differential drive``` using gazebo plug-in.
* Programmed an ```Estimator``` class which converts ```quartenion``` from odometry topic to ```euler``` angles. 
* And a ```Controller``` class uses these values from estimator for Obstacle Avoidance.



<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Demo

<div align="center">


  <h4 align="center">Robot navigation using simple obstacle avoidance (X8 Speed)</h4>


</div>

https://user-images.githubusercontent.com/90359587/224219794-b9494aa1-e709-4372-9e1f-9eed5431d9a3.mp4

<div align="center">


  <h4 align="center">Teleop result (X4 Speed)</h4>


</div>

https://user-images.githubusercontent.com/90359587/224219957-2b29a8ae-3ab5-4141-9508-41b424d332fa.mp4
<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

These are the instructions to get started on the project.
To get a local copy up and running follow these simple steps.

### Prerequisites
* Python 3.6 +
* ROS 2 Turtlebot3 teleop should be installed.
* ROS 2 : foxy (tested)
* OS - Linux (tested)


### Installation

1. Copy the package ```kick_ass``` inside the ```src``` folder of catkin workspace ```catkin_ws```.

2. Make the following changes inside the ```kick_ass.urdf``` file located at ```catkin_ws/src/kick_ass/urdf```.
    * Replace all ''**/home/jeffin/check_r2/src**'' inside the ```filename``` attribute of ```mesh``` tag to the absolute path of the current package. 
    * So if ```catkin_ws``` is in home directory, it should be ''**/home/username/catkin_ws/src**''
3. Navigate to ```src``` and build
   ```sh
   colcon build
   ```
4. Source the workspace
   ```sh
   source ~/catkin_ws/devel/setup.bash
   ```


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

### To run teleop

1. Run the command :
   ```sh
   ros2 launch kick_ass launch_urdf_into_gazebo.launch.py 
   ```
2. Open new terminal ,source workspace and run:
   ```sh
   ros2 run turtlebot3_teleop teleop_keyboard 
   ```
```NOTE``` : If turtlebot3 teleop not available, try:
   ```sh
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

### To run Robot Navigation node with Obstacle Avoidance

1. Source the workspace.
2. Start simulation:
   ```sh
   ros2 launch kick_ass launch_urdf_into_gazebo.launch.py
   ```
3. Open new terminal ,source workspace and run:
   ```sh
   ros2 launch kick_ass robot_move.launch.py
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Jeffin Johny K - [![MAIL](https://img.shields.io/badge/Gmail-D14836?style=for-the-badge&logo=gmail&logoColor=white)](mailto:jeffinjk@umd.edu)
	
[![portfolio](https://img.shields.io/badge/my_portfolio-000?style=for-the-badge&logo=ko-fi&logoColor=white)](https://kachappilly2021.github.io/)
[![linkedin](https://img.shields.io/badge/linkedin-0A66C2?style=for-the-badge&logo=linkedin&logoColor=white)](http://www.linkedin.com/in/jeffin-johny-kachappilly-0a8597136)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See [MIT](https://choosealicense.com/licenses/mit/) for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=for-the-badge
[contributors-url]: https://github.com/othneildrew/Best-README-Template/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=for-the-badge
[forks-url]: https://github.com/othneildrew/Best-README-Template/network/members
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge
[stars-url]: https://github.com/othneildrew/Best-README-Template/stargazers
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-url]: https://github.com/othneildrew/Best-README-Template/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge
[license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/othneildrew
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com 
