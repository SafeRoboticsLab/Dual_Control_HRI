# Active Uncertainty Reduction for Human-Robot Interaction: An Implicit Dual Control Approach
<!-- Implementation of implicit dual control-based active uncertainty learning for human-robot interaction -->

[![License][license-shield]][license-url]
[![Homepage][homepage-shield]][homepage-url]


<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/SafeRoboticsLab/Dual_Control_HRI">
    <img src="Misc/stree.png" alt="Logo" >
  </a>

  <h3 align="center">Active Uncertainty Reduction for HRI</h3>

  <p align="center">
    Active Uncertainty Reduction for Human-Robot Interaction: An Implicit Dual Control Approach
    <!--
    <br />
    <a href="https://github.com/SafeRoboticsLab/Dual_Control_HRI"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/SafeRoboticsLab/Dual_Control_HRI">View Demo</a>
    ·
    <a href="https://github.com/SafeRoboticsLab/Dual_Control_HRI/issues">Report Bug</a>
    ·
    <a href="https://github.com/SafeRoboticsLab/Dual_Control_HRI/issues">Request Feature</a>
    -->
  </p>
</p>



<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li><a href="#about-the-project">About The Project</a></li>
    <li><a href="#dependencies">Dependencies</a></li>
    <li><a href="#example">Example</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#paper">Paper</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

We provide an MATLAB implementation of implicit dual control-based active uncertainty learning for autonomous driving applications, which can be found [here](https://github.com/SafeRoboticsLab/Dual_Control_HRI/tree/main).

Click to watch our spotlight video:
[![Watch the video](https://haiminhu.files.wordpress.com/2022/02/dual_control_hri_video_cover.png)](https://haiminhu.files.wordpress.com/2022/02/dual_control_hri.mp4)


## Dependencies

#### Trajectory Optimization
* [`MPT3`](https://www.mpt3.org/) (Toolbox for MPC and parametric optimization.)
* [`SNOPT`](https://ccom.ucsd.edu/~optimizers/solvers/snopt/) (Nonlinear programming solver. Academic licenses are available.)

#### Visualization
* [`Robotics Toolbox for MATLAB`](https://petercorke.com/toolboxes/robotics-toolbox/) (Tools for plotting the vehicles. You need to first install it in MATLAB and then replace `plot_vehicle.m` in the root directory with [ours](https://github.com/SafeRoboticsLab/Dual_Control_HRI/blob/main/ThirdParty/Robotics%20Toolbox%20for%20MATLAB/plot_vehicle.m).)

## Example
In this repository, we provide an example of our method applied for human-robot interactive driving scenarios.

1. Clone the repo
   ```sh
   git clone https://github.com/SafeRoboticsLab/Dual_Control_HRI.git
   ```
2. In MATLAB, run the [`main.m`](https://github.com/SafeRoboticsLab/Dual_Control_HRI/blob/main/main.m) script to reproduce our results.

<!-- USAGE EXAMPLES 
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

_For more examples, please refer to the [Documentation](https://example.com)_
-->


<!-- ROADMAP 
## Roadmap

See the [open issues](https://github.com/SafeRoboticsLab/SHARP/issues) for a list of proposed features (and known issues).
-->


<!-- CONTRIBUTING 
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request
-->


<!-- LICENSE -->
## License

Distributed under the BSD 3-Clause License. See `LICENSE` for more information.



<!-- CONTACT -->
## Contact

Haimin Hu - [@HaiminHu](https://twitter.com/HaiminHu) - haiminh@princeton.edu

Project Link: [https://github.com/SafeRoboticsLab/Dual_Control_HRI](https://github.com/SafeRoboticsLab/Dual_Control_HRI)

Homepage Link: [https://haiminhu.org/dual_control_hri](https://haiminhu.org/dual_control_hri)


<!-- PAPER -->
## Papers
### Journal paper:
The journal preprint is now available on arXiv: [https://arxiv.org/abs/2302.00171](https://arxiv.org/abs/2302.00171)
```tex
@article{hu2023active,
  title={Active Uncertainty Reduction for Safe and Efficient Interaction Planning: A Shielding-Aware Dual Control Approach},
  author={Hu, Haimin and Isele, David and Bae, Sangjae and Fisac, Jaime F},
  journal={arXiv preprint arXiv:2302.00171},
  year={2023}
}
```



### Conference paper/book chapter:
```tex
@inproceedings{hu2023active,
  title={Active Uncertainty Reduction for Human-Robot Interaction: An Implicit Dual Control Approach},
  author={Hu, Haimin and Fisac, Jaime F},
  booktitle={Algorithmic Foundations of Robotics XV},
  pages={385--401},
  year={2023},
  publisher={Springer International Publishing}
}
```

### Our prior work:

```tex
@article{hu2022sharp,
  author={Hu, Haimin and Nakamura, Kensuke and Fisac, Jaime F.},
  journal={IEEE Robotics and Automation Letters}, 
  title={SHARP: Shielding-Aware Robust Planning for Safe and Efficient Human-Robot Interaction}, 
  year={2022},
  volume={7},
  number={2},
  pages={5591-5598},
  doi={10.1109/LRA.2022.3155229}
}
```

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/SafeRoboticsLab/repo.svg?style=for-the-badge
[contributors-url]: https://github.com/SafeRoboticsLab/Dual_Control_HRI/contributors
[forks-shield]: https://img.shields.io/github/forks/SafeRoboticsLab/repo.svg?style=for-the-badge
[forks-url]: https://github.com/SafeRoboticsLab/Dual_Control_HRI/network/members
[stars-shield]: https://img.shields.io/github/stars/SafeRoboticsLab/repo.svg?style=for-the-badge
[stars-url]: https://github.com/SafeRoboticsLab/Dual_Control_HRI/stargazers
[issues-shield]: https://img.shields.io/github/issues/SafeRoboticsLab/repo.svg?style=for-the-badge
[issues-url]: https://github.com/SafeRoboticsLab/Dual_Control_HRI/issues
[license-shield]: https://img.shields.io/badge/License-BSD%203--Clause-blue.svg
[license-url]: https://opensource.org/licenses/BSD-3-Clause
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/SafeRoboticsLab
[homepage-shield]: https://img.shields.io/badge/-Homepage-brightgreen
[homepage-url]: https://haiminhu.org/dual_control_hri
