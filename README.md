<p align="center">
  <h1 align="center">Motion Matching for Responsive Animation for Digital Humans</h1>
  <p align="center">
    <a href="https://github.com/DecAd3"><strong>Longteng Duan*</strong></a>
    ·
    <a href="https://github.com/guo-han"><strong>Guo Han*</strong></a>
    ·
    <a href="https://github.com/Ribosome-rbx"><strong>Boxiang Rong*</strong></a>
    ·
    <a href="https://github.com/Milkiananas"><strong>Hang Yin*</strong></a>
  </p>
  <p align="center"><strong>(* Equal Contribution)</strong></p>
  <h3 align="center"><a href="https://www.youtube.com/watch?v=QZjcfzG_R1k&list=PLUffCQyBEYtYXr-pVqqUgSG1Ncxp4UzAb">Demo Videos</a> | <a href="">Report</a> | <a href="https://docs.google.com/presentation/d/13Kz_PvJAkfzi9m_gFjUCRpPG92N0RBROceEzudvjc8I/edit?usp=sharing">Slides</a></h3>
  <div align="center"></div>
</p>
<p align="center">
    <iframe width="560" height="315" src="https://www.youtube.com/embed/1ZTEGm7lSjg" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</p>
<br>

<details open="open" style='padding: 10px; border-radius:5px 30px 30px 5px; border-style: solid; border-width: 1px;'>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#compilation">Compilation</a>
    </li>
    <li>
      <a href="#dataset">Dataset</a>
    </li>
    <li>
      <a href="#functionalities">Functionalities</a>
    </li>
    <li>
      <a href="#acknowledgement">Acknowledgement</a>
    </li>
  </ol>
</details>

## Compilation
The program is easy to compile using Visual Studio Code and CMake.

```
git clone https://github.com/Ribosome-rbx/motion-matching-demo.git
cd motion-matching-demo
mkdir build
cd build
cmake ..
make
```
## Dataset
The dataset we used for constructing matching database is [LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset). But you do not need to redownload and process the dataset yourself. All necessary data have been added into this directory under the `/data` folder.
## Functionalities
Users can control the digital character using keyboard control, drawed trajectory or human pose input from camera. Details are elaborated below.
### Keyboard Control
|Keys|Actions|
|:-:|:-:|
|W, S, A, D|controll the character's movement in different directions|
|Shift|sprint|
|J|jump|
|C|crawl|
|P|dance|

### Drawed Trajectory
[to be finished]

<details>
  <summary>[Video demo for drawed trajectory (click to expand)]</summary>
  
  <iframe width="560" height="315" src="https://www.youtube.com/embed/mbaMwk-pqMA" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

</details>

### Human Pose Control
[to be finished]
<details>
  <summary>[Video demo for human pose control (click to expand)]</summary>
  
  <iframe width="560" height="315" src="https://www.youtube.com/embed/55wjVJXs2dg" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

</details>

### External Terrain Object Loading
[to be finished]
## Acknowledgement
This project was undertaken as part of the 2023HS digital human course at ETH Zurich. We would like to express our sincere appreciation for the valuable guidance and support provided by our supervisor, [Dongho Kang](https://donghok.me/).