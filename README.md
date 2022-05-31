# Avidrone Avalanche Rescue 🏔⛑

<!-- BADGE:START - Do not remove or modify this section -->
[![All Contributors](https://img.shields.io/badge/all_contributors-4-orange.svg?style=flat-square)](#contributors-)
![license](https://img.shields.io/github/license/AviDrone/AviDrone)
![stars](https://img.shields.io/github/stars/AviDrone/AviDrone?style=social)
<!-- BADGE:END -->

Avalanches kill over 150 people worldwide every year.[^1] After getting caught in an avalanche, your chances of survival in the first 15 minutes are around 90%. After 30 minutes, your chances drop to 30%. Rescue teams must get to avalanche victims in the fastest time possible.
[^1]: [National Geographic](https://www.nationalgeographic.org/encyclopedia/avalanche/)

## Objective

Our objective for this project is to use UAV systems to search for avalanche transceiver signals. This allows us to

- Search over unsafe avalanche debris
- Flag the victim's locations autonomously
- Reduce search time for avalanche victims.

## Table of Contents

- [Avidrone Avalanche Rescue 🏔⛑](#avidrone-avalanche-rescue-)
  - [Objective](#objective)
  - [Table of Contents](#table-of-contents)
  - [Getting Started](#getting-started)
    - [Set up](#set-up)
  - [How to use](#how-to-use)
    - [Search](#search)
    - [Example: Running a Basic Mission](#example-running-a-basic-mission)
    - [Example: Running a simulated search mission](#example-running-a-simulated-search-mission)
  - [How to contribute](#how-to-contribute)
  - [Contributors](#contributors)
  - [License](#license)
  - [Contact](#contact)

## Getting Started

To clone this repository, run the following command on your terminal.

```{bash}
git clone https://github.com/AviDrone/backend.git --single-branch --branch main
```

### Set up

Before getting started, you will need to install a few libraries by running **install.sh**.

  ```{bash)
chmod -x install.sh && bash install.sh
  ```

This will take care of installing the libraries used in the backend of this project.

<!-- #### QGroundControl

 Avidrone search is compatible with the original version of QGroundControl while using command line arguments, and a few other steps.

<!--  TODO Add GUI tutorial here -->
To use our custom build of QGroundControl, you will need to follow this [guide](linkGUI)

You can find more details about using QGroundControl in the [frontend](https://github.com/AviDrone/frontend) repository.

<!-- TODO Add youtube video -->
You can also see this video [walkthrough](https://youtu.be/glC99FwFnAc) to learn how to connect and interact with the UAV. -->

## How to use

### Search

To run a full search mission using command-line arguments, run the following command from the root directory

  ```{bash)
  avidrone-search --run
  ```
  
 To run primary search only, use the following command

  ```{bash)
  avidrone-search --primary
  ```
  
 To run secondary search only, use the following command

  ```{bash)
  avidrone-search --secondary
  ```

### Example: Running a Basic Mission

### Example: Running a simulated search mission

## How to contribute

To contribute, carefully read the [CONTRIBUTING.md](CONTRIBUTING.md) file.

## Contributors

<!-- TODO Update contributors -->
Thanks goes to these wonderful people

- Samuel Hernandez
- Nathaniel Price
- Stefan Hess
- Matthew Harter

## License

Licensed under the [LGPL-2.1](https://www.gnu.org/licenses/lgpl-3.0.html) License.

## Contact

For any questions, feel free to contact me at any time through email [Samuel S. Hernandez](mailto:samuel.hernandez@wallawalla.edu)
