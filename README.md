# Avidrone Avalanche Rescue 🏔⛑

<!-- BADGE:START - Do not remove or modify this section -->
[![All Contributors](https://img.shields.io/badge/all_contributors-4-orange.svg?style=flat-square)](#contributors-)
![license](https://img.shields.io/github/license/AviDrone/AviDrone)
![stars](https://img.shields.io/github/stars/AviDrone/AviDrone?style=social)

<!-- BADGE:END -->

## Table of Contents

- [Avidrone Avalanche Rescue 🏔⛑](#avidrone-avalanche-rescue-)
  - [Table of Contents](#table-of-contents)
  - [AviDrone Project Introduction](#avidrone-project-introduction)
  - [AviDrone GitHub Page Introduction](#avidrone-github-page-introduction)
  - [Getting Started](#getting-started)
    - [Set up](#set-up)
  - [How to use](#how-to-use)
    - [Examples](#examples)
    - [Search](#search)
    - [Simulation](#simulation)
  - [How to contribute](#how-to-contribute)
  - [Contributors](#contributors)
  - [License](#license)
  - [Contact](#contact)

## AviDrone Project Introduction

This project continues the AviDrone Project from last year, whose goal was to create a drone capable of completing this search process autonomously. This year, our goals were to resolve the hardware's shortcomings, expand the search procedure algorithms by implementing the primary search procedure, test tools for secondary search, and create a user-friendly interface to lead to a faster and safer victim search.

During the 2021-2022 academic year, we expanded search missions by implementing primary search and creating testing tools for both search algorithms. We improved hardware by fixing pre-existing shortcomings which prevented practical use. Additionally, we designed a user interface for fast deployment and an improved user experience. We successfully improved the drone on all fronts, bringing it closer to real-life deployment.

## AviDrone GitHub Page Introduction

This page is primarily set up with the code, software explanations, and relevant documentation required to set up the software portion of this project. To access further hardware documentation, as well as documentation created for our class requirements, contact us. This code is currently specifically configured and set up to simulate the various aspects of the AviDrone search stages, additional work would be required before running it on a physical drone.

## Getting Started

To clone this repository, run the following command on your terminal.

```{bash}
git clone https://github.com/AviDrone/backend.git --single-branch --branch main
```

### Set up

Before getting started, you will need to install a few libraries by running **install.sh**.

  ```{bash)
chmod -x scripts/install.sh && bash scripts/install.sh
  ```

This will take care of installing the libraries used in the backend of this project.

It is possible to run a search mission using our command-line arguments (CLI) arguments, but using our frontend will definitely speed things up. To install QGroundControl for Avidrone, see the [frontend repository](https://github.com/AviDrone/frontend).

## How to use

### Examples

Go through a few examples so you can learn to

- Run a basic mission
- Run a transceiver simulation
- Run a search mission simulation

 You can find the examples in [EXAMPLE.md](app/example/EXAMPLE.md).

### Search

To run a full search mission using CLI arguments, run

```{bash)
avidrone-search --run
```

To run primary search only, use

```{bash)
avidrone-search --primary
```

To run secondary search only, use

```{bash)
avidrone-search --secondary
```

If you'd like to learn more, run the following command to see what other things you can do with avidrone-search.

```{bash)
avidrone-search --help
```

### Simulation

To run a simulated search mission, follow the guide in [SIMULATION.md](app/example/SIMULATION.md).

## How to contribute

To contribute, carefully read the [CONTRIBUTING.md](CONTRIBUTING.md) file.

## Contributors

Thanks goes to these wonderful people

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
- [Samuel Hernandez](https://github.com/SamuelSHernandez)
- [Nate Price](https://github.com/pricna)
- [Stefan Hess](https://github.com/btw-ILTG)
- [Matthew Harter](https://github.com/matthartpi)
<!-- ALL-CONTRIBUTORS-LIST:END -->

## License

Licensed under the [LGPL-2.1](https://www.gnu.org/licenses/lgpl-3.0.html) License.

## Contact

For any questions, feel free to contact me at any time through email

- [Samuel S. Hernandez](mailto:samuel.hernandez@wallawalla.edu)
