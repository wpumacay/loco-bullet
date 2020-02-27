**Status**: Heavy development (you might run into crashes as the library is being developed, sorry in advance).

# tysocBullet: A locomotion toolkit, with Bullet as physics backend.

[![Build Status](https://travis-ci.com/wpumacay/tysocBullet.svg?branch=master)](https://travis-ci.com/wpumacay/tysocBullet)

[gif-demo-sample](https://media.giphy.com/media/ZDEAQSUraLao0fOhHi/giphy.gif)

This is an instance of the [**loco**](https://github.com/wpumacay/tysocCore) framework for locomotion, 
using [**Bullet**](http://bulletphysics.org) as physics backend. As explained in the core repository, the idea
is to provide a kind of abstraction layer on top of various physics backends, and allow you to just
focus on making your experiment regardless of the details of each specific backend.

I will be adding more documentation as I develop the library, and sorry in advance as I might forget 
to update the docs from time to time. However, one main objective is to write comprehensive documentation, 
and I will be doing it on the go. If you have any suggestions/issues, just post an issue or contact me 
at wpumacay@gmail.com .

## Setup (WIP)

### Requirements

#### Ubuntu >= 16.04

```bash
sudo apt install make cmake pkg-config
sudo apt install libassimp-dev libglfw3-dev libglew-dev
```

### Building

#### Ubuntu >= 16.04

```bash
# clone this repository (comes without dependencies)
git clone https://github.com/wpumacay/tysocBullet.git
# clone the third_party dependencies
cd tysocBullet && ./scripts/setup_dependencies.sh
# build the project
mkdir build && cd build && cmake .. && make -j4
```