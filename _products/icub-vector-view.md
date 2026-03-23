---
title: "Real-time contact force visualization on a humanoid robot"
company: "ISIR — CNRS / UPMC"
product: "VectorView & VectorGUI"
start_date: "2015"
end_date: "2015"
layout: product
---

In 2015, I joined [ISIR](http://www.isir.upmc.fr/) (Institut des Systèmes Intelligents et de Robotique, a joint research lab of CNRS and Université Pierre et Marie Curie) in Paris for an internship focused on humanoid robotics.

The project centred on the [iCub](https://icub.iit.it/) — one of the most sophisticated open-platform humanoid robots in the world. Developed by the Italian Institute of Technology and widely used across European research institutions, iCub is a full-size child-shaped humanoid capable of complex whole-body interaction with its environment. The goal was to make the contact forces that arise during such interactions **visible in real time**, directly inside the Gazebo physics simulator.

The result was two complementary tools:

- **VectorView** — a Gazebo visual plugin that attaches to any link of the iCub model and renders a live 3-D force vector at the exact contact point, giving researchers and engineers an immediate spatial understanding of every collision and interaction happening on the robot's body.
- **VectorGUI** — a Qt desktop application that streams contact telemetry from the simulation, displays force magnitudes and contact locations numerically, and lets operators spawn physical objects into the scene on the fly to trigger new interaction scenarios.

## Technical Highlights

The work required integrating across the full humanoid robotics middleware stack: Gazebo for simulation, YARP for inter-process communication, and the iCub SDK for model and sensor access.

Raw contact sensor data is inherently noisy. A custom DSP filtering pipeline (using [DSPFilters](https://github.com/vinniefalco/DSPFilters)) was built to clean the signal before rendering, ensuring the visualised vectors reflect meaningful physical forces rather than sensor artefacts.

The simulation environment was driven by the [ocra-core](https://github.com/ocra-recipes/ocra-core) whole-body controller, running predefined task sequences to exercise the robot's contacts systematically.

## Additional Resources

- [Source code](https://github.com/alexandrelheinen/vector-view)
- [iCub humanoid robot](https://icub.iit.it/)
- [ISIR laboratory](http://www.isir.upmc.fr/)
