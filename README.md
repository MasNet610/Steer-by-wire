# Steer-by-wire

This project presents the development of a compact, pseudo-realistic Steer-by-Wire (SBW) demonstration facility
aimed at showcasing the core concepts and operational principles behind modern vehicle steering systems. The
model replicates the structure and function of commercial SBW implementations, emphasizing modular design,
electronic control integration, and mechanical realism within a scale-appropriate prototype. Core subsystems
include a steering input module, an electronic control unit (ECU), and a steering actuation module, all designed
with real-time signal interchange and actuation dynamics. The model prioritizes educational value, system
scalability, and ease of future enhancement. Comprehensive testing and literature integration guide the design
process, balancing mechanical accuracy with cost, space, and feasibility constraints. This facility serves as both
a learning platform and a foundation for further research into steer-by-wire technologies.

NOTES:
Changes that need to be made post project completion:
- Steering Wheel Motor returned is a 65rpm motor, however, the model is designed around a 270rpm motor (as noted in the report) that was provided by the student.
- MATLAB and circuit layout are designed for Arduino UNO. However, it does not have enough memory to operate adequately
- Rack and pinion sizes should be recalculated