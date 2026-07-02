**Role:** You are an Expert Full-Stack Developer and Technical Architect specializing in interactive data visualizations and browser-based simulations.
**Context & Objective:** > I need to build a single-page web application within one weekend to serve as a portfolio piece for a Technical Sales Manager interview at a robotics/SLAM technology company. The project must demonstrate my deep understanding of the Extended Kalman Filter (EKF), my hands-on experience tuning vehicle state estimation algorithms, and my ability to translate these complex mathematical concepts into clear, value-driven business propositions for a corporate audience.
**Tech Stack Constraint:**
* Frontend: Vanilla HTML, CSS (flexbox/grid for clean dashboard layout), and vanilla JavaScript.
* Visualization: `p5.js` (loaded via CDN) for the 2D simulation canvas.
* Mathematics: `math.js` (loaded via CDN) for matrix operations (inversions, transpositions, multiplications) required by the EKF.
* Architecture: Keep it modular within a single `index.html` and `script.js` file for easy deployment.


**Project Specifications: "The EKF Tuner - Navigating Uncertainty"**
Please plan the implementation for the following components. Do not write the full code yet; provide the architectural plan, file structure, and step-by-step execution strategy first.
**1. The Core Simulation (p5.js Canvas)**
* **Setting:** A top-down 2D view of an industrial warehouse or high-tech laboratory environment.
* **The Actor:** A small, grey automated guided vehicle (AGV)—perhaps subtly resembling the dimensions of a compact grey hatchback—navigating a predefined path from Point A to Point B.
* **The Mechanics (Gamification):** Inspired by the chaotic, time-pressure mechanics of viral games like *Overcooked*, the robot operates in an unpredictable environment.
* **The Loop:** Before the robot starts, the user must tune two sliders representing the EKF Covariance Matrices:
* Slider 1: *Process Noise ($Q$)* - "Trust the Internal Model (Odometry)"
* Slider 2: *Measurement Noise ($R$)* - "Trust the Sensors (Visual SLAM landmarks)"


* **The Distinctive Trap:** Halfway through the route, the robot hits a "slippery floor" or "featureless corridor." Here, wheel odometry fails (simulating off-road or slip dynamics), or visual landmarks disappear. The EKF must dynamically rely on the correct sensor. If the user tuned the sliders poorly, the estimated state diverges rapidly from the true state, and the robot crashes violently into a rack, triggering a "Game Over" state. If tuned correctly, it dynamically recovers and reaches Point B.
* **Visuals:** Draw three distinct elements on the canvas: the True Position (hidden/faded), the Raw Sensor Measurement (noisy, jumping dots), and the EKF Estimated Position (a solid, smoothed line).


**2. The Business Value Dashboard (UI/UX)**
Next to the canvas, build a clean, corporate dashboard to display real-time ROI and commercial value.
* **Live Metrics:** Display the "Hardware Savings Calculator." When the EKF is highly optimized (error covariance is low), show a metric like: *"Optimal Tuning Reached: Sensor cost reduced by 40% per unit."*
* **Sales Narrative:** Briefly explain how overcoming the "slippery floor" scenario reduces delivery risk and prevents catastrophic downtime in automated logistics.


**3. The Explanatory Tiers (Accordion/Tabbed UI)**
Below the simulation, create an area with three distinct explanations of the technology to prove my communication range as a Technical Account Manager:
* **The Executive View:** A high-level, simple analogy focused on business continuity and reliability (e.g., "Walking in a dark room and touching a wall to correct drift").
* **The Engineer View:** A mid-level explanation detailing the Predict and Update loop, and how the covariance matrices act as dynamic confidence scores.
* **The Specialist View:** Include the advanced mathematical equations (using simple HTML math formatting or an image placeholder), specifically showing the Kalman Gain equation and the Jacobian linearization process for non-linear systems.


**Instructions for the AI:**
1. Acknowledge these requirements.
2. Output a step-by-step development plan detailing how we will tackle the `math.js` EKF class first, the `p5.js` rendering second, and the UI integration last.
3. Provide the mathematical skeleton (just the class structure and matrix dimensions) for a 2D state vector $(x, y, \theta, v)$ to ensure we are aligned on the physics.