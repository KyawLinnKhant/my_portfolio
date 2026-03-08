window.PROJECTS = window.PROJECTS || {};

/* ─────────────────────────────────────────────────────────
   1. AI Self-Balancing Robot
───────────────────────────────────────────────────────── */
window.PROJECTS["rl-balance"] = {
  title: "AI Self-Balancing Robot",
  status: "Completed",
  cover: "src/sbr/v1.jpg",
  tags: ["Sim2Real", "PPO", "MuJoCo", "TFLite", "Teensy 4.1", "Raspberry Pi"],
  desc: "Reinforcement learning-based self-balancing robot with 85% Sim2Real transfer efficiency. PPO policy trained in MuJoCo, quantised to int8 TFLite, and deployed on a Teensy 4.1 microcontroller achieving <10 ms real-time control — zero GPU required.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          This project benchmarks five RL algorithms (A2C, TD3, SAC, DDPG, PPO) in MuJoCo with
          domain randomisation to identify the best candidate for embedded microcontroller deployment.
          PPO was selected for its small model size, training stability, and full microcontroller
          compatibility — inspired by BDX/Open Duck Mini bipedal locomotion research.
        </p>
        <ul>
          <li>Benchmarked 5 RL algorithms under domain randomisation — eliminated SAC (unsupported Exp op in int8 TFLite) and DDPG (unstable delta wheel speed oscillation)</li>
          <li>Converted PPO policy: ONNX → onnx2tf → int8 TFLite, validated inference parity on hardware</li>
          <li>85% Sim2Real transfer via Kalman-filtered IMU + encoder fusion</li>
          <li>&lt;10 ms real-time control loop on Teensy 4.1 — zero GPU required</li>
        </ul>
      `
    },
    {
      heading: "Components",
      content: `
        <ul>
          <li>Teensy 4.1 (primary deployment microcontroller)</li>
          <li>Raspberry Pi 5 / Raspberry Pi Zero 2W (alternative controller options)</li>
          <li>Adafruit BNO085 9-DOF IMU (Kalman-filtered)</li>
          <li>12V 520-geared DC motors (1:30, ~333 rpm)</li>
          <li>AT8236 2-Channel Motor Driver</li>
          <li>Geekworm X1201 UPS Shield (RPi5 version)</li>
          <li>ESP-01S WiFi module (Teensy version)</li>
          <li>11.1V Li-ion rechargeable battery</li>
        </ul>
      `
    },
    {
      heading: "MuJoCo Training",
      video: "https://www.youtube.com/embed/Rddmhgkn35E",
      content: `
        <p>
          A simplified MuJoCo model captured the robot's core dynamics. PPO training with domain
          randomisation developed a robust balancing policy by minimising falls and reacting to
          perturbations. The policy was then converted ONNX → int8 TFLite for microcontroller deployment.
        </p>
      `
    },
    {
      heading: "Hardware Versions",
      content: `
        <div class="cad-grid">
          <figure>
            <img src="src/sbr/v1.jpg" alt="Raspberry Pi version" loading="lazy" decoding="async">
            <figcaption><strong>Raspberry Pi Version</strong></figcaption>
          </figure>
          <figure>
            <img src="src/sbr/v2.jpg" alt="Pi Zero version" loading="lazy" decoding="async">
            <figcaption><strong>Pi Zero Version</strong></figcaption>
          </figure>
          <figure>
            <img src="src/sbr/v3.jpg" alt="Teensy version" loading="lazy" decoding="async">
            <figcaption><strong>Teensy Version</strong></figcaption>
          </figure>
        </div>
      `
    },
    {
      heading: "Custom PCB",
      content: `
        <div class="cad-grid">
          <figure>
            <img src="src/sbr/pizero.PNG" alt="Pi Zero Custom PCB" loading="lazy" decoding="async">
            <figcaption><strong>Pi Zero Custom PCB</strong></figcaption>
          </figure>
          <figure>
            <img src="src/sbr/teensy.PNG" alt="Teensy Custom PCB" loading="lazy" decoding="async">
            <figcaption><strong>Teensy Custom PCB</strong></figcaption>
          </figure>
        </div>
      `
    },
    {
      heading: "Controller (Teensy 4.1 Version)",
      content: `<img src="src/sbr/remote.jpg" alt="LilyGO T-Display Controller" loading="lazy" decoding="async">`
    },
    {
      heading: "Demo Videos",
      videos: [
        "https://www.youtube.com/embed/ZISe_C3mYUs",
        "https://www.youtube.com/embed/6jLbrzNLcS4"
      ]
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   2. Smart Waste Sorting System
───────────────────────────────────────────────────────── */
window.PROJECTS["smart-waste"] = {
  title: "Smart Waste Sorting System",
  status: "Completed",
  cover: "src/waste/cover.jpg",
  tags: ["Computer Vision", "CNN", "Raspberry Pi", "Continual Learning", "Edge AI"],
  desc: "Full Physical AI system: self-trained CNN classifies waste in real time and triggers servo-controlled bin lid sorting — 95%+ accuracy at 30 FPS on a single Raspberry Pi with no GPU. Includes a zero-downtime continual learning redeployment pipeline.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          This project unifies perception, decision-making, and actuation on a single Raspberry Pi —
          a complete Physical AI pipeline with no cloud dependency and no GPU.
          A custom CNN classifies waste type in real time; servo-actuated bin lids then sort
          the item into the correct compartment automatically.
        </p>
        <ul>
          <li>95%+ classification accuracy at 30 FPS on Raspberry Pi (CPU only)</li>
          <li>Real-time perception → decision → servo actuation loop</li>
          <li>Continual learning: misclassifications are collected, model retrained and redeployed with zero downtime</li>
          <li>Self-contained edge deployment — no internet, no GPU required</li>
        </ul>
      `
    },
    {
      heading: "System Architecture",
      content: `
        <p>
          The pipeline runs entirely on-device:
        </p>
        <ul>
          <li><strong>Perception:</strong> Camera feed → CNN inference (TFLite int8 quantised model)</li>
          <li><strong>Classification:</strong> Waste categories (plastic, paper, metal, organic, general)</li>
          <li><strong>Actuation:</strong> GPIO → servo driver → correct bin lid opens</li>
          <li><strong>Feedback loop:</strong> Misclassified samples flagged → batch retrain → hot-swap model without downtime</li>
        </ul>
      `
    },
    {
      heading: "Components",
      content: `
        <ul>
          <li>Raspberry Pi 4B (primary compute)</li>
          <li>Raspberry Pi Camera Module v2</li>
          <li>SG90 / MG996R servo motors (bin lid actuation)</li>
          <li>Custom 3D-printed bin housing (FDM)</li>
          <li>TFLite int8 quantised CNN model</li>
          <li>Python · OpenCV · TensorFlow Lite · RPi.GPIO</li>
        </ul>
      `
    },
    {
      heading: "Demo Video",
      video: "https://www.youtube.com/embed/dQw4w9WgXcQ"
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   3. PiCar-X F1 — Autonomous Racing
───────────────────────────────────────────────────────── */
window.PROJECTS["picar-x-f1"] = {
  title: "PiCar-X F1 — Autonomous Racing",
  status: "In Progress",
  tags: ["Raspberry Pi", "Computer Vision", "OpenCV", "Python", "Autonomous", "Lane Detection", "Edge AI"],
  desc: "F1-inspired autonomous racing on a SunFounder PiCar-X — real-time lane detection, adaptive speed control through corners, and obstacle handling running entirely on-device on Raspberry Pi.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          An autonomous racing system built on the SunFounder PiCar-X, inspired by Formula 1 racing.
          The robot detects track lanes in real time using computer vision, dynamically adjusts speed
          through corners, and handles obstacles — all running on-device with no external compute.
        </p>
        <ul>
          <li>Real-time lane detection and steering control via OpenCV</li>
          <li>Adaptive speed: full throttle on straights, braking through corners</li>
          <li>Obstacle detection and avoidance</li>
          <li>Fully on-device inference on Raspberry Pi — no GPU required</li>
        </ul>
      `
    },
    {
      heading: "Components",
      content: `
        <ul>
          <li>SunFounder PiCar-X platform</li>
          <li>Raspberry Pi 4B</li>
          <li>Raspberry Pi Camera Module v2</li>
          <li>Python · OpenCV · RPi motor control library</li>
        </ul>
      `
    },
    {
      heading: "GitHub",
      content: `
        <p>
          <a href="https://github.com/KyawLinnKhant/picar-x-f1" target="_blank" rel="noopener">
            github.com/KyawLinnKhant/picar-x-f1
          </a>
        </p>
      `
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   4. Autonomous Microcontroller Vehicle
───────────────────────────────────────────────────────── */
window.PROJECTS["esd"] = {
  title: "Autonomous Microcontroller Vehicle",
  status: "Completed",
  cover: "src/esd/car.jpg",
  tags: ["STM32", "Ultrasonic", "IR Sensors", "Embedded C", "Path Planning"],
  desc: "Autonomous vehicle built on STM32 Nucleo using fused ultrasonic and IR sensor data for real-time obstacle avoidance and waypoint navigation. PID-stabilised heading with BLE telemetry.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          The autonomous vehicle combines low-level embedded control with on-board sensor fusion to
          achieve self-navigation. Ultrasonic and IR readings are filtered and fused for stable
          range estimates; a local planner selects steering commands that maximise clearance while
          progressing toward the current waypoint.
        </p>
        <ul>
          <li>Real-time obstacle detection and avoidance</li>
          <li>Waypoint / path-following navigation</li>
          <li>Sensor fusion (ultrasonic + IR) for reliable distance estimates</li>
          <li>PID heading control via PWM motor commands</li>
          <li>BLE telemetry (HM-10) for parameter adjustment</li>
        </ul>
      `
    },
    {
      heading: "Components",
      content: `
        <ul>
          <li>STM32 Nucleo-F411RE</li>
          <li>L298N motor driver</li>
          <li>3× HC-SR04 ultrasonic sensors</li>
          <li>3× Sharp GP2Y0A21YK IR distance sensors</li>
          <li>4× SG90 servos, 2× TT motors + 1 castor wheel</li>
          <li>BLE 4.0 HM-10 module</li>
          <li>12V Li-ion rechargeable battery</li>
        </ul>
      `
    },
    {
      heading: "Implementation",
      content: `
        <p>
          The STM32 runs the real-time control loop. Sensor readings are filtered and fused to obtain
          stable range estimates. A simple local planner selects steering commands that maximise
          clearance while progressing toward the current waypoint. Motor speeds are driven with PWM,
          with a PID term to keep heading stable during maneuvers.
        </p>
        <p>
          Modes supported: <em>Obstacle Avoidance</em> and <em>Waypoint / Path Following</em>.
          Telemetry and parameters adjustable over BLE via the HM-10 module.
        </p>
      `
    },
    {
      heading: "Robotic Arm",
      img: "src/esd/arm.jpg"
    },
    {
      heading: "Demo Videos",
      videos: [
        "https://www.youtube.com/embed/ghL9OhbhNL4",
        "https://www.youtube.com/embed/reB1dbGUm5w",
        "https://www.youtube.com/embed/EworkVR8Dv0"
      ]
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   5. Naruto Jutsu Detection
───────────────────────────────────────────────────────── */
window.PROJECTS["handsign"] = {
  title: "Naruto Jutsu Detection with Facial Recognition",
  status: "Completed",
  cover: "src/naruto/naruto-shippuden-manga-cover.jpg",
  tags: ["Computer Vision", "MediaPipe", "CNN", "Facial Recognition", "PyQt5"],
  desc: "Multimodal interface combining Naruto hand-sign detection and facial expression recognition. Actions trigger only when both signals match simultaneously — built with MediaPipe landmarks + pretrained CNN and a live PyQt5 control interface.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          A fusion of hand-sign detection (Naruto-style gestures) and facial expression recognition
          to trigger actions via a live PyQt5 interface. Actions are only triggered when both hand
          sign and facial expression match — reducing false positives and enabling inclusive,
          intuitive multi-modal control.
        </p>
        <ul>
          <li>Recognises Naruto hand signs (Bird, Snake, Shadow Clone, etc.)</li>
          <li>Detects facial expressions (smile, surprise, neutral)</li>
          <li>Dual-signal requirement for robust command triggering</li>
          <li>PyQt5 live interface with video feed and status display</li>
        </ul>
      `
    },
    {
      heading: "Hand Seals",
      img: "src/naruto/Hand.jpg"
    },
    {
      heading: "Implementation",
      content: `
        <p>
          MediaPipe detects 21 hand landmarks per frame; a classifier maps landmark geometry to
          sign poses. Facial expressions are analysed using a pretrained CNN on FER-2013.
          When both gesture and expression match pre-defined triggers, the GUI executes an action
          (zoom, snapshot, custom command).
        </p>
      `
    },
    {
      heading: "Demo Video",
      video: "https://youtu.be/sXeTo-7n7YI"
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   6. Multi-Robot Search and Rescue
───────────────────────────────────────────────────────── */
window.PROJECTS["searchrescue"] = {
  title: "Multi-Robot Search and Rescue Operation",
  status: "Completed",
  cover: "src/mrnd/cover.png",
  tags: ["ROS2", "CoppeliaSim", "Swarm Autonomy", "PyQt5", "Multi-Agent"],
  desc: "Swarm autonomy system in CoppeliaSim + ROS2: coordinated task allocation across quadcopters, drone-arm platforms, and ground robots for fully autonomous search-and-rescue. PyQt5 ground control interface with real-time mission planning and manual override.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          This system integrates aerial scanning, autonomous ground navigation, and a remote operator
          interface to support efficient post-disaster response. The quadcopter performs live
          reconnaissance and marks points of interest; BoomBot plans and drives the fastest route
          to each site.
        </p>
        <ul>
          <li>Real-time aerial scanning and target marking (GPS + depth + timestamp)</li>
          <li>Autonomous ground navigation with obstacle avoidance and fast path planning</li>
          <li>ROS2 node/topic architecture across 5+ robot types</li>
          <li>Operator oversight via PyQt5 GUI with video, depth feeds, and mission logs</li>
          <li>Extended to TurtleBot nav, robotic arm control, and differential-drive robots</li>
        </ul>
      `
    },
    {
      heading: "GUI Interface (PyQt5)",
      img: "src/mrnd/gui.png",
      content: `
        <p>A custom PyQt5 application serves as the central control hub:</p>
        <ul>
          <li>Live quadcopter video feed and depth visualisation</li>
          <li>GPS coordinate tracking and detection logging</li>
          <li>Mission management: assign, reorder, clear targets</li>
          <li>Speed control: manual slider and automatic adaptive mode</li>
        </ul>
      `
    },
    {
      heading: "Demo Video",
      video: "https://www.youtube.com/embed/JOIw4MDWK8o"
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   7. Dobot Magician Pick and Place
───────────────────────────────────────────────────────── */
window.PROJECTS["dobot-pickplace"] = {
  title: "Dobot Magician Pick and Place Simulation",
  status: "Completed",
  cover: "src/dobot/dcover.png",
  tags: ["CoppeliaSim", "Dobot", "Kinematics", "Pick-and-Place"],
  desc: "Automated pick-and-place pipeline using the Dobot Magician in CoppeliaSim — precise end-effector control, collision-aware path planning, and physics-accurate object interactions.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          The <strong>Dobot Magician</strong> is a compact robotic arm suited for teaching and light
          industrial tasks. This project builds a full CoppeliaSim scene with a complete pick-and-place
          routine including:
        </p>
        <ul>
          <li>Precise end-effector control for grasping and placement</li>
          <li>Collision-aware path planning</li>
          <li>Physics-accurate object interactions (grasp, carry, release)</li>
          <li>Workspace visualisation and motion profile analysis</li>
        </ul>
      `
    },
    {
      heading: "Simulation Details",
      content: `
        <p>
          <strong>CoppeliaSim</strong> was chosen for its rich API and realistic dynamics. The setup includes
          an accurate kinematic model of the Dobot Magician, custom Lua scripts for motion control and task
          sequencing, and scene rendering for debugging and cycle-time verification.
        </p>
      `
    },
    {
      heading: "Demo Video",
      video: "https://www.youtube.com/embed/qM2qZJ-XIbI"
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   8. AlphaMini Bricklaying Robot
───────────────────────────────────────────────────────── */
window.PROJECTS["alphamini-bricklaying"] = {
  title: "AlphaMini Bricklaying Robot",
  status: "Completed",
  cover: "src/alphamini/minicover.jpg",
  tags: ["Robotics", "Voice Control", "Manipulation", "Fusion 360", "CAD"],
  desc: "Voice-controlled bricklaying robot prototype with precise end-effector positioning and repeatable placement patterns. Multiple CAD iterations optimised for reach, stiffness, and payload.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          This prototype combines speech recognition with accurate robotic manipulation to automate
          brick placement in structured patterns — reducing manual effort and improving consistency.
        </p>
        <ul>
          <li>Voice command interface for intuitive operator control</li>
          <li>Precise end-effector positioning for accurate brick placement</li>
          <li>Pattern execution for consistent courses and spacing</li>
          <li>Real-time feedback with corrective micro-adjustments</li>
        </ul>
      `
    },
    {
      heading: "CAD Design Iterations",
      content: `
        <p>
          Multiple CAD iterations in Fusion 360 to optimise reach, stiffness, and payload capacity:
        </p>
        <div class="cad-grid">
          <figure>
            <img src="src/alphamini/vone.PNG" alt="CAD v1" loading="lazy" decoding="async">
            <figcaption>Base and linear guide</figcaption>
          </figure>
          <figure>
            <img src="src/alphamini/vtwo.PNG" alt="CAD v2" loading="lazy" decoding="async">
            <figcaption>Wrist + gripper assembly</figcaption>
          </figure>
          <figure>
            <img src="src/alphamini/vthree.PNG" alt="CAD v3" loading="lazy" decoding="async">
            <figcaption>Cable routing and guards</figcaption>
          </figure>
          <figure>
            <img src="src/alphamini/vfour.PNG" alt="CAD v4" loading="lazy" decoding="async">
            <figcaption>Full assembly and workspace envelope</figcaption>
          </figure>
        </div>
      `
    },
    {
      heading: "Demo Videos",
      videos: [
        "https://www.youtube.com/embed/_xXYHWuNsXk",
        "https://www.youtube.com/embed/HHGH923CYmc"
      ]
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   9. Wireless Zigbee Communication
───────────────────────────────────────────────────────── */
window.PROJECTS["zigbee-communication"] = {
  title: "Wireless Zigbee Communication",
  status: "Completed",
  cover: "src/zb/zbc.jpg",
  tags: ["IoT", "STM32", "Zigbee", "AES-128", "Embedded C"],
  desc: "Secure, low-power Zigbee (XBee-S2C) link for real-time control and sensor telemetry between STM32 nodes. AES-128 encrypted communication suitable for distributed robotics and remote embedded control.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          Two <strong>Zigbee (XBee-S2C)</strong> modules paired with <strong>STM32 Nucleo</strong> boards
          exchange data in real time. Each node interfaces with sensors and actuators, with traffic protected
          by <strong>AES-128</strong> encryption using Zigbee's built-in security layer.
        </p>
        <ul>
          <li>Real-time bidirectional sensor/actuator telemetry over Zigbee mesh</li>
          <li>AES-128 hardware encryption via XBee built-in security</li>
          <li>LDR + potentiometer inputs; SG90 servo actuation output</li>
          <li>Suitable for distributed robotics, remote sensing, and embedded control</li>
        </ul>
      `
    },
    {
      heading: "Components",
      content: `
        <ul>
          <li>2× Zigbee XBee-S2C radios</li>
          <li>STM32 Nucleo-F411RE + STM32 Nucleo-F401RE</li>
          <li>Light Dependent Resistor (LDR), Potentiometer</li>
          <li>SG90 continuous-rotation servo</li>
          <li>Resistors, jumper wires, breadboard</li>
        </ul>
      `
    },
    {
      heading: "Wiring — Transmitter Node",
      content: `
        <ul>
          <li>UART: STM32 <code>TX</code> ⇄ XBee <code>DIN</code>, STM32 <code>RX</code> ⇄ XBee <code>DOUT</code></li>
          <li>Power: 3.3V per XBee, common GND across both nodes</li>
        </ul>
      `,
      img: "src/zb/t.png"
    },
    {
      heading: "Wiring — Receiver Node",
      content: `<p>Receiver drives SG90 servo based on incoming setpoints from the transmitter node.</p>`,
      img: "src/zb/r.png"
    },
    {
      heading: "Demo Video",
      video: "https://www.youtube.com/embed/2xWlMc2PTuc"
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   10. Sensor and Signal Processing
───────────────────────────────────────────────────────── */
window.PROJECTS["sensor-signal-processing"] = {
  title: "Sensor and Signal Processing",
  status: "Completed",
  cover: "src/rssp/pi.jpg",
  tags: ["Raspberry Pi", "MCP3008", "ADC", "Ultrasonic", "Embedded"],
  desc: "Raspberry Pi system fusing analog (potentiometer via MCP3008 ADC) and ultrasonic distance readings to command a servo motor in real time. Limit-switch homing, obstacle detection, and LED state indicators.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          A Raspberry Pi reads an analog potentiometer via MCP3008 ADC and an HC-SR04 ultrasonic
          sensor, then drives an SG90 servo with real-time control logic. A limit switch provides
          homing and restart capability; LEDs indicate system state throughout.
        </p>
        <ul>
          <li>Homing with limit switch, then manual angle control via potentiometer</li>
          <li>Obstacle detection within ~15 cm pauses / restricts servo motion</li>
          <li>LED feedback for status and error conditions</li>
          <li>Clean re-initialisation on restart</li>
        </ul>
      `
    },
    {
      heading: "Components",
      content: `
        <ul>
          <li>Raspberry Pi 4B</li>
          <li>MCP3008 8-channel 10-bit ADC (SPI)</li>
          <li>SG90 servo motor</li>
          <li>HC-SR04 ultrasonic sensor</li>
          <li>Potentiometer, limit switch</li>
          <li>LEDs (red/green), resistors, jumper wires, breadboard</li>
        </ul>
      `
    },
    {
      heading: "Demo Video",
      video: "https://www.youtube.com/embed/Ezl3cftk74o"
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   11. Shibuya Traffic Light System
───────────────────────────────────────────────────────── */
window.PROJECTS["shibuya-traffic-light"] = {
  title: "Shibuya Traffic Light System",
  status: "Completed",
  cover: "src/de/stl.jpg",
  tags: ["STM32", "State Machine", "Embedded C", "Real-Time"],
  desc: "Microcontroller-based simulation of Shibuya Crossing using LEDs and button inputs. Real-time state machine with vehicle and pedestrian phases, debounced inputs, and manual system open/close override.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          A state-driven traffic controller inspired by Shibuya Crossing. The system coordinates
          vehicle and pedestrian LEDs through explicit phases, with button presses governing
          transitions and manual overrides.
        </p>
        <ul>
          <li><strong>DEFAULT</strong> — Idle layout (Green/Red); safe reset landing state</li>
          <li><strong>TRAFFIC_LIGHT_SEQUENCE</strong> — Synchronised vehicle lights; ends with blinking amber</li>
          <li><strong>BLINK_PG</strong> — Pedestrian crossing (Red → Green); onboard LED indicates crossing window</li>
          <li><strong>CLOSE</strong> — Powers down all LEDs; halts operation</li>
          <li><strong>OPEN</strong> — Reactivates system and returns to DEFAULT</li>
        </ul>
      `
    },
    {
      heading: "Components",
      content: `
        <ul>
          <li>STM32 Nucleo-F401RE</li>
          <li>5× Red LEDs, 5× Green LEDs, 4× Yellow LEDs</li>
          <li>14× current-limiting resistors</li>
          <li>Push buttons, jumper wires, breadboard</li>
        </ul>
      `
    },
    {
      heading: "Demo Video",
      video: "https://www.youtube.com/embed/o-3O5DQ864w"
    }
  ]
};


/* ─────────────────────────────────────────────────────────
   12. Chameleon-SRE
───────────────────────────────────────────────────────── */
window.PROJECTS["chameleon-sre"] = {
  title: "Chameleon-SRE",
  status: "In Progress",
  tags: ["Python", "LangGraph", "Ollama", "Kubernetes", "RAG", "ChromaDB", "Docker", "LLM"],
  desc: "Autonomous Site Reliability Engineer — a Compound AI System that monitors, diagnoses, and self-heals Kubernetes clusters. Runs 100% locally on Apple Silicon via Ollama with a LangGraph state machine, RAG knowledge base, and direct kubectl access.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          Chameleon-SRE operates as a stateful agent using a Think → Act → Observe → Correct reasoning
          loop, rather than a linear chain. It autonomously detects cluster anomalies, queries a RAG
          knowledge base for resolution playbooks, and applies fixes — all without human intervention.
        </p>
        <ul>
          <li><strong>100% Local</strong> — Runs on Apple Silicon via Ollama (zero cloud costs)</li>
          <li><strong>Self-Healing</strong> — Autonomous error detection and correction loops</li>
          <li><strong>RAG-Powered</strong> — ChromaDB knowledge base of K8s docs and incident logs</li>
          <li><strong>Kubernetes Native</strong> — Direct cluster access via kubectl on Minikube</li>
          <li><strong>Observable</strong> — Full LangSmith tracing for debugging agent decisions</li>
        </ul>
      `
    },
    {
      heading: "Architecture",
      content: `
        <ul>
          <li><strong>Cognitive Engine</strong> — Llama 3.2 (3B) on Ollama</li>
          <li><strong>Orchestrator</strong> — LangGraph State Machine</li>
          <li><strong>Tools</strong> — kubectl, RAG Search, Voice Alerts</li>
          <li><strong>Knowledge Base</strong> — ChromaDB with K8s docs, error playbooks, incident logs</li>
          <li><strong>Infrastructure</strong> — Minikube cluster (Pods, Services, Deployments, ConfigMaps)</li>
        </ul>
      `
    },
    {
      heading: "Tech Stack",
      content: `
        <ul>
          <li>Python 3.10+, LangGraph, LangChain</li>
          <li>Ollama (Llama 3.2) — local LLM inference</li>
          <li>ChromaDB — vector store for RAG</li>
          <li>kubectl + Minikube — Kubernetes cluster management</li>
          <li>Docker + docker-compose — containerised deployment</li>
          <li>LangSmith — agent observability and tracing</li>
        </ul>
      `
    },
    {
      heading: "Roadmap",
      content: `
        <ul>
          <li>✅ Phase 1 — Core agent with LangGraph</li>
          <li>✅ Phase 2 — RAG knowledge base</li>
          <li>✅ Phase 3 — Kubernetes deployment</li>
          <li>🔄 Phase 4 — Prometheus metrics integration</li>
          <li>🔄 Phase 5 — Slack / PagerDuty notifications</li>
          <li>⬜ Phase 6 — Multi-cluster support</li>
          <li>⬜ Phase 7 — Predictive failure detection</li>
        </ul>
      `
    },
    {
      heading: "GitHub",
      content: `
        <p>
          <a href="https://github.com/KyawLinnKhant/Chameleon_SRE" target="_blank" rel="noopener">
            github.com/KyawLinnKhant/Chameleon_SRE
          </a>
        </p>
      `
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   13. FC Barcelona Matches Prediction — ML
───────────────────────────────────────────────────────── */
window.PROJECTS["fcb-prediction"] = {
  title: "FC Barcelona Match Prediction — ML",
  status: "Completed",
  tags: ["Python", "Machine Learning", "Jupyter", "EDA", "La Liga", "Scikit-learn"],
  desc: "Machine learning pipeline to predict La Liga match outcomes (Win / Draw / Loss) using 6 seasons of data (2019–2025). Covers EDA, feature engineering, and comparison of Logistic Regression, Ensemble methods, and Neural Networks.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          Explores Spanish La Liga match data from 2019 to 2025 to uncover performance patterns and
          build predictive models. The study identifies which factors — home advantage, possession,
          expected goals, form — most influence match results.
        </p>
        <ul>
          <li>4,318 team-match records across 2,159 unique fixtures from 27 La Liga teams</li>
          <li>Features: goals (gf/ga), xG/xGA, possession, shots, venue, referee, attendance</li>
          <li>80/20 train-test split on historical data</li>
          <li>Algorithms compared: Logistic Regression, Ensemble methods, Neural Networks</li>
        </ul>
      `
    },
    {
      heading: "Key Findings",
      content: `
        <ul>
          <li><strong>Home Advantage</strong> — 44.6% win rate at home vs 27.9% away</li>
          <li><strong>Top Teams</strong> — Real Madrid led with 67.6% win rate; Barcelona at 65.7%</li>
          <li><strong>xG Correlation</strong> — Expected Goals show a strong positive correlation with actual goals</li>
          <li><strong>Attendance</strong> — Sunday matches averaged the highest attendance (~32,400)</li>
          <li><strong>Possession</strong> — Winners averaged 51.2% possession vs 48.8% for losers — not the sole driver</li>
        </ul>
      `
    },
    {
      heading: "Methodology",
      content: `
        <ul>
          <li><strong>Data Quality</strong> — Handled 22.6% missing attendance values; removed empty columns</li>
          <li><strong>EDA</strong> — Temporal trends, win rates by day of week, team efficiency analysis</li>
          <li><strong>Feature Engineering</strong> — Form streaks, efficiency ratios, venue encoding</li>
          <li><strong>Modelling</strong> — Randomised 80/20 split; evaluated across multiple classifiers</li>
        </ul>
      `
    },
    {
      heading: "GitHub",
      content: `
        <p>
          <a href="https://github.com/KyawLinnKhant/FC-Barcelona-Matches-Prediction-ML" target="_blank" rel="noopener">
            github.com/KyawLinnKhant/FC-Barcelona-Matches-Prediction-ML
          </a>
        </p>
      `
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   14. SLAM TurtleBot — ROS
───────────────────────────────────────────────────────── */
window.PROJECTS["slam-turtlebot-ros"] = {
  title: "SLAM TurtleBot — ROS",
  status: "In Progress",
  tags: ["ROS", "SLAM", "TurtleBot3", "LiDAR", "Nav2", "EKF", "C++", "Python"],
  desc: "ROS-based Simultaneous Localisation and Mapping (SLAM) on TurtleBot3 — autonomous map building with LiDAR, EKF localisation, and goal-directed navigation in unknown environments.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          A ROS-based SLAM system for TurtleBot3 enabling autonomous exploration and map building
          in unknown environments. The robot simultaneously constructs a map and localises itself
          within it, then uses the built map for autonomous goal-directed navigation.
        </p>
        <ul>
          <li>SLAM: simultaneous map construction and robot localisation via LiDAR</li>
          <li>Extended Kalman Filter (EKF) for robust pose estimation</li>
          <li>Autonomous navigation in unmapped environments using Nav2</li>
          <li>ROS node architecture for modular mapping, localisation, and path planning</li>
          <li>Validated in both simulation and on real TurtleBot3 hardware</li>
        </ul>
      `
    },
    {
      heading: "Tech Stack",
      content: `
        <ul>
          <li>ROS (Noetic / Humble), Nav2, slam_toolbox</li>
          <li>C++ (core nodes) · Python (utilities)</li>
          <li>LiDAR (RPLiDAR / simulated scan), wheel encoders</li>
          <li>RViz for visualisation</li>
          <li>TurtleBot3 Burger (real hardware + Gazebo simulation)</li>
        </ul>
      `
    },
    {
      heading: "GitHub",
      content: `
        <p>
          <a href="https://github.com/KyawLinnKhant/slam_turtlebot_ros" target="_blank" rel="noopener">
            github.com/KyawLinnKhant/slam_turtlebot_ros
          </a>
        </p>
      `
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   NEW. Swarm Drones — ROS/CS
───────────────────────────────────────────────────────── */
window.PROJECTS["swarm-drones"] = {
  title: "Swarm Drones — ROS/CS",
  status: "In Progress",
  tags: ["ROS2", "Swarm Robotics", "UAV", "Multi-Agent", "Python", "C++"],
  desc: "Multi-drone swarm coordination system built on ROS2 — distributed formation control, autonomous task allocation, and collision-free navigation across a fleet of UAVs.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          A swarm robotics framework for coordinating multiple drones using ROS2. The system
          implements distributed task allocation, formation control, and collision-free navigation
          across a fleet of UAVs — designed for scalable multi-agent aerial operations.
        </p>
        <ul>
          <li>Distributed swarm coordination via ROS2 pub/sub architecture</li>
          <li>Formation control with dynamic re-configuration</li>
          <li>Collision-free trajectory planning across multiple agents</li>
          <li>Scalable to arbitrary swarm sizes</li>
        </ul>
      `
    },
    {
      heading: "GitHub",
      content: `
        <p>
          <a href="https://github.com/KyawLinnKhant/Swarm_Drones_ROSCS" target="_blank" rel="noopener">
            github.com/KyawLinnKhant/Swarm_Drones_ROSCS
          </a>
        </p>
      `
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   NEW. Sim2Real Quadruped
───────────────────────────────────────────────────────── */
window.PROJECTS["sim2real-quad"] = {
  title: "Sim2Real Quadruped",
  status: "In Progress",
  tags: ["Sim2Real", "Quadruped", "RL", "Domain Randomisation", "Quadruped Robot", "Python", "Control"],
  desc: "Sim-to-Real transfer pipeline for quadruped control — RL policy trained in simulation with domain randomisation and deployed on real hardware with minimal performance gap.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          A Sim2Real transfer pipeline for quadruped locomotion control. An RL policy is trained
          in simulation with domain randomisation to bridge the reality gap, then deployed on
          real hardware — achieving stable flight and manoeuvre execution with minimal degradation.
        </p>
        <ul>
          <li>RL-based locomotion controller trained in simulation</li>
          <li>Domain randomisation to close the sim2real gap</li>
          <li>Policy transfer to real quadruped hardware</li>
          <li>Stable walking, trotting, and terrain adaptation on real hardware</li>
        </ul>
      `
    },
    {
      heading: "GitHub",
      content: `
        <p>
          <a href="https://github.com/KyawLinnKhant/sim2real-quad" target="_blank" rel="noopener">
            github.com/KyawLinnKhant/sim2real-quad
          </a>
        </p>
      `
    }
  ]
};

/* ─────────────────────────────────────────────────────────
   NEW. IsaacLab SO-ARM100/101
───────────────────────────────────────────────────────── */
window.PROJECTS["isaaclab-so-arm"] = {
  title: "IsaacLab SO-ARM100/101",
  status: "In Progress",
  tags: ["IsaacLab", "IsaacSim", "Sim2Real", "Manipulation", "RL", "Robotic Arm", "NVIDIA"],
  desc: "RL-based dexterous manipulation training for the SO-ARM100/101 robotic arm in NVIDIA IsaacLab — GPU-accelerated policy learning with Sim2Real transfer to physical hardware.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          Training dexterous manipulation policies for the SO-ARM100 and SO-ARM101 robotic arms
          using NVIDIA IsaacLab. Reinforcement learning policies are developed in GPU-accelerated
          simulation and transferred to real hardware for physical task execution.
        </p>
        <ul>
          <li>GPU-accelerated RL training in NVIDIA IsaacLab / IsaacSim</li>
          <li>Dexterous manipulation tasks: pick-and-place, object reorientation</li>
          <li>Domain randomisation for robust Sim2Real transfer</li>
          <li>Policy deployment on SO-ARM100 / SO-ARM101 physical hardware</li>
        </ul>
      `
    },
    {
      heading: "GitHub",
      content: `
        <p>
          <a href="https://github.com/KyawLinnKhant/IsaacLab_SO-ARM100-101" target="_blank" rel="noopener">
            github.com/KyawLinnKhant/IsaacLab_SO-ARM100-101
          </a>
        </p>
      `
    }
  ]
};

/* ═══════════════════════════════════════════════════════════
   RENDER — reads ?id= from the URL and populates the page
   project.html?id=rl-balance  →  loads PROJECTS["rl-balance"]
═══════════════════════════════════════════════════════════ */
(function () {
  /* Only run on project.html (page must have #p-title) */
  if (!document.getElementById('p-title')) return;

  /* ── Set footer year ── */
  const yEl = document.getElementById('y');
  if (yEl) yEl.textContent = new Date().getFullYear();

  /* ── YouTube helpers ── */
  function getYouTubeId(url) {
    try {
      const u = new URL(url);
      if (u.hostname.includes('youtu.be')) return u.pathname.slice(1);
      const v = u.searchParams.get('v'); if (v) return v;
      const parts = u.pathname.split('/').filter(Boolean);
      const iS = parts.indexOf('shorts'); if (iS >= 0 && parts[iS + 1]) return parts[iS + 1];
      const iE = parts.indexOf('embed');  if (iE >= 0 && parts[iE + 1]) return parts[iE + 1];
      return null;
    } catch { return null; }
  }
  function makeYT(id) {
    const d = document.createElement('div');
    d.className = 'yt-wrap';
    d.innerHTML = `<iframe src="https://www.youtube-nocookie.com/embed/${id}?rel=0&modestbranding=1"
      title="YouTube video" allow="accelerometer;clipboard-write;encrypted-media;gyroscope;picture-in-picture"
      allowfullscreen loading="lazy"></iframe>`;
    return d;
  }

  /* ── Read project ID from URL: project.html?id=rl-balance ── */
  const params = new URLSearchParams(window.location.search);
  const projectId = params.get('id') || 'rl-balance'; // fallback just in case
  const p = (window.PROJECTS || {})[projectId];

  if (!p) {
    document.getElementById('p-title').textContent = 'Project not found';
    return;
  }

  /* ── Populate page ── */
  document.title = (p.title || 'Project') + ' — Kyaw Linn Khant';
  document.getElementById('p-title').textContent = p.title || 'Project';

  if (p.status) {
    const st = document.getElementById('p-status');
    st.textContent = p.status; st.style.display = 'inline-block';
    const k = p.status.trim().toLowerCase();
    if (k.includes('complete'))             st.classList.add('complete');
    else if (k.includes('progress'))        st.classList.add('inprogress');
    else if (k.includes('draft') || k.includes('wip')) st.classList.add('draft');
  }

  const tagsEl = document.getElementById('p-tags');
  (p.tags || []).forEach(t => {
    const s = document.createElement('span');
    s.className = 'badge'; s.textContent = t; tagsEl.appendChild(s);
  });

  if (p.cover || p.img) {
    const img = document.getElementById('p-img');
    img.src = p.cover || p.img; img.alt = p.title || ''; img.style.display = 'block';
  }

  document.getElementById('p-desc').textContent = p.desc || '';

  const wrap = document.getElementById('p-sections');
  (p.sections || []).forEach(sec => {
    const d = document.createElement('div');
    d.className = 'p-section';

    if (sec.heading) {
      const h = document.createElement('h3'); h.textContent = sec.heading; d.appendChild(h);
    }

    const vids = sec.video ? [sec.video] : (Array.isArray(sec.videos) ? sec.videos : []);

    const addVid = url => {
      const vid = getYouTubeId(url);
      if (vid) {
        d.appendChild(makeYT(vid));
      } else {
        const i = document.createElement('iframe');
        Object.assign(i, { src: url, width: '100%', height: '400', frameBorder: '0', allowFullscreen: true, loading: 'lazy' });
        i.style.marginBottom = '16px'; d.appendChild(i);
      }
    };

    if (vids.length) {
      vids.forEach(addVid);
      if (sec.content) { const c = document.createElement('div'); c.className = 'project-description'; c.innerHTML = sec.content; d.appendChild(c); }
      if (sec.img)     { const im = document.createElement('img'); im.src = sec.img; im.alt = sec.heading || ''; d.appendChild(im); }
    } else {
      if (sec.content) { const c = document.createElement('div'); c.className = 'project-description'; c.innerHTML = sec.content; d.appendChild(c); }
      if (sec.img)     { const im = document.createElement('img'); im.src = sec.img; im.alt = sec.heading || ''; d.appendChild(im); }
    }

    wrap.appendChild(d);
  });

  if (p.media && (p.media.youtube || (p.media.images && p.media.images.length))) {
    const mediaEl = document.getElementById('p-media');
    mediaEl.style.display = 'block';
    const h = document.createElement('h3'); h.textContent = 'Media'; mediaEl.appendChild(h);
    if (p.media.youtube) { const vid = getYouTubeId(p.media.youtube); if (vid) mediaEl.appendChild(makeYT(vid)); }
    if (Array.isArray(p.media.images) && p.media.images.length) {
      const g = document.createElement('div'); g.className = 'gallery';
      p.media.images.forEach(src => {
        const im = document.createElement('img'); im.loading = 'lazy'; im.src = src; im.alt = p.title || ''; g.appendChild(im);
      });
      mediaEl.appendChild(g);
    }
  }
})();
