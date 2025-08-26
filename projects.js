window.PROJECTS = window.PROJECTS || {};

// AI Self-Balancing Robot
window.PROJECTS["rl-balance"] = {
  title: "AI Self-Balancing Robot",
  status: "Completed", // shows green badge automatically
  cover:
    "src/sbr/v1.jpg",
  tags: ["Sim2Real", "RL", "MuJoCo", "Raspberry Pi"],
  desc:
    "A self-balancing robot trained with reinforcement learning. It maintains balance via continuous sensor-driven control and sim-to-real transfer.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          This robot demonstrates the integration of reinforcement learning with real-world robotics. 
          The system continuously adjusts motor actuation to keep the robot upright, adapting to external 
          disturbances and dynamic environments.
        </p>
        <ul>
          <li>Real-time sensor data processing</li>
          <li>Reinforcement learning control (PPO)</li>
          <li>Precise motor actuation for balance</li>
          <li>Adaptive response to disturbances</li>
        </ul>
      `
    },
    {
      heading: "Components",
      content: `
        <ul>
          <li>Raspberry Pi 5 (main controller option)</li>
          <li>Raspberry Pi Zero 2 W (lightweight option)</li>
          <li>Teensy 4.1 (high-performance microcontroller option)</li>
          <li>12V 520-geared DC motors (1:30, ~333 rpm)</li>
          <li>AT8236 2-Channel Motor Driver</li>
          <li>Adafruit BNO085 9-DOF IMU</li>
          <li>Geekworm X1201 UPS Shield (for Raspberry Pi 5)</li>
          <li>12V Li-ion rechargeable battery</li>
        </ul>
      `
    },
    {
      heading: "MuJoCo Training",
      video: "https://www.youtube.com/embed/Rddmhgkn35E",
      content: `
        <p>
          A simplified MuJoCo model captured the robot’s core dynamics. Training 
          with PPO reinforcement learning developed a robust balancing policy by 
          minimizing falls and reacting to perturbations. The policy was then 
          deployed to the physical robot for reliable self-balancing behavior.
        </p>
      `
    },
    {
      heading: "Hardware Versions",
      content: `
        <div class="cad-grid">
          <figure>
            <img src="src/sbr/v1.jpg" alt="Self-balancing robot — Raspberry Pi version" loading="lazy" decoding="async">
            <figcaption><strong>Raspberry Pi Version</strong></figcaption>
          </figure>
          <figure>
            <img src="src/sbr/v2.jpg" alt="Self-balancing robot — Pi Zero version" loading="lazy" decoding="async">
            <figcaption><strong>Pi Zero Version</strong></figcaption>
          </figure>
          <figure>
            <img src="src/sbr/v3.jpg" alt="Self-balancing robot — Teensy version" loading="lazy" decoding="async">
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
            <img src="src/sbr/pizero.PNG" alt="Custom PCB — Pi Zero" loading="lazy" decoding="async">
            <figcaption><strong>Pi Zero Custom PCB</strong></figcaption>
          </figure>
          <figure>
            <img src="src/sbr/teensy.PNG" alt="Custom PCB — Teensy" loading="lazy" decoding="async">
            <figcaption><strong>Teensy Custom PCB</strong></figcaption>
          </figure>
        </div>
      `
    },
    {
      heading: "Demo Video",
      videos: [
        "https://www.youtube.com/embed/ZISe_C3mYUs",
        "https://www.youtube.com/embed/6jLbrzNLcS4"
      ]
    }
  ],
};

// Autonomous Microcontroller Vehicle
window.PROJECTS["esd"] = {
  title: "Autonomous Microcontroller Vehicle",
  status: "Completed",
  cover:
    "src/esd/car.jpg",
  tags: ["STM32", "Ultrasonic", "IR Sensors", "Path Planning"],
  desc:
    "This project demonstrates the development of an autonomous vehicle using embedded systems and sensor integration. The vehicle navigates while avoiding obstacles and following predefined paths.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        The autonomous vehicle combines low-level embedded control with on-board sensing to
        achieve self-navigation. Key features include:
        <ul>
          <li>Real-time obstacle detection and avoidance</li>
          <li>Path-planning / waypoint navigation</li>
          <li>Sensor fusion for more reliable distance estimates</li>
          <li>Autonomous decision-making for route selection</li>
        </ul>
      `
    },
    {
      heading: "Components",
      content: `
        <ul>
          <li>STM32 Nucleo-F411RE</li>
          <li>L298N motor driver</li>
          <li>12V Li-ion rechargeable battery</li>
          <li>3× HC-SR04 ultrasonic sensors</li>
          <li>3× Sharp GP2Y0A21YK IR distance sensors</li>
          <li>2× TT motors + 1 castor wheel</li>
          <li>4× SG90 servos</li>
          <li>BLE 4.0 HM-10</li>
          <li>Push button, resistors, jumper wires, breadboard</li>
        </ul>
      `
    },
    {
      heading: "Implementation",
      content: `
        <p>
          The STM32 runs the real-time control loop. Ultrasonic and IR readings are filtered and
          fused to obtain stable range estimates. A simple local planner selects steering commands
          that maximize clearance while progressing toward the current waypoint. Motor speeds are
          driven with PWM, with a PID term to keep heading stable during maneuvers.
        </p>
        <p>
          Modes supported: <em>Obstacle Avoidance</em> and <em>Waypoint/Path Following</em>.
          Telemetry/parameters can be adjusted over BLE using the HM-10 module.
        </p>
      `
    },
    {
      heading: "Robotic Arm",
      img: "src/esd/arm.jpg"
    },
    {
      heading: "Demo Video",
      videos: [
        "https://www.youtube.com/embed/ghL9OhbhNL4",
        "https://www.youtube.com/embed/reB1dbGUm5w",
        "https://www.youtube.com/embed/EworkVR8Dv0"
      ]
    }
  ]
};
window.PROJECTS = window.PROJECTS || {};

window.PROJECTS["searchrescue"] = {
  title: "Search and Rescue Operation with BoomBot and Quadcopter",
  status: "Completed",
  cover: "src/mrnd/cover.png",
  tags: ["CoppeliaSim", "PyQt5", "Quadcopter", "Ground Robot", "Multi-Agent"],
  desc:
    "A coordinated aerial–ground system: a quadcopter scans and flags targets; BoomBot autonomously navigates fastest paths to each point for flood-recovery missions.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          This system integrates aerial scanning, autonomous ground navigation, and a remote
          operator interface to support efficient, safe post-disaster response.
          The quadcopter performs live reconnaissance and marks points of interest with
          coordinates; BoomBot then plans and drives the fastest route to each site.
        </p>
        <ul>
          <li>Real-time aerial scanning and target marking (GPS + depth + timestamp)</li>
          <li>Autonomous ground navigation with obstacle avoidance and fast path planning</li>
          <li>Operator oversight via PyQt5 GUI with video, depth, and logs</li>
          <li>Multi-target allocation for broad coverage in one mission</li>
        </ul>
      `
    },
    {
      heading: "Simulation Software",
      content: `
        <p>
          Development and testing were performed in <strong>CoppeliaSim</strong>,
          validating quadcopter flight patterns, BoomBot navigation algorithms,
          and inter-agent coordination in a safe, repeatable environment prior to deployment.
        </p>
      `
    },
    {
      heading: "GUI Interface (PyQt5)",
      img: "src/mrnd/gui.png",
      content: `
        <p>
          A custom <strong>PyQt5</strong> application serves as the central control hub:
        </p>
        <ul>
          <li>Live quadcopter video feed and depth visualization</li>
          <li>GPS coordinate tracking and logging of detections</li>
          <li>Robot status/telemetry monitoring</li>
          <li>Mission management (assign, reorder, clear targets)</li>
          <li>Speed control: manual slider and automatic adaptive mode</li>
        </ul>
        <p>
          In <em>manual</em> mode, operators set speed with a slider for precise maneuvers.
          In <em>automatic</em> mode, BoomBot adjusts speed from environmental feedback
          (e.g., proximity, terrain) to balance speed and safety.
        </p>
      `
    },
    {
      heading: "Demo Video",
      video: "https://www.youtube.com/embed/JOIw4MDWK8o"
    }
  ]
};
window.PROJECTS = window.PROJECTS || {};

window.PROJECTS["dobot-pickplace"] = {
  title: "Dobot Magician Pick and Place Simulation",
  status: "Completed",
  cover:
    "src/dobot/dcover.png",
  tags: ["CoppeliaSim", "Dobot", "Pick-and-Place"],
  desc:
    "Automated pick-and-place pipeline using the Dobot Magician in CoppeliaSim with precise end-effector control and collision-aware path planning.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          The <strong>Dobot Magician</strong> is a compact, versatile robotic arm suited for
          teaching and light industrial tasks. This project builds a CoppeliaSim scene to
          showcase a complete pick-and-place routine with:
        </p>
        <ul>
          <li>Precise end-effector control for grasping and placement</li>
          <li>Path planning with collision avoidance</li>
          <li>Physics-accurate object interactions (grasp, carry, release)</li>
          <li>Clear visualization of the workspace and motion profile</li>
        </ul>
      `
    },
    {
      heading: "Simulation Software",
      content: `
        <p>
          <strong>CoppeliaSim</strong> (V-REP) was chosen for its rich API and realistic
          dynamics. The setup includes:
        </p>
        <ul>
          <li>Accurate kinematic model of the Dobot Magician</li>
          <li>Custom scripts for motion control and task sequencing</li>
          <li>Scene rendering and sensor feedback for debugging</li>
          <li>Performance checks for cycle-time and reachability</li>
        </ul>
      `
    },
    {
      heading: "Demo Video",
      video: "https://www.youtube.com/embed/qM2qZJ-XIbI"
    }
  ]
};
window.PROJECTS = window.PROJECTS || {};

window.PROJECTS["alphamini-bricklaying"] = {
  title: "AlphaMini Bricklaying Robot",
  status: "Completed",
  cover:
    "src/alphamini/minicover.jpg",
  tags: ["Robotics", "Voice Control", "Manipulation", "CAD"],
  desc:
    "A voice-controlled bricklaying robot prototype demonstrating precise end-effector control and repeatable placement patterns.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          This prototype combines speech recognition with accurate robotic manipulation to automate
          brick placement in structured patterns—reducing manual effort and improving consistency.
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
      heading: "CAD Models",
      content: `
        <p>
          The mechanical design went through multiple CAD iterations to optimize reach, stiffness,
          and payload for a typical brick. Selected views:
        </p>

        <div class="cad-grid">
          <figure>
            <img src="src/alphamini/vone.PNG" alt="CAD Model 1 — Base and linear guide" loading="lazy" decoding="async" width="1200" height="900">
            <figcaption>CAD Model 1 — Base and linear guide</figcaption>
          </figure>
          <figure>
            <img src="src/alphamini/vtwo.PNG" alt="CAD Model 2 — Wrist + gripper assembly" loading="lazy" decoding="async" width="1200" height="900">
            <figcaption>CAD Model 2 — Wrist + gripper assembly</figcaption>
          </figure>
          <figure>
            <img src="src/alphamini/vthree.PNG" alt="CAD Model 3 — Cable routing and guards" loading="lazy" decoding="async" width="1200" height="900">
            <figcaption>CAD Model 3 — Cable routing and guards</figcaption>
          </figure>
          <figure>
            <img src="src/alphamini/vfour.PNG" alt="CAD Model 4 — Full assembly and workspace envelope" loading="lazy" decoding="async" width="1200" height="900">
            <figcaption>CAD Model 4 — Full assembly and workspace envelope</figcaption>
          </figure>
        </div>
      `
    },
    {
      heading: "Demo Video",
      videos: [
        "https://www.youtube.com/embed/_xXYHWuNsXk",
        "https://www.youtube.com/embed/HHGH923CYmc"
      ]
    }
  ]
};
window.PROJECTS = window.PROJECTS || {};

window.PROJECTS["zigbee-communication"] = {
  title: "Wireless Zigbee Communication",
  status: "Completed",
  cover:
    "src/zb/zbc.jpg",
  tags: ["IoT", "STM32", "Zigbee", "AES-128"],
  desc:
    "Secure, low-power Zigbee (XBee-S2C) link for real-time control and sensor telemetry between STM32 nodes, using AES-128 encryption.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          Two <strong>Zigbee (XBee-S2C)</strong> modules paired with <strong>STM32 Nucleo</strong> boards (F411RE and F401RE)
          exchange data in real time. Each node interfaces with sensors/actuators—an LDR and potentiometer for inputs,
          and an SG90 continuous-rotation servo for actuation. Traffic is protected with <strong>AES-128</strong> using Zigbee’s
          built-in security, making it suitable for distributed robotics, remote sensing, and embedded control.
        </p>
      `
    },
    {
      heading: "Components",
      content: `
        <ul>
          <li>2× Zigbee (XBee-S2C) radios</li>
          <li>STM32 Nucleo-F411RE</li>
          <li>STM32 Nucleo-F401RE</li>
          <li>Light Dependent Resistor (LDR)</li>
          <li>Potentiometer</li>
          <li>SG90 continuous-rotation servo</li>
          <li>Resistors, jumper wires, breadboard</li>
        </ul>
      `
    },
    {
      heading: "Wiring Diagrams",
      content: `
        <p>The following diagrams illustrate TX/RX wiring for both Zigbee nodes.</p>
        <ul>
          <li>UART: STM32 <code>TX</code> ⇄ XBee <code>DIN</code>, STM32 <code>RX</code> ⇄ XBee <code>DOUT</code></li>
          <li>Power: 3.3V supply per XBee module, common GND across both nodes</li>
          <li>Servo powered from a suitable 5V rail (separate from logic), GND commoned</li>
        </ul>
      `,
      img: "src/zb/t.png"
    },
    {
      heading: "Wiring Diagrams (Receiver Node)",
      content: `
        <p>Receiver connects Zigbee to MCU UART and drives the SG90 based on incoming setpoints.</p>
      `,
      img: "src/zb/r.png"
    },
    {
      heading: "Demo Video",
      videos: [
        "https://www.youtube.com/embed/2xWlMc2PTuc"
      ]
    }
  ]
};
window.PROJECTS = window.PROJECTS || {};

window.PROJECTS["sensor-signal-processing"] = {
  title: "Sensor and Signal Processing",
  status: "Completed",
  cover:
    "src/rssp/pi.jpg",
  tags: ["Raspberry Pi", "MCP3008", "Ultrasonic", "Servo"],
  desc:
    "Raspberry Pi–based system that fuses analog + distance readings to command a servo motor in real time, illustrating core embedded control concepts.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          A Raspberry Pi reads an analog potentiometer (via MCP3008) and an HC-SR04 ultrasonic sensor,
          then drives an SG90 servo with real-time logic. A limit switch provides homing/restart, and LEDs
          indicate system states. Behavior is verified across repeated cycles for consistent operation.
        </p>
        <ul>
          <li>Homing with limit switch, then manual angle control via potentiometer</li>
          <li>Obstacle detection within ~15&nbsp;cm pauses/restricts servo motion</li>
          <li>LED feedback for status and error conditions</li>
          <li>Clean re-initialization on restart</li>
        </ul>
      `
    },
    {
      heading: "Components",
      content: `
        <ul>
          <li>Raspberry Pi 4B</li>
          <li>MCP3008 ADC</li>
          <li>SG90 servo motor</li>
          <li>HC-SR04 ultrasonic sensor</li>
          <li>Potentiometer, limit switch</li>
          <li>LEDs (red/green), resistors, jumper wires, breadboard</li>
        </ul>
      `
    },
    {
      heading: "Demo Video",
      videos: [
        "https://www.youtube.com/embed/Ezl3cftk74o"
      ]
    }
  ]
};
window.PROJECTS = window.PROJECTS || {};

window.PROJECTS["shibuya-traffic-light"] = {
  title: "Shibuya Traffic Light System",
  status: "Completed",
  cover:
    "src/de/stl.jpg",
  tags: ["STM32", "State Machine", "Embedded", "Simulation"],
  desc:
    "Microcontroller-based simulation of Shibuya Crossing using LEDs and button inputs, showcasing a real-time state machine with pedestrian phases and system open/close.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          A state-driven traffic controller inspired by Shibuya Crossing. The system coordinates vehicle
          and pedestrian LEDs through explicit phases, with button presses governing transitions and
          manual overrides. Core focus: real-time logic, debounced inputs, and clear user feedback.
        </p>
        <ul>
          <li><strong>DEFAULT</strong> — Idle layout (Green/Red); safe reset landing state</li>
          <li><strong>TRAFFIC_LIGHT_SEQUENCE</strong> — Synchronized vehicle lights; ends with blinking amber</li>
          <li><strong>BLINK_PG</strong> — Pedestrian crossing (Red→Green); onboard LED indicates crossing window</li>
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
          <li>14× resistors (LED current-limiting)</li>
          <li>Jumper wires, breadboard</li>
        </ul>
      `
    },
    {
      heading: "Demo Video",
      videos: [
        "https://www.youtube.com/embed/o-3O5DQ864w"
      ]
    }
  ]
};
window.PROJECTS = window.PROJECTS || {};

// Naruto Jutsu Detection with Facial Recognition
window.PROJECTS["handsign"] = {
  title: "Naruto Jutsu Detection with Facial Recognition",
  status: "Completed", // shows green badge automatically
  cover:
    "src/naruto/naruto-shippuden-manga-cover.jpg",
  tags: ["Computer Vision", "Hand Gesture", "Facial Recognition", "PyQt5", "MediaPipe"],
  desc:
    "A fusion of hand-sign detection (Naruto-style gestures) and facial expression recognition to trigger actions via a playful PyQt5 interface.",

  sections: [
    {
      heading: "Project Overview",
      content: `
        <p>
          We developed a multimodal interface that recognizes Naruto hand signs alongside facial expressions.
          Actions are triggered when both are detected simultaneously, enabling inclusive and intuitive control.
        </p>
        <ul>
          <li>Recognizes hand signs (e.g. Bird, Snake, Shadow Clone)</li>
          <li>Detects facial expressions (smile, surprise, neutral)</li>
          <li>Combines signals for robust command triggering</li>
          <li>PyQt5 interface showing status and live video feed</li>
        </ul>
      `
    },
    {
      heading: "Hand Seals",
      img: "src/naruto/Hand.jpg"

    },
    {
      heading: "Implementation Details",
      content: `
        <p>
          The pipeline uses MediaPipe to detect hand landmarks, then classifies sign poses.
          Facial expressions are analysed using a pretrained CNN. When both gesture and expression
          match pre-defined triggers, the GUI executes an action like zoom, snapshot, or other commands.
        </p>
      `
    },
    {
      heading: "Demo Video",
      video: "https://youtu.be/sXeTo-7n7YI"
    }
  ]
};