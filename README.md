# Autumn-2026 Assignment 4 — Arm Control

## Part A: Studying and technical implementation — 4 points


### Overview

This assignment focuses on how autonomous cyber-physical systems execute actions in the physical world through control and actuation. Students implement a control pipeline for a robotic arm in a Webots and ROS 2 environment, exploring how discrete commands are translated into continuous motion. The task covers forward kinematics, PID control for trajectory tracking, and a state-machine-based pick-and-place behavior, highlighting the interaction between modeling, control, and physical constraints. Through this, students gain practical understanding of how perception and decision-making are realized as stable, controlled physical actions.

Setup is familiar from previous assignments:
> **Webots R2025a** runs on your host OS.  
> **ROS2 Jazzy** runs inside Docker.  
> Your code lives in `src/` — edit on host, build and run in container.

---

## Quick start

### 1. First-time setup

```bash
cp .env.example .env
# Linux:        uncomment NETWORK_MODE=host and WEBOTS_HOST=localhost
# Mac/Windows:  leave as-is

docker compose build   # ~5 min first time
```

### 2. Open Webots

1. Open Webots R2025a → **File → Open World** → `worlds/arm_assignment.wbt`
2. Press **Play ▶** before launching the driver

> ⚠️ If you see `"retrying for another 50 seconds..."` — just press Play in Webots.  
> The driver reconnects automatically.

### 3. Start the container and build

```bash
docker compose run --rm ros2 bash
cb && src
```

| Alias | What it does |
|-------|-------------|
| `cb`  | `colcon build --symlink-install` |
| `src` | `source install/setup.bash` |
| `rt`  | `ros2 topic list` |

### 4. Start the driver (always Terminal 1)

```bash
ros2 launch arm_control_pkg arm.launch.py
```

Confirm the driver loaded — you should see:
```
[YouBotArmDriver] Driver ready — all topics live
```

Then check topics in a second terminal:
```bash
rt | grep youBot
```

You should see `/youBot/arm_joint_1/sensor` through `arm_joint_5/sensor` and `/youBot/end_effector_position`. If you see nothing, the driver is not running.

---

## Task 1 — Forward Kinematics

**Learning goal:** Understand how joint angles map to end-effector position.

```bash
# Terminal 1 — driver (keep open for all tasks)
ros2 launch arm_control_pkg arm.launch.py

# Terminal 2 — FK explorer
ros2 run arm_control_pkg task1_fk
```

Edit `joint_index` and `target_angle` at the bottom of `task1_fk_explorer.py`, then rebuild and run again for each experiment:

```bash
# After editing on your host:
cb && src
ros2 run arm_control_pkg task1_fk
```

### What to record

Fill in this table in your report (run the node once per row):

| Joint | Start (rad) | End (rad) | EE start (x, y, z) | EE end (x, y, z) |
|-------|-------------|-----------|---------------------|------------------|
| 1 | 0.0 | 1.0 | | |
| 1 | 0.0 | 2.0 | | |
| 2 | 0.0 | 0.5 | | |
| 2 | 0.0 | 1.0 | | |
| 3 | 0.0 | -1.0 | | |
| 4 | 0.0 | 1.0 | | |

### Hints

- **Wait for values to stop changing** before recording — the arm takes 1–2 seconds to settle after a command.
- **What the axes mean:** `x` is forward (away from the robot body), `y` is sideways, `z` is up. All values are relative to the arm's base joint.
- **Which joints do what:** Joint 1 rotates the base (changes x and y, z stays the same). Joints 2 and 3 control how far forward and how high the arm reaches. Joints 4 and 5 control wrist angle.
- **The workspace question (Task 1b):** Think about what the arm is physically attached to, and what that means about positions directly below or behind it.

---

## Task 2 — PID Controller (Sine Wave Tracking)

**Learning goal:** Implement and tune a PID controller. Observe how each term affects tracking.

Joint 2 follows a continuously moving sine wave target. Your controller must keep the arm tracking it. Watch the arm in Webots — tracking visibly improves with each term you add.

### Workflow — one run at a time

```bash
# Terminal 1 — driver (keep open across all runs)
ros2 launch arm_control_pkg arm.launch.py

# Terminal 2 — your PID controller
ros2 run arm_control_pkg task2_pid

# Terminal 3 — data logger (saves plot automatically on Ctrl+C)
ros2 run arm_control_pkg data_logger --ros-args -p label:=P_only
```

Run for ~60 seconds (3 full cycles), then Ctrl+C both Terminal 2 and Terminal 3.

Repeat with labels `PD` and `PID`. When you stop the logger it automatically saves the comparison plot to:
```
src/arm_control_pkg/logs/task2_plot.png
```
Open that file directly on your host machine — no display needed in Docker.

### What to implement

Open `task2_pid_joint.py` and find the TODO section in `_control_loop`. Implement in order:

**Task 2a — P term only** (`Ki=0, Kd=0`):
```python
correction = Kp * error
```
The arm will start following the sine wave but lag behind the target.

**Task 2b — Add D term** (keep your Kp):
```python
derivative = (error - self.prev_error) / self.dt
correction = Kp * error + Kd * derivative
```
The lag should reduce. Watch the arm in Webots become more responsive.

**Task 2c — Add I term** (keep your Kp and Kd):
```python
self.integral += error * self.dt
correction = Kp * error + Kd * derivative + Ki * self.integral
```
Any remaining constant offset between arm and target should disappear.

### Hints

- **With `correction = 0.0`** (before you implement anything), the arm sends `target + 0 = target` — it will move, but it is not using any feedback. The error will be non-zero and visible in the log. This is expected.
- **Starting gain values:** `Kp = 1.0` is a good starting point. If the arm barely moves, increase it. If it oscillates wildly, decrease it.
- **Keep Kd small** (0.01–0.05). The D term amplifies sensor noise — too large and the arm will vibrate rather than dampen.
- **Keep Ki small** (0.01–0.1). The I term accumulates over time — too large causes slow growing oscillations.
- **Each run must be independent** — stop Terminal 2 (controller) and Terminal 3 (logger) with Ctrl+C before starting the next run. Use a different `label` each time so the plot shows all three curves.

---

## Task 3 — Pick and Place State Machine

**Learning goal:** Implement a discrete state machine and observe the gap between discrete decisions and continuous arm dynamics.

The arm must complete this sequence:
```
HOME → APPROACH → GRASP → CLOSE_GRIP → LIFT
```

```bash
# Terminal 1 — driver
ros2 launch arm_control_pkg arm.launch.py

# Terminal 2 — state machine
ros2 run arm_control_pkg task3_pick
```

### What to implement

Open `task3_pick_place.py` and implement the two methods:

**`_send_joint_commands(state)`** — publish target angles to all 5 joints:
```python
for i in range(5):
    msg = Float64()
    msg.data = STATE_ANGLES[state][i]
    self.joint_pubs[i].publish(msg)
```

**`_check_transition(state)`** — return `True` only when safe to advance:
```python
# Condition 1: all joints close enough to target
all_close = all(
    abs(self.current_angles[i] - STATE_ANGLES[state][i]) < POSITION_THRESHOLD
    for i in range(5)
)
# Condition 2: arm has held that position for DWELL_TIME seconds
if all_close:
    if self.dwell_start is None:
        self.dwell_start = time.time()
    return time.time() - self.dwell_start >= DWELL_TIME
else:
    self.dwell_start = None
    return False
```

### Run it twice

Edit `DWELL_TIME` at the top of the file:

| Run | Setting | Expected behaviour |
|-----|---------|-------------------|
| 1 | `DWELL_TIME = 0.0` | Arm skips states before finishing motion |
| 2 | `DWELL_TIME = 1.5` | Each state completes before the next begins |

Rebuild between runs: `cb && src`

### Adjusting STATE_ANGLES for your setup

The provided `STATE_ANGLES` are approximate. **The arm may not reach the box exactly** — this is normal and part of the task. To improve the arm's reach:

1. **Use Task 1's FK explorer** to find angles that position the EE near your box:
   - Run `task1_fk` with different joint angles
   - Look at the EE `x` and `z` values — aim to match the box position visually
   - Joint 2 (shoulder, `arm_joint_2`) primarily controls height (z)
   - Joint 3 (elbow, `arm_joint_3`) primarily controls forward reach (x) — use negative values to extend forward
   - Joint 4 (wrist, `arm_joint_4`) fine-tunes the approach angle

2. **Update `STATE_ANGLES`** in `task3_pick_place.py` with your found angles:
   - `APPROACH` should position the EE a few centimetres **above** the box
   - `GRASP` should position the EE at the box level
   - `LIFT` should raise the arm back up

3. **You can also move the box** in the Webots Scene Tree to a position that's easier to reach — click the TARGET_BOX node, change its `translation` field.

> The arm's maximum forward reach is approximately **0.46 m** from its base joint (which is offset ~0.16 m from the robot centre). The box must be within this range.

---



## Troubleshooting

**No `/youBot/...` topics after launch**  
→ Driver not loaded. Check Webots is open and simulation is playing ▶.

**`angle=0.000` never changes in task2**  
→ Driver must be running in a separate terminal. `task2_pid` alone does nothing — it needs the driver running in Terminal 1 simultaneously.

**Arm doesn't move in Task 3 / stuck on HOME forever**  
→ `_send_joint_commands` is not implemented yet (it's `pass`). Implement it first — without it, no commands are published and the arm never moves.

**Segmentation fault when stopping the driver (Ctrl+C)**  
→ Known bug in `webots_ros2_driver`. Safe to ignore — driver shuts down correctly.

**`rcl_shutdown already called` on Ctrl+C**  
→ Cosmetic only. Safe to ignore.

**Changes not taking effect**  
→ Run `cb && src` after every edit. Always rebuild before testing.

**Container can't connect to Webots (Mac/Windows)**  
→ Check `.env` has `WEBOTS_HOST=host.docker.internal`.

**Mac Apple Silicon — build fails**  
→ Docker Desktop → Settings → General → enable "Use Rosetta for x86/amd64 emulation".



---

## Part B: Learning process, use of AI tools, and self-reflection — 4 points

Part B is an equally important part of the assignment as the technical implementation in Part A. The purpose is to reflect on **what you learned, how you worked, and how AI or other tools and resources affected your learning and problem-solving process**.

You are **not assessed based on how much AI you use**. You may use AI extensively, selectively, or not at all. Instead, you are assessed based on the quality of your reflection, your critical evaluation of your working and learning process, and your demonstrated understanding and ownership of the final solution.

### If you did not use AI

Using AI is **not required and does not affect the maximum number of points available**.

If you chose not to use AI, explain why and focus instead on **how you learned and solved the assignment independently**. Describe the resources, tools, documentation, experimentation, debugging, discussions, or other approaches you used to understand the technical concepts and overcome problems.

The same assessment criteria apply: the important thing is to demonstrate and critically reflect on **how you worked, what you learned, and how you evaluated the correctness of your solution**.

In your presentation, address the following four areas:

### 1. What did you learn?

Describe the most important things you learned while completing Part A.

Focus especially on your **technical learning**. For example, in this assignment you might discuss what you learned about sensor data, noise and drift, filtering, sensor fusion, ROS2 topics, or implementing and debugging the provided code.

Do not simply describe what you did. Explain **what you understand now that you did not understand before the assignment**.

**If you used AI:** Explain how AI helped you understand new concepts, code, technologies, or problems and how your understanding developed through this interaction.

**If you did not use AI:** Explain how you developed this understanding. For example, did you study course materials or documentation, search for examples, experiment with the system, inspect code, discuss problems with others, or learn through trial and error?

### 2. How did you solve problems and use AI, tools, and other resources?

Describe **how you approached the assignment and solved the technical problems you encountered**.

**If you used AI**, explain the significant ways in which AI contributed to your work. For each important use, consider:

- what you were trying to accomplish or understand;
- which tool and model you used;
- why you decided to use AI for this particular purpose;
- how you used it; and
- how useful the result was.

You do not need to report every individual prompt. Instead, focus on the **most important ways in which AI influenced your work**.

You must nevertheless transparently identify the AI tools and models you used in Parts A and B, including the model/version and relevant mode or settings when this information is available.

For example:

> I used ChatGPT with GPT-X in thinking mode to understand why integrating a small constant gyroscope bias produces an increasing heading error. I then used it to help interpret the results produced by my implementation.

**If you did not use AI:** Describe the corresponding problem-solving process. What resources or tools did you use instead? How did you find the information you needed? How did you approach debugging or unfamiliar technical concepts? Explain why you chose these approaches and how useful they were.

### 3. Critically evaluate your tools and problem-solving process

Choose at least **one concrete example of a problem, challenge, or learning situation** from the assignment and analyze how you addressed it.

**If you used AI**, this could be a situation where AI:

- significantly helped you;
- produced an incomplete or incorrect answer;
- suggested an approach that needed modification; or
- provided information whose correctness you needed to verify.

Explain what AI suggested, whether the suggestion was useful, **how you determined whether you could trust it**, and how you tested, modified, improved, or rejected the suggestion.

**If you did not use AI:** Select a corresponding technical challenge and explain how you investigated and solved it. For example, describe how you used documentation, course materials, experimentation, debugging, testing, online resources, or discussions with others. Consider whether your initial approach worked, how you evaluated different alternatives, and how you verified that the final solution was correct.

The purpose is to demonstrate that you can **critically evaluate information, tools, and proposed solutions rather than simply accepting them**.

### 4. Reflect on your own learning and working process

Finally, reflect on your overall experience and how you approached learning during the assignment.

Consider questions such as:

- What was the most difficult part of the assignment?
- What did you understand better after completing it?
- Which problem-solving or learning approaches worked particularly well?
- Which approaches did not work well?
- How did you verify that you understood the technical concepts rather than merely producing a working solution?
- If you completed the assignment again, what would you do differently?

**If you used AI:** Also consider whether AI made you more productive or helped you learn, whether it introduced additional problems or confusion, and whether there were situations where solving or studying something without AI was more useful.

**If you did not use AI:** Reflect on how effective your chosen learning and problem-solving strategies were. Consider whether documentation, experimentation, debugging, course materials, discussions, or other resources supported your learning effectively and whether another approach could have made your work or learning more effective.

Your reflection should demonstrate that **you understand and take responsibility for the work you submitted**, regardless of whether or how much AI contributed to the process.

---


## Returning instructions

The assignment must be submitted in video format. Create a video that reports Parts A and B, demonstrating that you have successfully set up the environment, explaining how you learned to use your codebase, and presenting your self-reflection as well as describing how you used AI in the assignment.

In the video, use software such as Microsoft Teams to record your screen while presenting and recording your voice. In the recording, you should demonstrate that your environment is running correctly and show the most essential parts of your codebase. In addition, you should use PowerPoint, Google Slides, or a similar tool to document Part B and present it in the video.

This process will not only teach you how to present your work to others, but it will also help facilitate peer learning and support among students. Although in this first assignment there is only a small amount of technical implementation to report, you will learn the submission procedure that will be used for the rest of the assignments, which will be much more implementation-oriented.

We recommend using Microsoft Teams, as it allows you to record both your screen and voice. The recordings are automatically uploaded to SharePoint, which makes it easy to share your recording later for the peer review assignment. Alternatively, you may use other software such as QuickTime Player or OBS to record your video and then upload the recording to SharePoint.

**Important:** You must ensure that your recording is accessible to others who have the link. So, via Sharepoint user interface in your browser (see below) define the shared settings so that anyone who has the link can access the file for maximum number of days. Finally, you should ensure for example in privacy mode or another browser that the link is truly accessible without login.

![create_sharelink](https://github.com/TEKS4430/Autumn-2026_Assignment_1/blob/main/screenshots/accessrights.png)

<p align="center">
<img src="https://github.com/TEKS4430/Autumn-2026_Assignment_1/blob/main/screenshots/link_settings.png" width=50% height=50%>    
</p>
