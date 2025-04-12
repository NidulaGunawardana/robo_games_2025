# 🔥 Round 2 – RoboGames 2024 University Category (Elimination Round)

This directory contains the simulation code and world file for **Round 2 (Elimination Round)** of RoboGames 2024 University Category. The challenge is based on a fire rescue scenario, where the robot must rescue survivors from a hazardous maze environment.

---

## 🎯 Mission Overview

**Scenario:**  
The Faculty of Robo Games is on fire! Your robot must rescue **three survivors** trapped inside a burning maze.

### 🛠 Objectives

- Navigate a **5m × 5m maze** with predefined wall spacing.
- **Avoid fire pits** (danger zones) which apply damage over time:
  - 🔴 **Red Zone**: Damage 40
  - 🟠 **Orange Zone**: Damage 10
  - 🟡 **Yellow Zone**: Damage 0
- **Find and rescue 3 survivors** (marked as green squares).
  - Reward: +20 points each.
  - Rescue by staying in the square for **3 consecutive seconds**.
- Return to the starting position after completing all rescues.
- Start with **100 health points**. If it reaches zero, the robot is disqualified.

---

## 📐 Arena & Robot Specs

### Arena
- Maze Size: **5m × 5m**
- Wall dimensions: **Height: 0.1m**, **Thickness: 0.01m**, **Length: multiples of 0.25m**
- Fire pits are positioned throughout the maze.

### Robot
- Must be **custom-built** in Webots (no prebuilt robots).
- Max dimensions: **0.25m × 0.25m × 0.25m**
- No wall climbing or vision beyond walls allowed.

---

## 🧮 Evaluation Criteria

- ✅ **Base Score**: 100 (only if at least one survivor is rescued).
- ➕ **+20 points per survivor**.
- 🏁 **Tiebreakers**:
  1. Fastest completion time
  2. Code quality (if time and score are equal)
- ❌ Robot disqualified if health reaches 0 or task is incomplete.

---

## 📹 Simulation

[![Watch Demo](demo_thumbnail.jpg) 

