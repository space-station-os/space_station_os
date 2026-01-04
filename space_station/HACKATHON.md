
# Space Station OS – NOVA

With gpt-oss-20b, NOVA transforms the space station into an AI-enabled habitat — a copilot that interprets telemetry, explains anomalies, and helps astronauts focus on the mission instead of the data.

## Inspiration

Running a space station is a constant balancing act — astronauts and engineers juggle life support, thermal management, power, and communications under tight resource and time constraints. While simulations and control software exist, they are often technical, fragmented, and inaccessible to non-experts.

With the arrival of large open-source language models like **gpt-oss-20b**, we saw an opportunity to bridge that gap. What if astronauts could ask the station questions in plain language, *“How much oxygen do we have left for 4 crew members?”* or *“Which thermal nodes are overheating?”*,and get actionable answers, instead of parsing raw telemetry or complex logs?

This became the inspiration for **Space Station OS – NOVA**, where we integrated gpt-oss-20b into our open-source space station simulator. NOVA acts as an **AI mission assistant**, interpreting subsystem data, monitoring diagnostics, and providing human-readable explanations of system health.

Our goal is to show that **AI can democratize space operations** — lowering the barrier from raw engineering details to astronaut-friendly insights, and paving the way toward **software-defined, AI-augmented habitats** in long-duration missions.

---

## What it does

**Space Station OS – NOVA** transforms raw telemetry and subsystem outputs into **intuitive, AI-driven insights** using gpt-oss-20b. Instead of parsing through logs or dashboards, astronauts and mission operators can interact with the space station in natural language.

NOVA acts as a **mission assistant** that:

* **Understands telemetry** – Subscribes to ROS 2 topics (oxygen storage, CO₂ buildup, heat flux) and translates them into human-readable summaries.
* **Answers astronaut queries** – Responds to natural-language questions like *“Which nodes are running hottest?”* or *“How much water reserve is left?”*.
* **Explains anomalies** – Interprets diagnostics and explains *why* a failure occurred and what it means for the crew.
* **Bridges perspectives** – Engineers see structured ROS 2 data; astronauts see actionable insights, powered by gpt-oss-20b.

In short: NOVA uses the language model as a **reasoning and communication layer** between robotic middleware (ROS 2) and the humans depending on it.

---

## How we built it

We built **Space Station OS – NOVA** by combining our modular ROS 2–based simulator with the **gpt-oss-20b model via the NVIDIA API**, creating a real-time AI mission assistant.

**Architecture highlights:**

1. **Subsystem data streams** – Telemetry from ECLSS and Thermal Control is published as ROS 2 topics (oxygen, CO₂, water, thermal nodes, diagnostics).
2. **ROS–AI bridge** – A Python agent subscribes to these topics, summarizes messages, and sends them to gpt-oss-20b through the NVIDIA API.
3. **LLM-powered reasoning** – The model interprets telemetry and diagnostics, returning astronaut-friendly explanations or direct answers.
4. **Interactive interface** – A PyQt GUI lets astronauts “chat with the station” while viewing real-time telemetry.
5. **Diagnostics integration** – The agent proactively explains failures (e.g., cooling issues, abnormal CO₂ rise) rather than waiting for queries.

By fusing **ROS 2’s real-time control** with **gpt-oss-20b’s reasoning via NVIDIA**, we created an intelligent layer that makes space station data **accessible, conversational, and actionable**.

---

## Challenges we ran into

* **Bridging ROS 2 and LLMs** – Raw telemetry is not language-friendly; we needed a preprocessing layer for gpt-oss-20b prompts.
* **Latency vs. usability** – Using the NVIDIA API required balancing response time with detail to keep interactions “real-time.”
* **Context management** – Thousands of messages per second meant carefully selecting what data to send to the model.
* **Trust and explainability** – Astronauts need clear *why* and *what next* answers, not vague summaries — tuning prompts was key.

---

## Accomplishments that we’re proud of

* Integrated **gpt-oss-20b via the NVIDIA API** into a live ROS 2 simulation.
* Built an **AI mission assistant (NOVA)** that astronauts can query in natural language.
* Demonstrated **proactive fault explanation**, where the AI reacts to subsystem failures with context-aware insights.
* Developed a **PyQt GUI with integrated AI chat**, combining technical telemetry with conversational intelligence.
* Proved that **open-source AI + robotics middleware** can lower the barrier to simulating and controlling complex space infrastructure.

---

## What we learned

* **LLMs bridge the gap** between engineering data and astronaut communication, but require careful context curation.
* AI in mission control is about **augmenting humans**, not replacing them, making telemetry interpretable and actionable.
* **Failure handling builds trust** — AI must explain anomalies as well as normal ops.
* Real-time AI requires balancing **efficiency, clarity, and trustworthiness**.

---

## What’s next for Space Station OS – NOVA

This hackathon proved that AI can support astronauts by making **life support and thermal control data conversational**. But this is just the beginning.

Next, we’ll expand NOVA’s reach across other subsystems:

* **GNC (Guidance, Navigation, and Control)** – AI driven control architecture for handling GNC subsystems. 
* **EPS (Electrical Power System)** – Monitoring solar arrays, batteries, and power distribution with AI-driven insights.
* **Propulsion systems** – Translating engine telemetry and maneuvers into astronaut-friendly recommendations.
* **Robotic systems** – Assisting astronauts with robotic arm operations, payload handling, and autonomous repairs.

Our long-term vision is clear: **space stations of the future will be AI-enabled habitats**, where models like gpt-oss-20b serve as copilots, advisors, and safety nets , ensuring astronauts focus on exploration while AI manages complexity.

