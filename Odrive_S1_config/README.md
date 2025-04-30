
| File | Parameter Changes | Description |
|------|-----------------------|----------------|
| `comp24_odrive_config.json` |N/A| Used in 2024 comp. Worked well on the test stand, but the robot would not start moving on the ground unless commanded ≥ 0.4 m/s. |
| `feb15_25_config.json` |`vel_gain` increased from 0.01 to 0.014<br>`vel_integrator_gain` increased from 0 to 0.0014 | Reduced over-current errors and gave better straight line tracking at low speeds. |
