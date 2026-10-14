# Motor Speed Control with a PI Controller and Anti-Windup

Embedded Real-Time Control laboratory (Lab 3) — M.Sc. Control Systems Engineering, University of Padova,
course held by Prof. Alberto Morato. Platform: the course's **T-Bot** (STM32F767, bare-metal C), two DC motors with
quadrature encoders driven through a PWM bridge.

**One-page overview (PDF): [`docs/project-overview.pdf`](docs/project-overview.pdf)** ·
**Submitted report (PDF): [`report/Embedded-Report-lab3.pdf`](report/Embedded-Report-lab3.pdf)**

---

## Purpose of the lab

Implement a **PID controller for the robot drive motors**: measure the wheel speed from the quadrature encoders,
close the loop on the motor voltage, command a reference speed from the on-board keypad, and stream telemetry so the
response can be analysed and tuned in MATLAB.

## What I implemented

### 1. Speed measurement from the encoders

The TIM3/TIM4 encoder counters are read every control period and differenced, with explicit overflow/underflow
handling so direction and magnitude stay correct:

```c
float ReadCurrentSpeed(int motorNumber){
    uint32_t CurrentCount = 0;
    int32_t  DiffCount    = 0;
    static uint32_t PreviousCount = 0;

    switch (motorNumber){
        case 1:
            CurrentCount = __HAL_TIM_GET_COUNTER(&htim3);   // encoder of motor 1
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
                if (CurrentCount <= PreviousCount)  DiffCount = CurrentCount - PreviousCount;              // no underflow
                else DiffCount = -((TIM3_ARR_VALUE + 1) - CurrentCount) - PreviousCount;                   // underflow
            } else {
                if (CurrentCount >= PreviousCount)  DiffCount = CurrentCount - PreviousCount;              // no overflow
                else DiffCount = ((TIM3_ARR_VALUE + 1) - PreviousCount) + CurrentCount;                    // overflow
            }
            PreviousCount = CurrentCount;
            break;
        ...
```

### 2. Digital PI controller with anti-windup

The control law runs in the timer period-elapsed callback at a fixed sampling time of **10 ms**, with the error
expressed in rad/s and a conditional-integration anti-windup on the integral term:

```c
#define TS     0.01          // 10 ms sampling time
#define VBATT  12.0          // battery voltage
#define RPM2RADS (2*M_PI/60)
#define P_gain 0.871
#define I_gain 7.5563

float calculateControlSignal(float currentSpeed, float referenceSpeed, float* I_term, int motorNumber){
    float error   = (referenceSpeed - currentSpeed) * RPM2RADS;   // rad/s
    float P_term  = P_gain * error;
    /* integral action per motor (state kept in I_term_1 / I_term_2) */
    *I_term = *I_term + I_gain * TS * error;

    float Output_temp     = P_term + *I_term;                     // volts
    float Output_clamped  = clamp(Output_temp);                   // saturate to +/-6 V

    /* Anti wind-up: drop the integral step only if the output is saturated
       AND the error would drive it further into saturation */
    bool is_saturated       = (Output_clamped != Output_temp);
    bool worsens_saturation = (error * Output_temp > 0);
    if ((is_saturated) && (worsens_saturation)) *I_term = *I_term - I_gain * TS * error;

    return Output_clamped;
}
```

The control signal (limited to ±6 V) is scaled to PWM duty against the 12 V battery and the timer ARR value, and
applied through two drive modes I implemented: **`ForwardCoast`** (single-channel PWM) and **`ForwardBreak`**
(complementary PWM, i.e. active braking).

### 3. Operator interface and telemetry

The reference speed is typed on the robot keypad (up to three digits followed by `#`, decoded with `atoi`) and handled
through the interrupt-driven keypad buffer. The control signal and the measured quantities are streamed through the
course datalogger (`data.u1` and friends) and plotted in MATLAB for tuning.

### Lab 4 — line following on the same base

Lab 4 reuses exactly this motor-control implementation and adds the line-sensor handling: storing the sensor pattern
(`StoreBinary`) and computing the lateral error used by the steering law. Lab 4 has no separate report; its
implementation lives in the same code base and is documented here.

## Results

| From the report | |
|---|---|
| ![Report page](docs/images/report-page-2.png) | ![Report page](docs/images/report-page-3.png) |

The speed loop tracks the commanded reference on both motors; the measured response and control signal were recorded
with the datalogger and analysed in MATLAB. The report documents the controller structure, the gains, the anti-windup
logic and the tracking behaviour.

## Repository contents

| Path | Content |
|---|---|
| `report/Embedded-Report-lab3.pdf` | My submitted Lab 3 report |
| `src/user_code_main.c` | The `USER CODE` sections of the `main.c` I submitted — speed measurement, PI controller with anti-windup, `ForwardCoast`/`ForwardBreak`, keypad command handling, telemetry |
| `docs/project-overview.pdf` | One-page English overview of the lab |
| `docs/images/` | Pages extracted from the report for quick viewing |

> **Scope note:** `src/user_code_main.c` collects only the CubeMX `USER CODE` sections of my submitted file. The
> surrounding project — the course's T-Bot framework and the STM32 HAL — is the course's material and is deliberately
> **not** redistributed; read the file together with the report.

## Skills demonstrated

`Digital control implementation (PI, fixed sampling time, anti-windup)` · `quadrature encoder interfacing and
overflow-safe speed estimation` · `PWM motor drives including active braking` · `embedded real-time scheduling
(timer-interrupt control loop)` · `measurement, logging and tuning with MATLAB` · `bare-metal C on ARM Cortex-M` ·
`technical reporting`

## Licence

Code and text are mine, released under the [MIT licence](LICENSE); the report is © Kiavash Rabbani Soudabi.
