# I2C codes
## Single Purpose
| I2C Code #  | Visual Output | Purpose | Decorative/Functional |
|---|---|---|---|
| 0 | Black/All Off | Clear all LEDS | Decorative/Functional (Initial state)
| 1 | Red Pulse | Before match start if red team | Decorative
| 2 | Blue Pulse | Before match start if blue team | Decorative
| 3 | Purple Pulse | Requesting cube | Functional
| 4 | Yellow Pulse | Requesting cone | Functional
| 5 | Purple Chase | In cube mode while arm extended | Functional/Decorative
| 6 | Yellow Chase | In cone mode while arm extended | Functional/Decorative
| 7 | Purple Solid | In possession of cube | Decorative
| 8 | Yellow Solid | In possession of cone | Decorative

## Multi Purpose
| I2C Code #  | Visual Output | Purpose | Decorative/Functional |
|---|---|---|---|
| 9 | Red Solid | Out of range I2C code / General Error | N/A
| 10 | Green Solid | Positive Outcome / Success | N/A
| 11 | Rainbow | Robot display / Celebratory outcome | Decorative

## Effect Usage Flowchart
```
                   ┌─────────────────────────┐
                   │                         │
┌─────────────────►│ Robot Init              │
│                  │                         │
│                  └────────────┬────────────┘
│                               │
│                               │
│                  Blue Team ◄──┴──► Red Team
│                    FX#2              FX#1
│                      │                │
│                  ┌───▼────────────────▼────┐
│                  │                         │
│                  │ Game Start - Req Object │◄─────────────────┐
│                  │                         │                  │
│                  └────┬──────────────┬─────┘                  │
│                       │              │                        │
│                       ▼              ▼                        │
│                   Cone Mode      Cube Mode                    │
│                     FX#4           FX#3                       │
│                       │              │                        │
│                  ┌────┴──────────────┴─────┐                  │
│                  │                         │                  │
│                  │ Arm Extention           │                  │
│                  │                         │                  │
│                  └────┬──────────────┬─────┘                  │
│                       │              │                        │
│                       ▼              ▼                        │
│                   Cone Extend     Cube Extend                 │
│                      FX#6          FX#5                       │
│                       │              │                        │
│                  ┌────┴──────────────┴─────┐                  │
│                  │                         │                  │
│      ◄───────────┤ Object Acquired         ├────────────►     │
│         Success  │                         │    Fail          │
│          [TBD]   └────┬──────────────┬─────┘    [TBD]         │
│          FX#10        │              │          FX#9          │
│                       ▼              ▼                        │
│                   Hold Cone      Hold Cube                    │
│                     FX#8           FX#7                       │
│                       │              │                        │
│                  ┌────┴──────────────┴─────┐                  │
│                  │                         │                  │
│                  │ Lost Possession         ├──────────────────┘
│                  │                         │
│                  └───────────┬─────────────┘
│                              │
│                              ▼
│                  ┌─────────────────────────┐
│                  │                         │
└──────────────────┤ Game End                │
                   │                         │
                   └─────────────────────────┘
```

# Effect Design Overview
- Effects are designed to be as simple as possible to avoid distraction.
- Effects follow a small colour pallete for clarity.
- Effects should avoid flashing faster than 3Hz for more than 3s.
- Core effects are not multi-purpose.
- Effect functions are not blocking as they cause intermittent behaviour on I2C communications. AKA, they do not use `FastLED.delay()` or `delay()`.
- Effects should run indefinitely until requested by RIO.