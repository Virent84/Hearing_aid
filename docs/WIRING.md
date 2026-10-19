# Wiring – ESP32-S3 + 4× INMP441

## ESP32-S3 pins
| ESP32-S3 pin | Goes to |
|---|---|
| GPIO 4 | SCK on all 4 mics (clock net) |
| GPIO 15 | **Jumper wire to GPIO 4** |
| GPIO 5 | WS on all 4 mics (word-select net) |
| GPIO 16 | **Jumper wire to GPIO 5** |
| GPIO 6 | SD on M1 and M2 (data line A) |
| GPIO 7 | SD on M3 and M4 (data line B) |
| 3V3 | VDD on all 4 mics, plus the L/R pins of M2 and M4 |
| GND | GND on all 4 mics, plus the L/R pins of M1 and M3 |

## Per mic
| Mic | Position | SCK | WS | SD | L/R | VDD | GND |
|---|---|---|---|---|---|---|---|
| M1 | +x (0°)   | GPIO 4 net | GPIO 5 net | GPIO 6 | GND | 3V3 | GND |
| M2 | +y (90°)  | GPIO 4 net | GPIO 5 net | GPIO 6 | 3V3 | 3V3 | GND |
| M3 | −x (180°) | GPIO 4 net | GPIO 5 net | GPIO 7 | GND | 3V3 | GND |
| M4 | −y (270°) | GPIO 4 net | GPIO 5 net | GPIO 7 | 3V3 | 3V3 | GND |

## Why two I2S ports and jumpers
One I2S port reads only two mics (left + right slot). To get four, the firmware uses both ESP32-S3 I2S ports on the
same clocks: I2S0 is the clock **master** (drives GPIO15 = BCLK, GPIO16 = WS, reads GPIO6); the jumpers carry those clocks
onto the mic net (GPIO4/GPIO5), where I2S1 listens as a **slave** (reads GPIO7). Do not remove the jumpers.

## Geometry (must match config.json `array`)
Mics on a circle of 5 cm radius, counter-clockwise seen from above: M1 = 0°, M2 = 90°, M3 = 180°, M4 = 270°.
Angle 0° is the direction of M1. If your board is laid out differently, edit `array.mic_angles_deg` / `array.radius_m`.

## Cables
* ESP32-S3 **native "USB" port** → Pi 5 USB port (data cable, not charge-only). This carries the audio.
* The other port (UART/COM) is optional and only for flashing/power.
* Keep the SCK/WS wires short and of similar length; add 100 nF + 10 µF close to the mics' VDD if you see noise.
